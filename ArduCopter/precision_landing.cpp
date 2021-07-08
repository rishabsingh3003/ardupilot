//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

static const float MAX_POS_ERROR_CM = 35.0f;  // Maximum possition errors horizontally and vertically for retry locations
static const uint32_t FAILSAFE_INIT_TIMEOUT_MS = 7000;   // Timeout in ms before failsafe measures are started. During this period vehicle is completely stopped to give user the time to take over

Mode::PrecLand_StateMachine Mode::precland_statemachine;

void Copter::init_precland()
{
    copter.precland.init(400);
}

void Copter::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;

    // use range finder altitude if it is valid, otherwise use home alt
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.alt_cm_glitch_protected;
    }

    precland.update(height_above_ground_cm, rangefinder_alt_ok());
}


void Mode::PrecLand_StateMachine::init()
{
    // init is only called ONCE per mode change. So in a particuar mode we can retry only a finite times.
    // The counter will be reset if the statemachine is called from a different mode
    _retry_count = 0;
    // reset every other statemachine
    reset_failed_landing_statemachines();
}

// Reset the landing statemachines. This needs to be called everytime the landing target is back in sight.
// So that if the landing target goes out of sight again, we can start the failed landing procedure back from the beginning stage
void Mode::PrecLand_StateMachine::reset_failed_landing_statemachines()
{
    landing_target_lost_action = Landing_Target_Lost_Action::INIT;
    retry_status = RetryLanding::RETRY_INIT;
    failsafe_initialized = false;
}

// Run Prec Land State Machine. During Prec Landing, we might encouter four scenarios:
// 1. We had the target in sight, but have lost it now. 2. We never had the target in sight and user wants to land.
// 3. We have the target in sight and can continue landing. 4. The sensor is out of range
// This method deals with all of these scenarios
void Mode::PrecLand_StateMachine::update_precland_state_machine()
{
    // grab the current status of Landing Target
    AC_PrecLand::PldState current_state =  copter.precland.get_plnd_target_status();

    switch (current_state)
    {
    case AC_PrecLand::PldState::TARGET_RECENTLY_LOST:
        // we have lost the target but had it in sight atleast once recently
        // action will depend on what user wants
        target_lost_actions();
        break;

    case AC_PrecLand::PldState::TARGET_LOST:
        // we have no clue where we are supposed to be landing
        // let user decide how strict our failsafe actions need to be
        do_failsafe_actions();
        break;

    case AC_PrecLand::PldState::TARGET_OUT_OF_RANGE:
        // The target isn't in sight, but we can't run any fail safe measures or do landing retry
        // Therefore just descend for now, and check again later if retry is allowed
    case AC_PrecLand::PldState::TARGET_FOUND:
    default:
        // no action required, target is in sight
        copter.flightmode->run_land_controller();
        reset_failed_landing_statemachines();
        break;
    }
}


// Target is lost (i.e we had it in sight some time back), this method helps decide on what needs to be done next
// The chosen action depends on user set landing strictness
void Mode::PrecLand_StateMachine::target_lost_actions()
{
    switch (landing_target_lost_action)
    {
    case Landing_Target_Lost_Action::INIT:
    {
        // figure out how strict the user is with the landing
        AC_PrecLand::PldRetryStrictness strictness = copter.precland.get_plnd_retry_strictness();
        if ((strictness == AC_PrecLand::PldRetryStrictness::NORMAL)
            || (strictness == AC_PrecLand::PldRetryStrictness::VERY_STRICT)) {
            // We eventually want to retry landing, but lets descend for sometime and hope the target gets in sight
            // If not, we will retry landing
            landing_target_lost_action = Landing_Target_Lost_Action::DESCEND;
        } else if (strictness == AC_PrecLand::PldRetryStrictness::NOT_STRICT) {
            // User just wants to land, prec land isn't a priority
            landing_target_lost_action = Landing_Target_Lost_Action::LAND_VERTICALLY;
        }
    }
        break;

    case Landing_Target_Lost_Action::DESCEND:
        copter.flightmode->run_land_controller();
        if (AP_HAL::millis() - copter.precland.get_last_precland_output_ms() >= copter.precland.get_min_retry_time_sec()) {
            // we have descended for sometime and the target still isn't in sight
            // lets retry
            landing_target_lost_action = Landing_Target_Lost_Action::RETRY_LANDING;
            retry_status = RetryLanding::RETRY_INIT;
        }
        break;

    case Landing_Target_Lost_Action::RETRY_LANDING:
        // retry the landing by going to the last known horizontal position of the target
        retry_landing();
        break;

    case Landing_Target_Lost_Action::LAND_VERTICALLY:
    default:
        // Just land vertically
        copter.flightmode->run_land_controller();
        break;
    }
}

// Retry landing based on a previously known location of the landing target
void Mode::PrecLand_StateMachine::retry_landing()
{
    if (_retry_count > copter.precland.get_max_retry_allowed() || copter.precland.get_max_retry_allowed() == 0) {
        // we have exhausted the amount of times vehicle was allowed to retry landing
        // do failsafe measure so the vehicle isn't stuck in a constant loop
        do_failsafe_actions();
        return;
    }

    // get the last known location of the landing target
    Vector3f last_known_loc;
    copter.precland.get_last_detected_landing_location(last_known_loc);

    switch (retry_status)
    {
    case RetryLanding::RETRY_INIT:
        // Init the Retry
        _retry_count ++;
        copter.flightmode->land_retry_position(last_known_loc);
        retry_status = RetryLanding::RETRY_IN_PROGRESS;
        // inform the user what we are doing
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Retrying Precision Landing");
        break;

    case RetryLanding::RETRY_IN_PROGRESS:
    {
        // continue converging towards the target till we are close by
        copter.flightmode->land_retry_position(last_known_loc);
        const float dist_to_target_xy = copter.pos_control->get_pos_error_xy_cm();
        const float dist_to_target_z = copter.pos_control->get_pos_error_z_cm();
        if ((dist_to_target_xy < MAX_POS_ERROR_CM) && (fabsf(dist_to_target_z) < MAX_POS_ERROR_CM)) {
            // we have approx reached landing location previously detected
            retry_status = RetryLanding::RETRY_COMPLETE;
            copter.gcs().send_text(MAV_SEVERITY_INFO, "Landing Retry Completed");
        }
        break;
    }

    case RetryLanding::RETRY_COMPLETE:
        // Vehicle has completed a retry, and most likely the landing location still isn't sight
        // we have no choice but to force a failsafe action
        do_failsafe_actions();

        break;
    }
}

// Stop landing (hover) to a stopping location
void Mode::PrecLand_StateMachine::stop_landing()
{
    copter.flightmode->land_retry_position(stopping_pos.tofloat());
}

// Decide what the next action is going to be if we have hit a failsafe
// Failsafe will only trigger as a last resort
void Mode::PrecLand_StateMachine::do_failsafe_actions()
{
    if (!failsafe_initialized) {
        // start the timer
        failsafe_start_ms = AP_HAL::millis();
        // set stopping position
        copter.pos_control->get_stopping_point_xy_cm(stopping_pos.xy());
        copter.pos_control->get_stopping_point_z_cm(stopping_pos.z);
        failsafe_initialized = true;
        copter.gcs().send_text(MAV_SEVERITY_INFO, "Starting Precision Landing Failsafe Measures");
    }
    if (AP_HAL::millis() - failsafe_start_ms < FAILSAFE_INIT_TIMEOUT_MS) {
        // stop the vehicle for atleast a few seconds before going on to the next action
        // this might give user the chance to take over
        stop_landing();
        return;
    }

    // We have waited for some time now, user should either take over from here manually
    // or we will follow the statemachine below
    switch (copter.precland.get_plnd_retry_strictness()) {
    case AC_PrecLand::PldRetryStrictness::VERY_STRICT:
        // user does not want to land on anything but the target
        // stop landing (hover)
        stop_landing();
        break;

    case AC_PrecLand::PldRetryStrictness::NORMAL:
    case AC_PrecLand::PldRetryStrictness::NOT_STRICT:
    default:
        // User wants to prioritize landing over staying in the air
        copter.flightmode->run_land_controller();
        break;
    }
}

#endif
