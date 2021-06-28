#include "AC_PrecLand_State_Machine.h"

const AP_Param::GroupInfo AC_PrecLand_State_Machine::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Precision Land SM enabled/disabled
    // @Description: Precision Land SM enabled/disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AC_PrecLand_State_Machine, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPINFO("TYPE", 1, AC_PrecLand_State_Machine, _type, 1),

    AP_GROUPINFO("FS_ACT", 2, AC_PrecLand_State_Machine, _fs_action, 0),

    AP_GROUPEND
};


AC_PrecLand_State_Machine::AC_PrecLand_State_Machine(AC_PrecLand& precland)
        :
        _precland(precland)
{
    // // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// This function is called when user switches into Landing mode
void AC_PrecLand_State_Machine::init()
{
    _plnd_state = PLD_STATE::PRECISION_LAND_STATE_INIT;
    if (!AP::ahrs().get_relative_position_NED_origin(last_known_pos)) {
        return;
    }

    vertical_run_counter = VERTICAL_RETRY_STATE::START;

}

void AC_PrecLand_State_Machine::plnd_status()
{
    const bool current_status= _precland.target_acquired();
    if (current_status) {
        // target is in sight
        _plnd_state = PLD_STATE::PRECISION_LAND_STATE_ACTIVE;
        init();
    } else {
        _plnd_state = PLD_STATE::PRECISION_LAND_STATE_TARGET_LOST;
    }
    if (last_status && !current_status) {
        // we just lost the landing target
        _plnd_state = PLD_STATE::PRECISION_LAND_STATE_TARGET_LOST;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Target Lost!");
    }
    last_status = current_status;
}

void AC_PrecLand_State_Machine::landing_retry_status()
{
    if (_plnd_state == PLD_STATE::PRECISION_LAND_STATE_ACTIVE) {
        landing_retry_state = LANDING_RETRY_STATE::LANDING_RETRY_NOT_REQUIRED;
        return;
    }

    switch (_type)
    {
    case (uint8_t)LANDING_RETRY_STATE::LANDING_RETRY_VERTICAL:
        {
            const bool success = run_vertical_retry();
            if (!success) {
                landing_retry_state = LANDING_RETRY_STATE::LANDING_RETRY_FAILED;
            } else {
                landing_retry_state = LANDING_RETRY_STATE::LANDING_RETRY_VERTICAL;
            }
            break;
        }

    default:
        break;
    }
}

bool AC_PrecLand_State_Machine::run_vertical_retry()
{
    if (vertical_run_counter == VERTICAL_RETRY_STATE::START) {
        // come up with first target, which is a few meters above the vehicle
        retry_location = last_known_pos;
        // projected location
        retry_location.z -= 10.0f;
        vertical_run_counter = VERTICAL_RETRY_STATE::CLIMB;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Climb starting!");
        return true;
    } else if (vertical_run_counter == VERTICAL_RETRY_STATE::CLIMB) {
        Vector3f current_pos;
        if (!AP::ahrs().get_relative_position_NED_origin(current_pos)) {
            return false;
        };
        if (fabsf(current_pos.z - retry_location.z) < 0.5) {
            // reached first loc
            vertical_run_counter = VERTICAL_RETRY_STATE::REACHED_TOP;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Reached Top!");
        }
        return true;
    } else if (vertical_run_counter == VERTICAL_RETRY_STATE::REACHED_TOP) {
        // come up with second target, which is a few meters below the vehicle
        retry_location = last_known_pos;
        retry_location.z += 10.0f;
        vertical_run_counter = VERTICAL_RETRY_STATE::DESCEND;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Going Down");
        return true;
    } else if (vertical_run_counter == VERTICAL_RETRY_STATE::DESCEND) {
        Vector3f current_pos;
        if (!AP::ahrs().get_relative_position_NED_origin(current_pos)) {
            return false;
        }
        if (fabsf(current_pos.z - retry_location.z) < 0.5) {
            // reached last loc
            vertical_run_counter = VERTICAL_RETRY_STATE::REACHED_BOTTOM;
            // we have compelted the entire cycle, yet target was not found
            gcs().send_text(MAV_SEVERITY_CRITICAL, "All done!");
            return false;
        }
        return true;
    }
    return true;
}