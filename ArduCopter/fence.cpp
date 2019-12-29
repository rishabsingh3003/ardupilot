#include "Copter.h"
#include <GCS_MAVLink/GCS.h>

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void Copter::fence_check()
{
    const uint8_t orig_breaches = fence.get_breaches();

    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = fence.check();

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached 
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed.
    if (!motors->armed()) {
        return;
    }

    // if there is a new breach take action
    if (new_breaches) {

        // if the user wants some kind of response and motors are armed
        uint8_t fence_act = fence.get_action();
        if (fence_act != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                arming.disarm();

            } else {

                // if more than 100m outside the fence just force a land
                if (fence.get_breach_distance(new_breaches) > AC_FENCE_GIVE_UP_DISTANCE) {
                    set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                } else {
                    switch (fence_act) {
                    case AC_FENCE_ACTION_RTL_AND_LAND:
                    default:
                        // switch to RTL, if that fails then Land
                        if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case AC_FENCE_ACTION_ALWAYS_LAND:
                        // if always land option mode is specified, land
                        set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        break;
                    case AC_FENCE_ACTION_SMART_RTL:
                        // Try SmartRTL, if that fails, RTL, if that fails Land
                        if (!set_mode(Mode::Number::SMART_RTL, ModeReason::FENCE_BREACHED)) {
                            if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                                set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                            }
                        }
                        break;
                    case AC_FENCE_ACTION_BRAKE:
                        // Try Brake, if that fails Land
                        if (!set_mode(Mode::Number::BRAKE, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    }
                }
            }
        }

        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(new_breaches));

    } else if (orig_breaches) {
        // record clearing of breach
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

void Copter::warn_approaching_fence()
{
    //return if disarmed
    if (!motors->armed()) {
            return;
        }
    
    //time threshold for alerting the user
    uint16_t alert_time = fence.get_alert_time();

    //check if unbreached
    if ((!fence.get_breaches()) && alert_time) {
        uint8_t enabled_fence = fence.get_enabled_fences();
        //variables to store time to breach particular fence
        float time_alt = 0.0f;
        float time_circle = 0.0f;
        float time_poly= 0.0f;
        //speed required to estimate time from distance 
        float ground_speed = ahrs_view->groundspeed();
        float vertical_speed = inertial_nav.get_velocity_z()*0.01f;

        //warn according to enabled fence
        switch (enabled_fence) {
            case 1:
                //only altitude fence is enabled
                time_alt= fence.get_distance_to_breach_fence_alt()/vertical_speed; 
                if (0<time_alt && time_alt<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Altitidue fence breach in %5.3f seconds", (double)time_alt);
                break;
            
            case 2:
                //only circle fence is enabled
                time_circle = fence.get_distance_to_breach_fence_circle()/ground_speed;
                if (0<time_circle && time_circle<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Circle fence breach in %5.3f seconds", (double)time_circle);
                break;

            case 3:
                //only altitude and circle fence is enabled
                time_alt= fence.get_distance_to_breach_fence_alt()/vertical_speed;
                if (0<time_alt && time_alt<alert_time)
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Altitidue fence breach in %5.3f seconds", (double)time_alt);

                time_circle = fence.get_distance_to_breach_fence_circle()/ground_speed;
                if (0<time_circle && time_circle<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Circle fence breach in %5.3f seconds", (double)time_circle);
                break;

            case 4:
                //only polygon fence is enabled
                time_poly =fence.get_distance_to_breach_fence_polygon()/ground_speed;
                if (0<time_poly && time_poly<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Polygon Fence breach in %5.3f seconds", (double)time_poly);
                break;

            case 5:
                //only altitude and polygon fence is enabled
                time_alt= fence.get_distance_to_breach_fence_alt()/vertical_speed;
                if (0<time_alt && time_alt<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Altitidue fence breach in %5.3f seconds", (double)time_alt);

                time_poly =fence.get_distance_to_breach_fence_polygon()/ground_speed;
                if (0<time_poly && time_poly<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Polygon Fence breach in %5.3f seconds", (double)time_poly);
                break;

            case 6:
                //only circle and polygon fence is enabled
                time_circle = fence.get_distance_to_breach_fence_circle()/ground_speed;
                if (0<time_circle && time_circle<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Circle fence breach in %5.3f seconds", (double)time_circle);
                
                time_poly =fence.get_distance_to_breach_fence_polygon()/ground_speed;
                if (0<time_poly && time_poly<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Polygon Fence breach in %5.3f seconds", (double)time_poly);
                break;

            case 7:
                //All fences are enabled
                time_circle = fence.get_distance_to_breach_fence_circle()/ground_speed;
                if (0<time_circle && time_circle<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Circle fence breach in %5.3f seconds", (double)time_circle);

                time_alt= fence.get_distance_to_breach_fence_alt()/vertical_speed;
                if (0<time_alt && time_alt<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Altitidue fence breach in %5.3f seconds", (double)time_alt);

                time_poly =fence.get_distance_to_breach_fence_polygon()/ground_speed;
                if (0<time_poly && time_poly<alert_time)  
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Polygon Fence breach in %5.3f seconds", (double)time_poly);
                break;
        }

    }
    
}

#endif