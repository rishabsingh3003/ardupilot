#include "mode.h"
#include "Rover.h"
#include <GCS_MAVLink/GCS.h>

void ModeLineFollower::update()
{   float desired_steering, desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    gcs().send_text(MAV_SEVERITY_INFO, "Throttle set to %5.3f")(float)desired_throttle;
    g2.motors.set_throttle(desired_throttle);
    ///turn clockwise
     steering_out = attitude_control.get_steering_out_rate(4500.0f,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    set_steering(steering_out * 4500.0f);

}