#include "mode.h"
#include "Rover.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal ;

void ModeLineFollower::update()
{   float desired_steering, desired_throttle,IR_Reading;
    
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    // i = millis();
    g2.motors.set_throttle(desired_throttle);
    gcs().send_text(MAV_SEVERITY_INFO, "FLAG1");
    // gcs().send_text(MAV_SEVERITY_INFO, "IR value %5.3f",(double)i);
    

    AR_IR IR2;
    IR_Reading= IR2.readIR();
    gcs().send_text(MAV_SEVERITY_INFO, "IR value %5.3f",(double)IR_Reading);
    
    float steering_out;
    //turn clockwise    
    steering_out = attitude_control.get_steering_out_rate(4500.0f,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    //turn anti-clockwise
    steering_out = attitude_control.get_steering_out_rate(-4500.0f,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    set_steering(steering_out * 4500.0f);

}
