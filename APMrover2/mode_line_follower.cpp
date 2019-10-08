#include "mode.h"
#include "Rover.h"
#include <AR_IR/AR_IR_Sensor.h>
#include <GCS_MAVLink/GCS.h>

void ModeLineFollower::update()
{   float desired_steering, desired_throttle, IR_Reading;
    
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    
    g2.motors.set_throttle(desired_throttle);
    AR_IR IR1;
    IR_Reading=IR1.readIR();
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
