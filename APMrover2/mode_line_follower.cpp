#include "mode.h"
#include "Rover.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal ;

void ModeLineFollower::update()
{   float desired_steering, desired_throttle,IR_Reading_Left, IR_Reading_Right;
    
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    // i = millis();
    g2.motors.set_throttle(desired_throttle);
    gcs().send_text(MAV_SEVERITY_INFO, "FLAG1");
    
    
    //set analog pins 
    AR_IR* IR2 = new AR_IR;
    IR_Reading_Right= IR2->readIR(14);// pin 14 and 13 correspond to ADC pins on my pixhawk, will be taken from parameters later
    IR_Reading_Left = IR2->readIR(13);
    delete IR2;
    gcs().send_text(MAV_SEVERITY_INFO, "IR value %5.3f",(double)IR_Reading_Right);
    
    float steering_out;
    //turn clockwise    
    if (IR_Reading_Right>= 400)
    {steering_out = attitude_control.get_steering_out_rate(4500.0f,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    if (IR_Reading_Left>= 400)
    //turn anti-clockwise
    {steering_out = attitude_control.get_steering_out_rate(-4500.0f,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);}

    else
    {steering_out = attitude_control.get_steering_out_rate(0.0f,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);}


        
    set_steering(steering_out * 4500.0f);

}
