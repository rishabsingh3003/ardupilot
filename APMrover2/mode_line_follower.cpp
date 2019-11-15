#include "mode.h"
#include "Rover.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal ;

void ModeLineFollower::update()
{   float desired_steering, desired_throttle,IR_Reading_Right;
    
    get_pilot_input(desired_steering, desired_throttle);
    // i = millis();
    g2.motors.set_throttle(desired_throttle);
   // gcs().send_text(MAV_SEVERITY_INFO, "FLAG1");
    
    
    //set analog pins 
    AR_IR* IR2 = new AR_IR;
    IR_Reading_Right= IR2->readIR(16);// pin 14 and 13 correspond to ADC pins on my pixhawk, will be taken from parameters later
    //IR_Reading_Left = IR2->readIR(17);
    delete IR2;
    gcs().send_text(MAV_SEVERITY_INFO, "IR value %5.3f",(double)desired_throttle);
    
    float steering_out;
    //turn clockwise    
    // if (IR_Reading_Right>= 400)
    // {steering_out = attitude_control.get_steering_out_rate(4500.0f,
    //                                                           g2.motors.limit.steer_left,
    //                                                           g2.motors.limit.steer_right,
    //                                                           rover.G_Dt);
    // }

    // if (IR_Reading_Left>= 400)
    // //turn anti-clockwise
    // {steering_out = attitude_control.get_steering_out_rate(-4500.0f,
    //                                                           g2.motors.limit.steer_left,
    //                                                           g2.motors.limit.steer_right,
    //                                                           rover.G_Dt);}

    // else
    const float target_turn_rate = (-IR_Reading_Right / 4500.0f) * radians(g2.acro_turn_rate);
    {steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);}


        
    set_steering(steering_out * 4500.0f);

}
