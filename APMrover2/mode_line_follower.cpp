#include "mode.h"
#include "Rover.h"
#include <GCS_MAVLink/GCS.h>

void ModeLineFollower::update()
{   float desired_steering, desired_throttle,steering_out,IR_Reading;
    
    get_pilot_input(desired_steering, desired_throttle);
   
    g2.motors.set_throttle(desired_throttle);
   
    //set analog pins 
    AR_IR* IR2 = new AR_IR;
    IR_Reading= IR2->readIR(16);// pin 16 defined on SITL Analog input 
    delete IR2;
    gcs().send_text(MAV_SEVERITY_INFO, "IR value %5.3f",(double)IR_Reading);
      
    const float target_turn_rate = (-IR_Reading) * radians(g2.acro_turn_rate);
    {
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

 
    set_steering(steering_out * 4500.0f);
}
