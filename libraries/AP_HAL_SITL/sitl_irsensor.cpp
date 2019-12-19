/*
  SITL handling

  This simulates a Analog IR Sensor

 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "SITL_State.h"
#include <SITL/SITL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

void SITL_State::_update_irsensor(){

    const Location &home_loc = AP::ahrs().get_home();// home location
    Location projected_home_line; //virtual projected line from home
    Location sensor_1; //location of first sensor
    Location sensor_2;// Location of second sensor
    
    float current_heading =  AP::ahrs().yaw_sensor * 0.01f;
    float max_sensor_range = 300; // maximum range of the sensor (cm)

    projected_home_line.lat = home_loc.lat;
    projected_home_line.lng = home_loc.lng;
    projected_home_line.offset_bearing(0,1000);//1km line projected from home

    sensor_1.lat = _sitl->state.latitude*1.0e7;
    sensor_1.lng = _sitl->state.longitude*1.0e7;
    sensor_1.offset_bearing(wrap_360(current_heading+90),0.75); //First sensor offset is set at 75cm 

    sensor_2.lat = _sitl->state.latitude*1.0e7;
    sensor_2.lng = _sitl->state.longitude*1.0e7;
    sensor_2.offset_bearing(wrap_360(current_heading-90),0.75); //Second sensor offset is set at -75cm
    
    Vector2f start_NE, end_NE,sensor_1_offset_NE,sensor_2_offset_NE; //get vectors from origin to the points
    bool home_loc_vector = home_loc.get_vector_xy_from_origin_NE(start_NE);
    bool projected_line_vector = projected_home_line.get_vector_xy_from_origin_NE(end_NE);    
    bool sensor_1_vector = sensor_1.get_vector_xy_from_origin_NE(sensor_1_offset_NE);
    bool sensor_2_vector = sensor_2.get_vector_xy_from_origin_NE(sensor_2_offset_NE); 

    float crosstrack_error_1 = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, sensor_1_offset_NE); // distance between individual sensors and the virtual line
    float crosstrack_error_2 = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, sensor_2_offset_NE);

    if (crosstrack_error_1>= max_sensor_range){ //force sensor to stay in range limit
        crosstrack_error_1 = max_sensor_range;
    }if(crosstrack_error_2>= max_sensor_range){
        crosstrack_error_2 = max_sensor_range;
    }
    float voltage_sensor_1 = linear_interpolate(0,5,crosstrack_error_1,0,max_sensor_range); //convert the distance to voltage
    float voltage_sensor_2 = linear_interpolate(0,5,crosstrack_error_2,0,max_sensor_range);

     
    if(home_loc_vector&projected_line_vector&sensor_1_vector&sensor_2_vector){   //check for valid origin
        ir1_pin_value = voltage_sensor_1*1023/5;
        ir2_pin_value = voltage_sensor_2*1023/5;
    }else{
        ir1_pin_value = 0; //set voltage 0 if valid origin is not available
        ir2_pin_value = 0;
    }
}
#endif