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
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

void SITL_State::_update_irsensor() 
{   
    const Location &home_loc = AP::ahrs().get_home();
    Location projected_home_line;
    Location current_loc;   

    projected_home_line.lat = home_loc.lat;
    projected_home_line.lng = home_loc.lng;
    current_loc.lat = _sitl->state.latitude*1.0e7;
    current_loc.lng = _sitl->state.longitude*1.0e7;
    projected_home_line.offset_bearing(0,1000);

    Vector2f start_NE, end_NE, current_pos_NE;
    bool home_loc_vector = home_loc.get_vector_xy_from_origin_NE(start_NE);
    bool projected_line_vector = projected_home_line.get_vector_xy_from_origin_NE(end_NE);
    bool current_pos_vector = current_loc.get_vector_xy_from_origin_NE(current_pos_NE);
    float check_side= ((end_NE.x - start_NE.x)*(current_pos_NE.y- start_NE.y)-(end_NE.y - start_NE.y)*(current_pos_NE.x- start_NE.x))/1.0e7;

    float crosstrack_error = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, current_pos_NE);
    int check_side_signum = (check_side > 0) ? check_side = 1 : ((check_side < 0) ? check_side = -1 : check_side = 0);
    int max_sensor_range = 500;
    int input_start = -max_sensor_range;
    int input_end = max_sensor_range;
    int input_range = input_end - input_start;
    
    if(home_loc_vector&projected_line_vector&current_pos_vector)
    {
        if ( crosstrack_error >= max_sensor_range)
        {
            (check_side_sigum == 1) ? ir_pin_value = 5 : ir_pin_value = 0;
        }
        else
        {
         ir_pin_value = (crosstrack_error *check_side_signum- input_start)*5/input_range ;}
        }
    else
    {
        ir_pin_value = 69;
    }
  
    
}
#endif