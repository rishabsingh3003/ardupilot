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
//check AP_APHRS

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

void SITL_State::_update_irsensor()//const struct Location &prev_WP, const struct Location &next_WP)  
{
//   //double vehicle_yaw = fdm.yawDeg; //defined in sitl_fdm, SITL.h
//  const AP_InertialNav&   _inav;
//  const Vector3f &current_pos = _inav.get_position();
//   float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
//   Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
//     // calculate terrain adjustments
//     float terrain_offset = 0.0f;
//     terrain_offset=_inav.get_altitude() - _rangefinder_alt_cm;

//     // calculate 3d vector from segment's origin
//     Vector3f curr_delta = (current_pos - Vector3f(0,0,terrain_offset)) - _origin;

//     // calculate how far along the track we are
//     track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

//     // calculate the point closest to the vehicle on the segment from origin to destination
//     Vector3f track_covered_pos = _pos_delta_unit * track_covered;

//     // calculate the distance vector from the vehicle to the closest point on the segment from origin to destination
//     track_error = curr_delta - track_covered_pos;
//     float track_error_norm = norm(track_error.x,track_error.y)
//////////////////////////////////////////////////////////////////////
    // struct Location _current_loc;
    // gcs().send_text(MAV_SEVERITY_INFO, "FLAGABC");
    // Vector2f WPB_rel_WPA = prev_WP.get_distance_NE(next_WP);
    // const Vector2f WP_A_pos =prev_WP.get_distance_NE(_current_loc);
    // float crosstrack_error = WP_A_pos % WPB_rel_WPA
   
    Location test;
    Location home_line;
    Location current_loc;
    
    const Location &home_loc = AP::ahrs().get_home();
    //Location home_loc;
    Vector2f start_NE, end_NE, current_pos_NE;

    
    
    
    // home_loc.lat = _sitl->state.latitude*1.0e7;
    // home_loc.lng = _sitl->state.longitude*1.0e7;
    home_line.lat = _sitl->state.latitude*1.0e7;
    home_line.lng = _sitl->state.longitude*1.0e7;
    current_loc.lat = _sitl->state.latitude*1.0e7;
    current_loc.lng = _sitl->state.longitude*1.0e7;
    
    
    //float intial_bearing = _sitl->state.heading * 0.01f;
    float intial_bearing = ToDeg(atan2f(_sitl->state.speedE, _sitl->state.speedN)); //-180 to 180 degre3es
    // test.lat =  _sitl->state.latitude+20;
    // test.lng =  _sitl->state.longitude+20;
   // const Vector3f IRPosSensorBF = _sitl->rngfnd_pos_offset;
    //float vecWP = home_loc.get_distance(test);
    home_line.offset_bearing(intial_bearing,1000);
    bool temp = home_loc.get_vector_xy_from_origin_NE(start_NE);
    bool temp1 = home_line.get_vector_xy_from_origin_NE(end_NE);
    bool temp2 = current_loc.get_vector_xy_from_origin_NE(current_pos_NE);


/*
    // adjust altitude for position of the sensor on the vehicle if position offset is non-zero
    
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        _sitl->state.quaternion.rotation_matrix(rotmat);
        // rotate the offset into earth frame
        const Vector3f IRPosSensorEF = rotmat * relPosSensorBF;
        // correct the altitude at the sensor
        altitude -= IRPosSensorEF.x;
    
    
    //float vecWP = sqrtf(home_loc.get_distance_NE(test).length_squared());
  */  
    float crosstrack_error_IR = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, current_pos_NE);

    //int32_t local_home_lat = _sitl->state.home.lng; 
    //  gcs().send_text(MAV_SEVERITY_INFO, "loc lang test value %5.3f",(double)home_line.lng);
    //  gcs().send_text(MAV_SEVERITY_INFO, "loc lang value %5.3f",(double)home_loc.lng);
    //   gcs().send_text(MAV_SEVERITY_INFO, "loc lat test value %5.3f",(double)home_line.lat);
    //  gcs().send_text(MAV_SEVERITY_INFO, "loc lat value %5.3f",(double)home_loc.lat);
    //gcs().send_text(MAV_SEVERITY_INFO, "bearing value %5.3f",(double)intial_bearing);
     //gcs().send_text(MAV_SEVERITY_INFO, "distance value %5.3f",(double)vecWP);
    // gcs().send_text(MAV_SEVERITY_INFO, "long lat value %5.3f",(double)local_heading);
    //gcs().send_text(MAV_SEVERITY_INFO, "home value %5.3f",(double)temp);
    
    if(temp&temp1&temp2){
     ir_pin_value= crosstrack_error_IR;
    }
    else
    {ir_pin_value = 69;}//crosstrack_error();
    //gcs().send_text(MAV_SEVERITY_INFO, "IR value %5.3f",(double)ir_pin_value);
    
}
#endif