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
    Location current_loc_ir1;
    Location current_loc_ir2;
    
    const Location &home_loc = AP::ahrs().get_home();
    //Location home_loc;
    //bool temp4 = AP::ahrs().get_origin(home_loc);
    //Location home_loc;
    Vector2f start_NE, end_NE, current_pos_NE_ir1, current_pos_NE_ir2;
    
    // home_loc.lat = _sitl->state.latitude*1.0e7;
    // home_loc.lng = _sitl->state.longitude*1.0e7;
    home_line.lat = home_loc.lat;
    home_line.lng = home_loc.lng;
    current_loc_ir1.lat = _sitl->state.latitude*1.0e7;
    current_loc_ir1.lng = _sitl->state.longitude*1.0e7;
    current_loc_ir2.lat = _sitl->state.latitude*1.0e7;
    current_loc_ir2.lng = _sitl->state.longitude*1.0e7;

     Vector3f posRelOffsetBF_ir1(0.00f,0.00f,0.00); //_sitl->ir1_pos_offset;
 //posRelOffsetBF_ir1.y = 0.093;
 //posRelOffsetBF.y = 0.03;
    if (!posRelOffsetBF_ir1.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat_ir1;
        _sitl->state.quaternion.rotation_matrix(rotmat_ir1);

        // rotate the antenna offset into the earth frame
        Vector3f posRelOffsetEF_ir1 = rotmat_ir1 * posRelOffsetBF_ir1;

        // Add the offset to the latitude, longitude and height using a spherical earth approximation
        double const earth_rad_inv = 1.569612305760477e-7; // use Authalic/Volumetric radius
        double lng_scale_factor = earth_rad_inv / cos(radians(current_loc_ir1.lat));
        current_loc_ir1.lat += degrees(posRelOffsetEF_ir1.x * earth_rad_inv);
        current_loc_ir1.lng += degrees(posRelOffsetEF_ir1.y * lng_scale_factor);
    }
    

     Vector3f posRelOffsetBF_ir2(0.00f,0.0f,0.0f);
 //posRelOffsetBF.y = 0.03;
    if (!posRelOffsetBF_ir2.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat_ir2;
        _sitl->state.quaternion.rotation_matrix(rotmat_ir2);

        // rotate the antenna offset into the earth frame
        Vector3f posRelOffsetEF_ir2 = rotmat_ir2 * posRelOffsetBF_ir2;

        // Add the offset to the latitude, longitude and height using a spherical earth approximation
        double const earth_rad_inv = 1.569612305760477e-7; // use Authalic/Volumetric radius
        double lng_scale_factor = earth_rad_inv / cos(radians(current_loc_ir2.lat));
        current_loc_ir2.lat += degrees(posRelOffsetEF_ir2.x * earth_rad_inv);
        current_loc_ir2.lng += degrees(posRelOffsetEF_ir2.y * lng_scale_factor);
      
    }
    
   // float intial_bearing = _sitl->state.heading;
    //float intial_bearing = ToDeg(atan2f(_sitl->state.speedE, _sitl->state.speedN)); //-180 to 180 degre3es
    // test.lat =  _sitl->state.latitude+20;
    // test.lng =  _sitl->state.longitude+20;
   // const Vector3f IRPosSensorBF = _sitl->rngfnd_pos_offset;
    //float vecWP = home_loc.get_distance(test);
    home_line.offset_bearing(0,1000);
    // float crosstrack_error_IR1 = home_loc.lng;
    // float crosstrack_error_IR2 =home_loc.lat;
    bool temp = home_loc.get_vector_xy_from_origin_NE(start_NE);
    bool temp1 = home_line.get_vector_xy_from_origin_NE(end_NE);
    bool temp2 = current_loc_ir1.get_vector_xy_from_origin_NE(current_pos_NE_ir1);
    bool temp3 = current_loc_ir2.get_vector_xy_from_origin_NE(current_pos_NE_ir2);
    float check_side= ((end_NE.x - start_NE.x)*(current_pos_NE_ir1.y- start_NE.y)-(end_NE.y - start_NE.y)*(current_pos_NE_ir1.x- start_NE.x))/1.0e7;


    float crosstrack_error_IR1 = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, current_pos_NE_ir1);
    int check_side_signum = (check_side > 0) ? check_side = 1 : ((check_side < 0) ? check_side = -1 : check_side = 0);//Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, current_pos_NE_ir2);
    
    //int32_t local_home_lat = _sitl->state.home.lng; 
    //  gcs().send_text(MAV_SEVERITY_INFO, "loc lang test value %5.3f",(double)home_line.lng);
    //  gcs().send_text(MAV_SEVERITY_INFO, "loc lang value %5.3f",(double)home_loc.lng);
    //   gcs().send_text(MAV_SEVERITY_INFO, "loc lat test value %5.3f",(double)home_line.lat);
    //  gcs().send_text(MAV_SEVERITY_INFO, "loc lat value %5.3f",(double)home_loc.lat);
    //cs().send_text(MAV_SEVERITY_INFO, "bearing value %5.3f",(double)intial_bearing);
     //gcs().send_text(MAV_SEVERITY_INFO, "distance value %5.3f",(double)vecWP);
    // gcs().send_text(MAV_SEVERITY_INFO, "long lat value %5.3f",(double)local_heading);
    //gcs().send_text(MAV_SEVERITY_INFO, "home value %5.3f",(double)temp);
    
    if(temp&temp1&temp2&temp3){
        if ( crosstrack_error_IR1 >= 500)
        {ir1_pin_value = 500 *check_side_signum;}
        else
     {ir1_pin_value= crosstrack_error_IR1 * check_side_signum;}
     ir2_pin_value =check_side_signum;
    //  gcs().send_text(MAV_SEVERITY_INFO, "IR1 value %5.3f",(double)ir1_pin_value);
    //  gcs().send_text(MAV_SEVERITY_INFO, "IR2 value %5.3f",(double)ir2_pin_value);
    }
    else
    {ir1_pin_value = 69;
    ir2_pin_value= -69;}//crosstrack_error();
  
    
}
#endif