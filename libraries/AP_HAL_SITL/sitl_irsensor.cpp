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
#include "AC_WPNav.h"
#include <GCS_MAVLink/GCS.h>
//check AP_APHRS

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

void SITL_State::_update_irsensor()  
{
  //double vehicle_yaw = fdm.yawDeg; //defined in sitl_fdm, SITL.h
 const Vector3f &current_pos = _inav.get_position();
  float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
  Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
    // calculate terrain adjustments
    float terrain_offset = 0.0f;
    terrain_offset=_inav.get_altitude() - _rangefinder_alt_cm;

    // calculate 3d vector from segment's origin
    Vector3f curr_delta = (current_pos - Vector3f(0,0,terrain_offset)) - _origin;

    // calculate how far along the track we are
    track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

    // calculate the point closest to the vehicle on the segment from origin to destination
    Vector3f track_covered_pos = _pos_delta_unit * track_covered;

    // calculate the distance vector from the vehicle to the closest point on the segment from origin to destination
    track_error = curr_delta - track_covered_pos;
    float track_error_norm = norm(track_error.x,track_error.y)
    gcs().send_text(MAV_SEVERITY_INFO, "IR value %5.3f",(double)track_error_norm);
    return 1;
}