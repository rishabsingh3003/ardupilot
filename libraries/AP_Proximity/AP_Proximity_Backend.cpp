/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Avoidance/AP_OADatabase.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Backend::AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state) :
        frontend(_frontend),
        state(_state)
{
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity_Backend::get_closest_object(float& angle_deg, float &distance) const
{
    bool sector_found = false;
    uint8_t sector = 0;
    // check all sectors for shorter distance
    for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++) {
        if (boundary.check_distance_valid(i,PROXIMITY_MIDDLE_STACK)) {
            if (!sector_found || (boundary.get_distance(i, PROXIMITY_MIDDLE_STACK) < boundary.get_distance(sector, PROXIMITY_MIDDLE_STACK))) {
                sector = i;
                sector_found = true;
            }
        }
    }

    if (sector_found) {
        angle_deg = boundary.get_angle(sector, PROXIMITY_MIDDLE_STACK);
        distance = boundary.get_distance(sector, PROXIMITY_MIDDLE_STACK);
    }
    return sector_found;
}

// get number of objects, used for non-GPS avoidance
uint8_t AP_Proximity_Backend::get_object_count() const
{
    return PROXIMITY_NUM_SECTORS;
}

// get an object's angle and distance, used for non-GPS avoidance
// returns false if no angle or distance could be returned for some reason
bool AP_Proximity_Backend::get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const
{
    if (object_number < PROXIMITY_NUM_SECTORS && boundary.check_distance_valid(object_number, PROXIMITY_MIDDLE_STACK)) {
        angle_deg = boundary.get_angle(object_number, PROXIMITY_MIDDLE_STACK);
        distance = boundary.get_distance(object_number, PROXIMITY_MIDDLE_STACK);
        return true;
    }
    return false;
}

// get distances in PROXIMITY_MAX_DIRECTION directions. used for sending distances to ground station
bool AP_Proximity_Backend::get_horizontal_distances(AP_Proximity::Proximity_Distance_Array &prx_dist_array) const
{
    // cycle through all sectors filling in distances and orientations
    // see MAV_SENSOR_ORIENTATION for orientations (0 = forward, 1 = 45 degree clockwise from north, etc)
    bool valid_distances = false;
    for (uint8_t i=0; i<PROXIMITY_MAX_DIRECTION; i++) {
        prx_dist_array.orientation[i] = i;
        if (boundary.check_distance_valid(i, PROXIMITY_MIDDLE_STACK)) {
            valid_distances = true;
            prx_dist_array.distance[i] = boundary.get_distance(i, PROXIMITY_MIDDLE_STACK);
        } else {
            prx_dist_array.distance[i] = distance_max();
        }
    }

    return valid_distances;
}

// set status and update valid count
void AP_Proximity_Backend::set_status(AP_Proximity::Status status)
{
    state.status = status;
}

// correct an angle (in degrees) based on the orientation and yaw correction parameters
float AP_Proximity_Backend::correct_angle_for_orientation(float angle_degrees) const
{
    const float angle_sign = (frontend.get_orientation(state.instance) == 1) ? -1.0f : 1.0f;
    return wrap_360(angle_degrees * angle_sign + frontend.get_yaw_correction(state.instance));
}

// check if a reading should be ignored because it falls into an ignore area
bool AP_Proximity_Backend::ignore_reading(uint16_t angle_deg) const
{
    // check angle vs each ignore area
    for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
        if (frontend._ignore_width_deg[i] != 0) {
            if (abs(angle_deg - frontend._ignore_angle_deg[i]) <= (frontend._ignore_width_deg[i]/2)) {
                return true;
            }
        }
    }
    return false;
}

// returns true if database is ready to be pushed to and all cached data is ready
bool AP_Proximity_Backend::database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    if (!AP::ahrs().get_relative_position_NED_origin(current_pos)) {
        return false;
    }

    body_to_ned = AP::ahrs().get_rotation_body_to_ned();

    return true;
}

// update Object Avoidance database with Earth-frame point
void AP_Proximity_Backend::database_push(float angle, float distance)
{
    Vector3f current_pos;
    Matrix3f body_to_ned;

    if (database_prepare_for_push(current_pos, body_to_ned)) {
        database_push(angle, distance, AP_HAL::millis(), current_pos, body_to_ned);
    }
}

// update Object Avoidance database with Earth-frame point
void AP_Proximity_Backend::database_push(float angle, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }
    
    //Assume object is angle bearing and distance meters away from the vehicle 
    Vector2f object_2D = {0.0f,0.0f};
    object_2D.offset_bearing(wrap_180(angle), distance);	
    
    //rotate that vector to a 3D vector in NED frame
    const Vector3f object_3D = {object_2D.x,object_2D.y,0.0f};
    const Vector3f rotated_object_3D = body_to_ned * object_3D;
    
    //Calculate the position vector from origin
    Vector3f temp_pos = current_pos + rotated_object_3D;
    //Convert the vector to a NEU frame from NED
    temp_pos.z = temp_pos.z * -1.0f;
    
    oaDb->queue_push(temp_pos, timestamp_ms, distance);
}

void AP_Proximity_Backend::database_push_3D_obstacle(const Vector3f &obstacle, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned) 
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }

    const Vector3f rotated_object_3D = body_to_ned * obstacle;
    //Calculate the position vector from origin
    Vector3f temp_pos = current_pos + rotated_object_3D;
    //Convert the vector to a NEU frame from NED
    temp_pos.z = temp_pos.z * -1.0f;
    oaDb->queue_push(temp_pos, timestamp_ms, obstacle.length());
}