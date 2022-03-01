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

#include "AP_Proximity_Lua.h"

#if HAL_PROXIMITY_ENABLED
#if AP_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_LUA_TIMEOUT_MS 1500 // distance messages must arrive within this many milliseconds

// update the state of the sensor
void AP_Proximity_Lua::update(void)
{
    // check for timeout and set health status
    if ((_last_update_ms == 0 || (AP_HAL::millis() - _last_update_ms > PROXIMITY_LUA_TIMEOUT_MS)) &&
        (_last_upward_update_ms == 0 || (AP_HAL::millis() - _last_upward_update_ms > PROXIMITY_LUA_TIMEOUT_MS))) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get maximum distance (in meters) of sensor
float AP_Proximity_Lua::distance_max() const
{
    ap_var_type ptype;
    AP_Float *dist_max = (AP_Float *)AP_Param::find("PRX_MAX", &ptype);  // ! is there a better way to do this?
    if (dist_max == nullptr || ptype != AP_PARAM_FLOAT) {
        return 500;
    }
    return dist_max->cast_to_float();
}

// get minimum distance (in meters) of sensor
float AP_Proximity_Lua::distance_min() const
{
    ap_var_type ptype;
    AP_Float *dist_min = (AP_Float *)AP_Param::find("PRX_MIN", &ptype);  // ! is there a better way to do this?
    if (dist_min == nullptr || ptype != AP_PARAM_FLOAT) {
        return 0;
    }
    return dist_min->cast_to_float();
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_Lua::get_upward_distance(float &distance) const
{
    if ((_last_upward_update_ms != 0) && (AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_LUA_TIMEOUT_MS)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}

// handle script distance messages
bool AP_Proximity_Lua::handle_script_distance_msg(float dist_m, float yaw_deg, float pitch_deg)
{
    _last_update_ms = AP_HAL::millis();

    Vector3f current_pos;
    Matrix3f body_to_ned;
    const bool database_ready = database_prepare_for_push(current_pos, body_to_ned);

    if (dist_m < distance_min() || dist_m > distance_max() || is_zero(dist_m)) {
        // message isn't healthy
        return false;
    }

    if (ignore_reading(pitch_deg, yaw_deg, dist_m, false)) {
        // obstacle is probably near ground or out of range
        return false;
    }

    // allot to correct layer and sector based on calculated pitch and yaw
    const AP_Proximity_Boundary_3D::Face face = boundary.get_face(pitch_deg, yaw_deg);

    // add to temp boundary and update boundary
    temp_boundary.reset();
    temp_boundary.add_distance(face, pitch_deg, yaw_deg, dist_m);
    temp_boundary.update_3D_boundary(boundary);

    if (database_ready) {
        database_push(yaw_deg, pitch_deg, dist_m, _last_update_ms, current_pos, body_to_ned);
    }

    // store upward distance
    if (is_equal(yaw_deg, 0.f) && is_equal(pitch_deg, 90.f)) {
        _distance_upward = dist_m;
        _last_upward_update_ms = _last_update_ms;
    }

    return true;
}

// handle script vector messages
bool AP_Proximity_Lua::handle_script_3d_msg(Vector3f vec_to_obstacle)
{
    // convert to FRU  // ! necessary?  should we expect Lua to conform to MAVLink coordinate frame or simply expect FRU?
    const Vector3f obstacle(vec_to_obstacle.x, vec_to_obstacle.y, vec_to_obstacle.z * -1.0f);

    // extract yaw and pitch from Obstacle Vector
    const float yaw = wrap_360(degrees(atan2f(obstacle.y, obstacle.x)));
    const float pitch = wrap_180(degrees(M_PI_2 - atan2f(obstacle.xy().length(), obstacle.z)));

    // now simply handle as a distance msg
    return handle_script_distance_msg(obstacle.length(), yaw, pitch);
}

#endif // AP_SCRIPTING_ENABLED
#endif // HAL_PROXIMITY_ENABLED
