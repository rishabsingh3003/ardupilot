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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity.h"
#include <AP_Common/Location.h>
#include "AP_Proximity_Boundary_3D.h"

class AP_Proximity_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // we declare a virtual destructor so that Proximity drivers can
    // override with a custom destructor if need be
    virtual ~AP_Proximity_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    // get maximum and minimum distances (in meters) of sensor
    virtual float distance_max() const = 0;
    virtual float distance_min() const = 0;

    // get distance upwards in meters. returns true on success
    virtual bool get_upward_distance(float &distance) const { return false; }

    // handle mavlink DISTANCE_SENSOR messages
    virtual void handle_msg(const mavlink_message_t &msg) {}

    // get boundary points around vehicle for use by avoidance
    //   returns nullptr and sets num_points to zero if no boundary can be returned
    Vector3f (*get_boundary_points(uint16_t& num_points, uint8_t& num_layers, uint32_t& stack_bit))[PROXIMITY_NUM_STACK];
    
    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    bool get_closest_object(float& angle_deg, float &distance) const;

    // get number of objects, angle and distance - used for non-GPS avoidance
    uint8_t get_object_count() const;
    bool get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const;

    // get distances in 8 directions. used for sending distances to ground station
    bool get_horizontal_distances(AP_Proximity::Proximity_Distance_Array &prx_dist_array) const;

protected:

    // set status and update valid_count
    void set_status(AP_Proximity::Status status);

    // correct an angle (in degrees) based on the orientation and yaw correction parameters
    float correct_angle_for_orientation(float angle_degrees) const;
    
    // check if a reading should be ignored because it falls into an ignore area
    // angles should be in degrees and in the range of 0 to 360
    bool ignore_reading(uint16_t angle_deg) const;
    
    // database helpers.  all angles are in degrees
    static bool database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned);
    static void database_push(float angle, float distance);
    static void database_push(float angle, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned);
    static void database_push_3D_obstacle(const Vector3f &obstacle, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned);
        
    AP_Proximity &frontend;
    AP_Proximity::Proximity_State &state;   // reference to this instances state

    // Methods to manipulate 3D boundary in this class
    AP_Proximity_Boundary_3D boundary;
};
