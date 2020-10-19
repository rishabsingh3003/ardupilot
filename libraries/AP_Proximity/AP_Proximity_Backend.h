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

#define PROXIMITY_NUM_SECTORS           8       // number of sectors
#define PROXIMITY_SECTOR_WIDTH_DEG      45.0f   // width of sectors in degrees
#define PROXIMITY_BOUNDARY_DIST_MIN 0.6f    // minimum distance for a boundary point.  This ensures the object avoidance code doesn't think we are outside the boundary.
#define PROXIMITY_BOUNDARY_DIST_DEFAULT 100 // if we have no data for a sector, boundary is placed 100m out
#define PROXIMITY_NUM_STACK           5
#define PROXIMITY_MIDDLE_STACK        2
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
    Vector3f (*get_bnd_points(uint16_t& num_points, uint32_t& stack_bit))[PROXIMITY_NUM_STACK];
    
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

    // find which sector a given angle falls into
    uint8_t convert_angle_to_sector(float angle_degrees) const;
    // find which stack a given pitch falls into
    uint8_t convert_pitch_to_stack(float pitch) const;
    
    // initialise the boundary and sector_edge_vector array used for object avoidance
    //   should be called if the sector_middle_deg or _sector_width_deg arrays are changed
    void init_boundary();

    // update boundary points used for object avoidance based on a single sector's distance changing
    //   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
    //   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
    void update_boundary_for_sector(const uint8_t sector, const bool push_to_OA_DB) {
        update_boundary_for_sector_and_stack(sector, PROXIMITY_MIDDLE_STACK, push_to_OA_DB);
    };
    void update_boundary_for_sector_and_stack(const uint8_t sector, const uint8_t stack, const bool push_to_OA_DB);

    // check if a reading should be ignored because it falls into an ignore area
    // angles should be in degrees and in the range of 0 to 360
    bool ignore_reading(uint16_t angle_deg) const;

    // database helpers.  all angles are in degrees
    bool database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned);
    void database_push(float angle, float distance);
    void database_push(float angle, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned);
    void database_push_3D_obstacle(const Vector3f &obstacle, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned);
    
    // set values
    void set_angle(float value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _angle[sector][stack] = value; }
    void set_pitch(float value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _pitch[sector][stack] = value; }
    void set_distance(float value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _distance[sector][stack] = value; }
    void mark_distance_valid(bool value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _distance_valid[sector][stack] = value; }
    
    // value getter
    float get_distance(uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { return _distance[sector][stack]; }
    
    AP_Proximity &frontend;
    AP_Proximity::Proximity_State &state;   // reference to this instances state

    // sectors
    const uint16_t _sector_middle_deg[PROXIMITY_NUM_SECTORS] = {0, 45, 90, 135, 180, 225, 270, 315};    // middle angle of each sector
    const int16_t _pitch_middle_deg[PROXIMITY_NUM_STACK] = {-60, -30, 0, 30, 60};

    Vector3f _sector_edge_vector[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];
    Vector3f _boundary_points[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];

private:

    // sensor data
    float _angle[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];            // angle to closest object within each sector
    float _pitch[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];
    float _distance[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];         // distance to closest object within each sector
    bool _distance_valid[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];    // true if a valid distance received for each sector
};
