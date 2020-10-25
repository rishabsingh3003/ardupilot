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

#define PROXIMITY_NUM_SECTORS         8       // number of sectors
#define PROXIMITY_NUM_STACK           5       // num of stacks in a sector
#define PROXIMITY_MIDDLE_STACK        2       // middle stack 
#define PROXIMITY_PITCH_WIDTH_DEG     30      // width between each stack in degrees
#define PROXIMITY_SECTOR_WIDTH_DEG    45.0f   // width of sectors in degrees
#define PROXIMITY_BOUNDARY_DIST_MIN   0.6f    // minimum distance for a boundary point.  This ensures the object avoidance code doesn't think we are outside the boundary.
#define PROXIMITY_BOUNDARY_DIST_DEFAULT 100 // if we have no data for a sector, boundary is placed 100m out

class AP_Proximity_Boundary_3D 
{ 
public:
    // constructor. This incorporates initialisation as well.
	AP_Proximity_Boundary_3D();

    // update boundary points used for object avoidance based on a single sector and pitch distance changing
    //   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
    //   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
    void update_boundary(const uint8_t sector) { update_boundary(sector, PROXIMITY_MIDDLE_STACK);   }
    void update_boundary(const uint8_t sector, const uint8_t stack);

    // find which sector a given angle falls into
    uint8_t convert_angle_to_sector(float angle_degrees) const;
    // find which stack a given pitch falls into
    uint8_t convert_pitch_to_stack(float pitch) const;

    // sectors
    const uint16_t _sector_middle_deg[PROXIMITY_NUM_SECTORS] {0, 45, 90, 135, 180, 225, 270, 315};    // middle angle of each sector
    // stacks
    const int16_t _pitch_middle_deg[PROXIMITY_NUM_STACK] {-60, -30, 0, 30, 60};

    // set values
    void set_angle(float value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _angle[sector][stack] = value; }
    void set_pitch(float value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _pitch[sector][stack] = value; }
    void set_distance(float value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _distance[sector][stack] = value; }
    void mark_distance_valid(bool value, uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) { _distance_valid[sector][stack] = value; }

    // value getter
    float get_angle(uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) const { return _angle[sector][stack]; }
    float get_pitch(uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) const { return _pitch[sector][stack]; }
    float get_distance(uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) const { return _distance[sector][stack]; }
    bool check_distance_valid(uint8_t sector, uint8_t stack = PROXIMITY_MIDDLE_STACK) const { return _distance_valid[sector][stack]; }

    // return boundary points to be used by Simple Avoidance
    Vector3f (*get_boundary_points(uint16_t& num_points, uint8_t& num_layers, uint32_t& stack_bit))[PROXIMITY_NUM_STACK];

private:
    // initialise the boundary and sector_edge_vector array used for object avoidance
    //   should be called if the sector_middle_deg or _sector_width_deg arrays are changed
    void init_boundary();

    Vector3f _sector_edge_vector[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];
    Vector3f _boundary_points[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];
    // sensor data
    float _angle[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];            // angle to closest object within each sector
    float _pitch[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];
    float _distance[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];         // distance to closest object within each sector
    bool _distance_valid[PROXIMITY_NUM_SECTORS][PROXIMITY_NUM_STACK];    // true if a valid distance received for each sector
};
