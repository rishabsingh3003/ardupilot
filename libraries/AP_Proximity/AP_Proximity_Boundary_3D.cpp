#include "AP_Proximity_Backend.h"
#include "AP_Proximity_Boundary_3D.h"

/*
  Constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Boundary_3D::AP_Proximity_Boundary_3D() 
{
    // initialise sector edge vector used for building the boundary fence
    init_boundary();
}

// initialise the boundary and sector_edge_vector array used for object avoidance
//   should be called if the sector_middle_deg or _sector_width_deg arrays are changed
void AP_Proximity_Boundary_3D::init_boundary()
{
    for (uint8_t stack = 0; stack < PROXIMITY_NUM_STACK; stack ++) {
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            float angle_rad = ((float)_sector_middle_deg[sector]+(PROXIMITY_SECTOR_WIDTH_DEG/2.0f));
            float pitch = ((float)_pitch_middle_deg[stack]);
            _sector_edge_vector[sector][stack].offset_bearing(angle_rad, pitch, 100.0f);
            _boundary_points[sector][stack] = _sector_edge_vector[sector][stack] * PROXIMITY_BOUNDARY_DIST_DEFAULT;
        }
    }
}

// update boundary points used for object avoidance based on a single sector and pitch distance changing
//   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
//   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
void AP_Proximity_Boundary_3D::update_boundary(const uint8_t sector, const uint8_t stack) {
    
    // sanity check
    if (sector >= PROXIMITY_NUM_SECTORS) {
        return;
    }

    // find adjacent sector (clockwise)
    uint8_t next_sector = sector + 1;
    if (next_sector >= PROXIMITY_NUM_SECTORS) {
        next_sector = 0;
    }

    // boundary point lies on the line between the two sectors at the shorter distance found in the two sectors
    float shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[sector][stack] && _distance_valid[next_sector][stack]) {
        shortest_distance = MIN(_distance[sector][stack], _distance[next_sector][stack]);
    } else if (_distance_valid[sector][stack]) {
        shortest_distance = _distance[sector][stack];
    } else if (_distance_valid[next_sector][stack]) {
        shortest_distance = _distance[next_sector][stack];
    }
    if (shortest_distance < PROXIMITY_BOUNDARY_DIST_MIN) {
        shortest_distance = PROXIMITY_BOUNDARY_DIST_MIN;
    }
    _boundary_points[sector][stack] = _sector_edge_vector[sector][stack] * shortest_distance;

    // if the next sector (clockwise) has an invalid distance, set boundary to create a cup like boundary
    if (!_distance_valid[next_sector][stack]) {
        _boundary_points[next_sector][stack] = _sector_edge_vector[next_sector][stack] * shortest_distance;
    }

    // repeat for edge between sector and previous sector
    uint8_t prev_sector = (sector == 0) ? PROXIMITY_NUM_SECTORS-1 : sector-1;
    shortest_distance = PROXIMITY_BOUNDARY_DIST_DEFAULT;
    if (_distance_valid[prev_sector][stack] && _distance_valid[sector][stack]) {
        shortest_distance = MIN(_distance[prev_sector][stack], _distance[sector][stack]);
    } else if (_distance_valid[prev_sector][stack]) {
        shortest_distance = _distance[prev_sector][stack];
    } else if (_distance_valid[sector][stack]) {
        shortest_distance = _distance[sector][stack];
    }
    _boundary_points[prev_sector][stack] = _sector_edge_vector[prev_sector][stack] * shortest_distance;

    // if the sector counter-clockwise from the previous sector has an invalid distance, set boundary to create a cup like boundary
    uint8_t prev_sector_ccw = (prev_sector == 0) ? PROXIMITY_NUM_SECTORS - 1 : prev_sector - 1;
    if (!_distance_valid[prev_sector_ccw][stack]) {
        _boundary_points[prev_sector_ccw][stack] = _sector_edge_vector[prev_sector_ccw][stack] * shortest_distance;
    }
}

// This method draws a line between this sector, and sector + 1, given a stack. Then returns the closest point on this line from vehicle, in body-frame. 
// Also checks for a valid distance in the stack layer, returns False if not found 
bool AP_Proximity_Boundary_3D::find_closest_point_to_boundary(const uint8_t sector, const uint8_t stack, Vector3f& vec_to_boundary) const
{   
    bool valid_layer = false;
    // check if this layer has atleast one valid sector
    for (uint8_t i=0; i<PROXIMITY_NUM_SECTORS; i++ ) {
        if (_distance_valid[i][stack]) {
            valid_layer = true;
            break;
        }
    }
    if(!valid_layer) {
        // no valid sector found, return
        return false;
    }

    const uint8_t sector_end = sector;
    uint8_t sector_start = sector + 1;
    if (sector_start >= PROXIMITY_NUM_SECTORS) {
        sector_start = 0;
    }
    const Vector3f start = _boundary_points[sector_start][stack];
    const Vector3f end = _boundary_points[sector_end][stack];
    vec_to_boundary = Vector3f::closest_point_between_line_and_point(start, end, Vector3f{0.0f, 0.0f, 0.0f});
    return true;
}

// This method draws a line between this sector, and sector + 1, given a stack. 
// Then returns the closest point on this line from the segment that was passed, in body-frame. 
float AP_Proximity_Boundary_3D::find_closest_point_to_boundary_from_segment(const uint8_t sector, const uint8_t stack, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const
{
    const uint8_t sector_end = sector;
    uint8_t sector_start = sector + 1;
    if (sector_start >= PROXIMITY_NUM_SECTORS) {
        sector_start = 0;
    }
    const Vector3f start = _boundary_points[sector_start][stack];
    const Vector3f end = _boundary_points[sector_end][stack];
    return Vector3f::segment_to_segment_dist(seg_start, seg_end, start, end, closest_point);
}

// find which sector a given angle falls into
uint8_t AP_Proximity_Boundary_3D::convert_angle_to_sector(float angle_degrees) const
{
    return wrap_360(angle_degrees + (PROXIMITY_SECTOR_WIDTH_DEG * 0.5f)) / 45.0f;
}

// find which stack a given pitch falls into
uint8_t AP_Proximity_Boundary_3D::convert_pitch_to_stack(float pitch_degrees) const
{   
    // TODO: handle pitch less than -75 or greater than 75 properly 
    pitch_degrees = constrain_float(pitch_degrees, -75.0f, 74.9f);
    return (pitch_degrees + 75.0f)/PROXIMITY_PITCH_WIDTH_DEG;
}
