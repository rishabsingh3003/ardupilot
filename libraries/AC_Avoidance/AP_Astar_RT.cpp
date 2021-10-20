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

#include "AP_Astar_RT.h"
#include <AC_Avoidance/AP_OADatabase.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>


// parameter defaults
const float MARGIN_CM = 300;
#define OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX   UINT16_MAX
#define OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK  50 

const AP_Param::GroupInfo AP_OART_AStar::var_info[] = {

    // // @Param: LOOKAHEAD
    // // @DisplayName: Object Avoidance look ahead distance maximum
    // // @Description: Object Avoidance will look this many meters ahead of vehicle
    // // @Units: m
    // // @Range: 1 100
    // // @Increment: 1
    // // @User: Standard
    // AP_GROUPINFO("LOOKAHEAD", 1, AP_OABendyRuler, _lookahead, OA_BENDYRULER_LOOKAHEAD_DEFAULT),

    AP_GROUPEND
};

AP_OART_AStar::AP_OART_AStar(): 
                 oa_db_nodes(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _short_path_data(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK),
        _path(OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK)
{ 
    // AP_Param::setup_object_defaults(this, var_info); 

}

// run background task to find best path and update avoidance_results
// returns true and updates origin_new and destination_new if a best path has been found
// bendy_type is set to the type of BendyRuler used
bool AP_OART_AStar::update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, bool proximity_only)
{   
    const uint32_t start_ms = AP_HAL::millis();

    // bendy ruler always sets origin to current_loc
    origin_new = current_loc;

    Vector3f current_pos,goal_pos;
    if (!current_loc.get_vector_from_origin_NEU(current_pos) || !destination.get_vector_from_origin_NEU(goal_pos)) {
        return false;
    }
    _path_destination = goal_pos;
    _path_source = current_pos;

    
    if (!create_oa_database_nodes()) {
        return false;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Create nodes %5.3f", (double)(AP_HAL::millis()-start_ms)*0.001f);

    if (!create_oa_db_visgraph()) {
        return false;
    }
    
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Create OA_DB vis %5.3f", (double)(AP_HAL::millis()-start_ms)*0.001f);
    send_debug_info();
    
    if (!create_goal_and_destination_visgraph(current_pos,goal_pos)) {
        return false;
    }
    
    gcs().send_text(MAV_SEVERITY_CRITICAL, "create goal and dest vis graph %5.3f", (double)(AP_HAL::millis()-start_ms)*0.001f);

    if (!find_shortest_path()) {
        return false;
    }

    Vector3f wp_loc;
    // uint16_t path_point = 0;
    // while (get_shortest_path_point(path_point, wp_loc)) {
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)path_point);
    //     path_point ++;
    // }
    
    // / path has been created, return latest point
    gcs().send_text(MAV_SEVERITY_CRITICAL, "time %5.3f", (double)(AP_HAL::millis()-start_ms)*0.001f);
    
    if (_path_numpoints <= 2) {
        // direct path exists
         gcs().send_text(MAV_SEVERITY_CRITICAL, "not needed");
        return false;
    }

    Vector3f dest_pos;
    uint16_t _path_idx_returned = 1;
    if (get_shortest_path_point(_path_idx_returned, dest_pos)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "active!");

        // for the first point return origin as current_loc
        Vector3f origin_pos;
        if ((_path_idx_returned > 0) && get_shortest_path_point(_path_idx_returned-1, origin_pos)) {
            // convert offset from ekf origin to Location
            Location temp_loc(origin_pos, Location::AltFrame::ABOVE_ORIGIN);
            origin_new = temp_loc;
        } else {
            // for first point use current loc as origin
            origin_new = current_loc;
        }

        // convert offset from ekf origin to Location
        Location temp_loc(Vector3f(dest_pos.x, dest_pos.y, 0.0f), Location::AltFrame::ABOVE_ORIGIN);
        destination_new = destination;
        destination_new.lat = temp_loc.lat;
        destination_new.lng = temp_loc.lng;

        // check if we should advance to next point for next iteration
        // const bool near_oa_wp = current_loc.get_distance(destination_new) <= 2.0f;
        // const bool past_oa_wp = current_loc.past_interval_finish_line(origin_new, destination_new);
        // if (near_oa_wp || past_oa_wp) {
        //     _path_idx_returned++;
        // }
        // log success
        // Write_OADijkstra(DIJKSTRA_STATE_SUCCESS, 0, _path_idx_returned, _path_numpoints, destination, destination_new);
        return true;
    }


    return false;
}


bool AP_OART_AStar::create_oa_database_nodes()
{
// exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    // clear all points
    _total_oa_db_numpoints = 0;

    // unit length offsets for polygon points around circles
    const Vector3f unit_offsets[] = {
            {cosf(radians(30)), cosf(radians(30-90)), 0.0f},  // north-east
            {cosf(radians(150)), cosf(radians(150-90)), 0.0f},// south-east
            {cosf(radians(210)), cosf(radians(210-90)), 0.0f},// south-west
            {cosf(radians(330)), cosf(radians(330-90)), 0.0f},// north-west
    };
    const uint16_t num_points_per_circle = ARRAY_SIZE(unit_offsets);

    // expand polygon point array if required
    const uint16_t num_obstacles = oaDb->database_count();
    gcs().send_text(MAV_SEVERITY_CRITICAL, "num obstacles %5.3f", (double)(num_obstacles));
    if (!oa_db_nodes.expand_to_hold(num_obstacles * num_points_per_circle)) {
        // err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
        return false;
    }

    bool oa_grouping[num_obstacles];
    memset(oa_grouping, 0, sizeof(oa_grouping));

    // iterate through exclusion circles and create outer polygon points
    for (uint16_t i = 0; i < num_obstacles; i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        Vector3f quad[4];
        const Vector3f circle_pos_cm = item.pos * 100.0f;
        const float radius = item.radius;
        // scaler to ensure lines between points do not intersect circle
        const float scaler = (1.0f / cosf(radians(180.0f / (float)num_points_per_circle))) * ((radius * 100.0f) + MARGIN_CM + 75);

        if (item.group_id != UINT16_MAX && 0) {
            if (oa_grouping[i]) {
                // already grouped
                continue;
            }
            oa_grouping[i] = true;
            for (uint8_t k = 0; k < num_points_per_circle; k++) {
                quad[k] = circle_pos_cm + (unit_offsets[k] * scaler);
            }
            uint16_t no_of_grouping = 0;
            for (uint16_t j = i+1; j < num_obstacles-i; j++) {
                const AP_OADatabase::OA_DbItem& item_j = oaDb->get_item(j);
                if (item_j.group_id == item.group_id) {
                    // match
                    no_of_grouping++;
                    oa_grouping[j] = true;
                    const float second_scaler = (1.0f / cosf(radians(180.0f / (float)num_points_per_circle))) * ((item_j.radius * 100.0f) + MARGIN_CM + 75);
                    Vector3f new_quad[4];
                    for (uint8_t k = 0; k < num_points_per_circle; k++) {
                        new_quad[k] = circle_pos_cm + (unit_offsets[k] * second_scaler);
                    }
                    find_max_quadrant_nodes(new_quad, quad);
                }
            }
            for (uint8_t j = 0; j < num_points_per_circle; j++) {
                oa_db_nodes[_total_oa_db_numpoints] = quad[j];
                _total_oa_db_numpoints++;
            }
            gcs().send_text(MAV_SEVERITY_CRITICAL, "grouped! %5.3f", (double)no_of_grouping);
        } else {
            // add points to array
            for (uint8_t j = 0; j < num_points_per_circle; j++) {
                oa_db_nodes[_total_oa_db_numpoints] = circle_pos_cm + (unit_offsets[j] * scaler);
                _total_oa_db_numpoints++;
            }
        }
    }

    // record fence update time so we don't process these exclusion circles again
    // _exclusion_proximity_update_ms = 

    return true;
}

void AP_OART_AStar::find_max_quadrant_nodes(Vector3f new_nodes[4], Vector3f quad_nodes[4])
{
    // first quadrant: +ve x, +ve y direction

    if (new_nodes[0].x > quad_nodes[0].x && new_nodes[0].y > quad_nodes[0].y) {
        quad_nodes[0] = new_nodes[0];
    }

    if (new_nodes[1].x > quad_nodes[1].x && new_nodes[1].y < quad_nodes[1].y) {
        quad_nodes[1] = new_nodes[1];
    }

    if (new_nodes[2].x < quad_nodes[2].x && new_nodes[2].y < quad_nodes[2].y) {
        quad_nodes[2] = new_nodes[2];
    }

    if (new_nodes[3].x < quad_nodes[3].x && new_nodes[3].y > quad_nodes[3].y) {
        quad_nodes[3] = new_nodes[3];
    }

}

bool AP_OART_AStar::create_oa_db_visgraph()
{
    // clear fence points visibility graph
    _oa_db_visgraph.clear();

    // calculate distance from each point to all other points in the obstacle database nodes
    for (uint16_t i = 0; i < _total_oa_db_numpoints - 1; i++) {
        const Vector3f start_seg = oa_db_nodes[i];
        for (uint16_t j = i + 1; j < _total_oa_db_numpoints; j++) {
            Vector3f end_seg = oa_db_nodes[j];
            if (!intersects_proximity_obstacle(start_seg, end_seg)) {
                if (!_oa_db_visgraph.add_item({AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i},
                                            {AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, j},
                                            (start_seg - end_seg).length())) {
                    // failure to add a point can only be caused by out-of-memory
                    return false;
                }
            }
        }
    }

    return true;
}

// bool AP_OART_AStar::create_visgraph_fast(const Vector3f &start_pos)
// {
//     // clear fence points visibility graph
//     _oa_db_visgraph.clear();

//     // exit immediately if db is empty
//     AP_OADatabase *oaDb = AP::oadatabase();
//     if (oaDb == nullptr || !oaDb->healthy()) {
//         return false;
//     }

//     uint16_t total_nodes = oaDb->database_count();

//     float angle_nodes[total_nodes];

//     // calculate distance from position to all inclusion/exclusion fence points
//     for (uint8_t i = 0; i < _total_oa_db_numpoints; i++) {
//         const Vector3f seg_end = oa_db_nodes[i];
//         const Vector3f line_seg = (seg_end - start_pos);
//         angle_nodes[i] = line_seg.angle_x();
//     }
//     sort_array(angle_nodes, total_nodes);
// }

// void AP_OART_AStar::sort_array(float arr[], uint16_t num)
// {
//     uint16_t i, j;
//     for (i = 0; i < num-1; i++){
//     // Last i elements are already in place
//         for (j = 0; j < num-i-1; j++) {
//             if (arr[j] > arr[j+1]) {
//                 float temp = arr[j];
//                 *&arr[j] = *&arr[j+1];
//                 *&arr[j+1] = temp;
//             }
//         }
//     }
// }

bool AP_OART_AStar::intersects_proximity_obstacle(const Vector3f &seg_start, const Vector3f &seg_end) const
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector3f point_cm = item.pos * 100.0f;
        // margin is distance between line segment and obstacle minus obstacle's radius
        const float m = Vector3f::closest_distance_between_line_and_point(seg_start, seg_end, point_cm) - item.radius*100.0f;
        if (m < MARGIN_CM) {
            return true;
        }
    }
    return false;
}


bool AP_OART_AStar::create_goal_and_destination_visgraph(const Vector3f &origin, const Vector3f &destination)
{
    // create visgraph from origin to all obstacles and destination
    if (!create_visgraph_from_point(_source_visgraph, {AP_OAVisGraph::OATYPE_SOURCE, 0}, origin, true, destination)) {
        return false;
    }

    // create a visgraph from the other end
    if (!create_visgraph_from_point(_destination_visgraph, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, destination)) {
        return false;
    }
    return true;
}

bool AP_OART_AStar::create_visgraph_from_point(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector3f &position, bool add_extra_position, Vector3f extra_position)
{
    // exit immediately if no fence (with margin) points
    if (_total_oa_db_numpoints == 0) {
        return false;
    }

    // clear visibility graph
    visgraph.clear();

    // calculate distance from position to all inclusion/exclusion fence points
    for (uint16_t i = 0; i < _total_oa_db_numpoints; i++) {
        Vector3f seg_end = oa_db_nodes[i];
        if (!intersects_proximity_obstacle(position, seg_end)) {
            // line segment does not intersect with fences so add to visgraph
            if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i}, (position - seg_end).length())) {
                return false;
            }
        }
    }

    // add extra point to visibility graph if it doesn't intersect with polygon fence or exclusion polygons
    if (add_extra_position) {
        if (!intersects_proximity_obstacle(position, extra_position)) {
            if (!visgraph.add_item(oaid, {AP_OAVisGraph::OATYPE_DESTINATION, 0}, (position - extra_position).length())) {
                return false;
            }
        }
    }

    return true;
}

bool AP_OART_AStar::find_shortest_path()
{
    // expand _short_path_data if necessary
    if (!_short_path_data.expand_to_hold(2 + _total_oa_db_numpoints)) {
        return false;
    }

    // add origin and destination (node_type, id, visited, distance_from_idx, distance_cm) to short_path_data array
    _short_path_data[0] = {{AP_OAVisGraph::OATYPE_SOURCE, 0}, false, 0, 0};
    _short_path_data[1] = {{AP_OAVisGraph::OATYPE_DESTINATION, 0}, false, UINT16_MAX, FLT_MAX};
    _short_path_data_numpoints = 2;

    // add all inclusion and exclusion fence points to short_path_data array (node_type, id, visited, distance_from_idx, distance_cm)
    for (uint16_t i=0; i<_total_oa_db_numpoints; i++) {
        _short_path_data[_short_path_data_numpoints++] = {{AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT, i}, false, UINT16_MAX, FLT_MAX};
    }

    // start algorithm from source point
    node_index current_node_idx = 0;

    // update nodes visible from source point
    for (uint16_t i = 0; i < _source_visgraph.num_items(); i++) {
        node_index node_idx;
        if (find_node_from_id(_source_visgraph[i].id2, node_idx)) {
            _short_path_data[node_idx].distance_cm = _source_visgraph[i].distance_cm;
            _short_path_data[node_idx].distance_from_idx = current_node_idx;
        } else {
            // err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
            return false;
        }
    }
    // mark source node as visited
    _short_path_data[current_node_idx].visited = true;

    // move current_node_idx to node with lowest distance
    while (find_closest_node_idx(current_node_idx)) {
        node_index dest_node;
        // See if this next "closest" node is actually the destination
        if (find_node_from_id({AP_OAVisGraph::OATYPE_DESTINATION,0}, dest_node) && current_node_idx == dest_node) {
            // We have discovered destination.. Don't bother with the rest of the graph
            break;
        }
        // update distances to all neighbours of current node
        update_visible_node_distances(current_node_idx);

        // mark current node as visited
        _short_path_data[current_node_idx].visited = true;
    }

    // extract path starting from destination
    bool success = false;
    node_index nidx;
    if (!find_node_from_id({AP_OAVisGraph::OATYPE_DESTINATION,0}, nidx)) {
        // err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
        return false;
    }
    _path_numpoints = 0;
    while (true) {
        if (!_path.expand_to_hold(_path_numpoints + 1)) {
            // err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_OUT_OF_MEMORY;
            return false;
        }
        // fail if newest node has invalid distance_from_index
        if ((_short_path_data[nidx].distance_from_idx == OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX) ||
            (_short_path_data[nidx].distance_cm >= FLT_MAX)) {
            break;
        } else {
            // add node's id to path array
            _path[_path_numpoints] = _short_path_data[nidx].id;
            _path_numpoints++;

            // we are done if node is the source
            if (_short_path_data[nidx].id.id_type == AP_OAVisGraph::OATYPE_SOURCE) {
                success = true;
                break;
            } else {
                // follow node's "distance_from_idx" to previous node on path
                nidx = _short_path_data[nidx].distance_from_idx;
            }
        }
    }
    // report error incase path not found
    // if (!success) {
    //     // err_id = AP_OADijkstra_Error::DIJKSTRA_ERROR_COULD_NOT_FIND_PATH;
    // }

    return success;
}


// find a node's index into _short_path_data array from it's id (i.e. id type and id number)
// returns true if successful and node_idx is updated
bool AP_OART_AStar::find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const
{
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        // source node is always the first node
        if (_short_path_data_numpoints > 0) {
            node_idx = 0;
            return true;
        }
        break;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        // destination is always the 2nd node
        if (_short_path_data_numpoints > 1) {
            node_idx = 1;
            return true;
        }
        break;
    case AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT:
        // intermediate nodes start from 3rd node
        if (_short_path_data_numpoints > id.id_num + 2) {
            node_idx = id.id_num + 2;
            return true;
        }
        break;
    }

    // could not find node
    return false;
}

// find index of node with lowest tentative distance (ignore visited nodes)
// returns true if successful and node_idx argument is updated
bool AP_OART_AStar::find_closest_node_idx(node_index &node_idx) const
{
    node_index lowest_idx = 0;
    float lowest_dist = FLT_MAX;

    // scan through all nodes looking for closest
    for (node_index i=0; i<_short_path_data_numpoints; i++) {
        const ShortPathNode &node = _short_path_data[i];
        if (node.visited || is_equal(_short_path_data[i].distance_cm, FLT_MAX)) {
            // if node is already visited OR cannot be reached yet, we can't use it
            continue;
        }
        // figure out the pos of this node
        Vector3f node_pos;
        float dist_with_heuristics = FLT_MAX;
        if (convert_node_to_point(node.id, node_pos)) {
            // heuristics is is simple Euclidean distance from the node to the destination
            // This should be admissible, therefore optimal path is guaranteed
            const float heuristics = (node_pos-_path_destination).length();
            dist_with_heuristics = node.distance_cm + heuristics;
        } else {
            // shouldn't happen
            return false;
        }
        if (dist_with_heuristics < lowest_dist) {
            // for NOW, this is the closest node
            lowest_idx = i;
            lowest_dist = dist_with_heuristics;
        }
    }

    if (lowest_dist < FLT_MAX) {
        // found the closest node
        node_idx = lowest_idx;
        return true;
    }
    return false;
}

// find the position of a node as an offset (in cm) from the ekf origin
bool AP_OART_AStar::convert_node_to_point(const AP_OAVisGraph::OAItemID& id, Vector3f& pos) const
{
    // convert id to a position offset from EKF origin
    switch (id.id_type) {
    case AP_OAVisGraph::OATYPE_SOURCE:
        pos = _path_source;
        return true;
    case AP_OAVisGraph::OATYPE_DESTINATION:
        pos = _path_destination;
        return true;
    case AP_OAVisGraph::OATYPE_INTERMEDIATE_POINT:
        pos = oa_db_nodes[id.id_num];
        return true;
    }

    // we should never reach here but just in case
    return false;
}

// update total distance for all nodes visible from current node
// curr_node_idx is an index into the _short_path_data array
void AP_OART_AStar::update_visible_node_distances(node_index curr_node_idx)
{
    // sanity check
    if (curr_node_idx >= _short_path_data_numpoints) {
        return;
    }

    // get current node for convenience
    const ShortPathNode &curr_node = _short_path_data[curr_node_idx];

    // for each visibility graph
    const AP_OAVisGraph* visgraphs[] = {&_oa_db_visgraph, &_destination_visgraph};
    for (uint8_t v=0; v<ARRAY_SIZE(visgraphs); v++) {

        // skip if empty
        const AP_OAVisGraph &curr_visgraph = *visgraphs[v];
        if (curr_visgraph.num_items() == 0) {
            continue;
        }

        // search visibility graph for items visible from current_node
        for (uint16_t i = 0; i < curr_visgraph.num_items(); i++) {
            const AP_OAVisGraph::VisGraphItem &item = curr_visgraph[i];
            // match if current node's id matches either of the id's in the graph (i.e. either end of the vector)
            if ((curr_node.id == item.id1) || (curr_node.id == item.id2)) {
                AP_OAVisGraph::OAItemID matching_id = (curr_node.id == item.id1) ? item.id2 : item.id1;
                // find item's id in node array
                node_index item_node_idx;
                if (find_node_from_id(matching_id, item_node_idx)) {
                    // if current node's distance + distance to item is less than item's current distance, update item's distance
                    const float dist_to_item_via_current_node = _short_path_data[curr_node_idx].distance_cm + item.distance_cm;
                    if (dist_to_item_via_current_node < _short_path_data[item_node_idx].distance_cm) {
                        // update item's distance and set "distance_from_idx" to current node's index
                        _short_path_data[item_node_idx].distance_cm = dist_to_item_via_current_node;
                        _short_path_data[item_node_idx].distance_from_idx = curr_node_idx;
                    }
                }
            }
        }
    }
}

void AP_OART_AStar::send_debug_info()
{
    gcs().send_named_float("Start", 0);
    for (uint16_t i=0; i<_total_oa_db_numpoints; i++) {
        const Vector3f vec = oa_db_nodes[i];
        gcs().send_named_float("Node_x", vec.x);
        gcs().send_named_float("Node_y", vec.y);
        gcs().send_named_float("Node_z", vec.z);
    }
    
    // for (uint)
    
}

// return point from final path as an offset (in cm) from the ekf origin
bool AP_OART_AStar::get_shortest_path_point(uint16_t point_num, Vector3f& pos)
{
    if ((_path_numpoints == 0) || (point_num >= _path_numpoints)) {
        return false;
    }

    // get id from path
    AP_OAVisGraph::OAItemID id = _path[_path_numpoints - point_num - 1];

    return convert_node_to_point(id, pos);
}