#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_OAVisGraph.h"


/*
 * BendyRuler avoidance algorithm for avoiding the polygon and circular fence and dynamic objects detected by the proximity sensor
 */
class AP_OART_AStar {
public:
    AP_OART_AStar();

    /* Do not allow copies */
    AP_OART_AStar(const AP_OART_AStar &other) = delete;
    AP_OART_AStar &operator=(const AP_OART_AStar&) = delete;

    // send configuration info stored in front end parameters
    void set_config(float margin_max);

    // run background task to find best path and update avoidance_results
    // returns true and populates origin_new and destination_new if OA is required.  returns false if OA is not required
    bool update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, bool proximity_only);

    static const struct AP_Param::GroupInfo var_info[];

private:
    bool create_oa_database_nodes();
    bool create_oa_db_visgraph();
    bool intersects_proximity_obstacle(const Vector3f &seg_start, const Vector3f &seg_end) const;
    bool create_goal_and_destination_visgraph(const Vector3f &origin, const Vector3f &destination);
    bool create_visgraph_from_point(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector3f &position, bool add_extra_position = false, Vector3f extra_position = Vector3f(0,0,0));
    bool find_shortest_path();
    void find_max_quadrant_nodes(Vector3f new_nodes[4], Vector3f quad_nodes[4]);

    AP_OAVisGraph _oa_db_visgraph;
    AP_OAVisGraph _source_visgraph;         // holds distances from source point to all other nodes
    AP_OAVisGraph _destination_visgraph;    // holds distances from the destination to all other nodes

    typedef uint16_t node_index;         // indices into short path data
    struct ShortPathNode {
        AP_OAVisGraph::OAItemID id;     // unique id for node (combination of type and id number)
        bool visited;                   // true if all this node's neighbour's distances have been updated
        node_index distance_from_idx;   // index into _short_path_data from where distance was updated (or 255 if not set)
        float distance_cm;              // distance from source (number is tentative until this node is the current node and/or visited = true)
    };
    AP_ExpandingArray<ShortPathNode> _short_path_data;
    node_index _short_path_data_numpoints;  // number of elements in _short_path_data array

    AP_ExpandingArray<Vector3f> oa_db_nodes; // array of nodes corresponding to inclusion polygon points plus a margin
    uint16_t _total_oa_db_numpoints;   // number of points held in above array
    uint32_t _oa_db_nodes_update_ms;  // system time of boundary update from AC_Fence (used to detect changes to polygon fence)

    bool find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const;
    bool find_closest_node_idx(node_index &node_idx) const;
    bool convert_node_to_point(const AP_OAVisGraph::OAItemID& id, Vector3f& pos) const;
    void update_visible_node_distances(node_index curr_node_idx);
    bool get_shortest_path_point(uint16_t point_num, Vector3f& pos);

    bool create_visgraph_fast(const Vector3f &start_pos);
    void sort_array(float arr[], uint16_t num);

    void send_debug_info();

    Vector3f _path_destination;
    Vector3f _path_source;
    
    uint16_t _path_numpoints;  
    AP_ExpandingArray<AP_OAVisGraph::OAItemID> _path;   // ids of points on return path in reverse order (i.e. destination is first element)

};
