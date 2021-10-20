#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

/*
 * BendyRuler avoidance algorithm for avoiding the polygon and circular fence and dynamic objects detected by the proximity sensor
 */
class AP_OAPotentialField {
public:
    AP_OAPotentialField();

    /* Do not allow copies */
    AP_OAPotentialField(const AP_OAPotentialField &other) = delete;
    AP_OAPotentialField &operator=(const AP_OAPotentialField&) = delete;

    // send configuration info stored in front end parameters
    void set_config(float margin_max);

    // run background task to find best path and update avoidance_results
    // returns true and populates origin_new and destination_new if OA is required.  returns false if OA is not required
    bool update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, bool proximity_only);

    static const struct AP_Param::GroupInfo var_info[];

private:
    void calculate_attraction_field(Vector3f &current_pos, Vector3f &goal_pos, Vector3f &attraction_field);
    void calculate_repulsion_field(Vector3f &current_pos, Vector3f &repulsion_field);
    bool was_active;
};
