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

#include "AP_OAPotentialField.h"
#include <AC_Avoidance/AP_OADatabase.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

// parameter defaults
const float K_ATT = 5.0f;
const float K_REPP = 350.0f;
const float OBSTACLE_THRESH = 100.0f;


#define VERTICAL_ENABLED APM_BUILD_TYPE(APM_BUILD_ArduCopter)

const AP_Param::GroupInfo AP_OAPotentialField::var_info[] = {

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

AP_OAPotentialField::AP_OAPotentialField() 
{ 
    // AP_Param::setup_object_defaults(this, var_info); 

}

// run background task to find best path and update avoidance_results
// returns true and updates origin_new and destination_new if a best path has been found
// bendy_type is set to the type of BendyRuler used
bool AP_OAPotentialField::update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, bool proximity_only)
{
    // bendy ruler always sets origin to current_loc
    origin_new = current_loc;

    Vector3f current_pos,goal_pos;
    if (!current_loc.get_vector_from_origin_NEU(current_pos) || !destination.get_vector_from_origin_NEU(goal_pos)) {
        return false;
    }
    current_pos *= 0.01f;
    goal_pos *= 0.01f;

    Vector3f attraction_field, repulsion_field;
    calculate_attraction_field(current_pos, goal_pos, attraction_field);
    calculate_repulsion_field(current_pos, repulsion_field);

    const Vector3f total_potential = attraction_field - repulsion_field;
    // float angle_diff = total_potential.angle((goal_pos-current_pos));
    gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)repulsion_field.length());

    if (repulsion_field.length() < 0.55) {
        was_active = false;
    }
    if (repulsion_field.length() > 1.5f || was_active) {
        was_active = true;
        // we are facing significant repulsion from obstacles. Lets start OA
        destination_new = origin_new;
        destination_new.offset(-total_potential.y, total_potential.x);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "repp! %5.3f", (double)repulsion_field.length());
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "total_potential.x! %5.3f", (double)total_potential.x);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "total_potential.y! %5.3f", (double)total_potential.y);
        return true;
    }

    return false;
}

void AP_OAPotentialField::calculate_attraction_field(Vector3f &current_pos, Vector3f &goal_pos, Vector3f &attraction_field)
{
    attraction_field = (goal_pos-current_pos) * K_ATT;
}

void AP_OAPotentialField::calculate_repulsion_field(Vector3f &current_pos, Vector3f &repulsion_field)
{
// exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }

    Vector3f repulsion;

    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);

        const Vector3f obstacle_vector = (current_pos-item.pos);
        // margin is distance between line segment and obstacle minus obstacle's radius
        const float m = (obstacle_vector).length() - item.radius;
        if (is_zero(m)) {
            // should not be possible
            continue;
        }
        if (m > OBSTACLE_THRESH) {
            // we don't care about this obstacle, its too far away
            continue;
        }

        const Vector3f direction_to_obstacle = obstacle_vector.normalized();
        repulsion += direction_to_obstacle * K_REPP * (1/m - 1/OBSTACLE_THRESH) / (m*m);
    }
    repulsion_field = repulsion;

}