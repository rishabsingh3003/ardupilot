#pragma once

#include "AP_Proximity_Backend.h"

#if HAL_PROXIMITY_ENABLED
#if AP_SCRIPTING_ENABLED

class AP_Proximity_Lua : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

    // handle script messages
    bool handle_script_distance_msg(float dist_m, float yaw_deg, float pitch_deg) override;
    bool handle_script_3d_msg(Vector3f vec_to_obstacle) override;

private:

    AP_Proximity_Temp_Boundary temp_boundary;

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last script message received

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update of upward distance
    float _distance_upward;             // upward distance in meters
};

#endif // AP_SCRIPTING_ENABLED
#endif // HAL_PROXIMITY_ENABLED