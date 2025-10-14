#pragma once

#include "AP_HAL/AP_HAL.h"
#include "airboss_joystick.h"
#include "airboss_switches.h"
#include <utility>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

class AirBoss_Utils
{
public:
    AirBoss_Utils();

    /**
     * @brief Send RC_CHANNELS MAVLink message based on joystick and switch state
     * @param js_state   Current joystick state
     * @param switches   Switch state provider
     */
    void send_rc_channels_mavlink(const AirBoss_Joystick::JoystickState &js_state,
                                  const AirBoss_Switches &switches);

    static const struct AP_Param::GroupInfo var_info[];


private:

    /**
     * @brief Convert a normalized value [-1, 1] into calibrated PWM based on RCx_MIN/MAX/TRIM
     * @param norm   Input in range [-1, 1]
     * @param ch_num Channel number (1–4)
     * @return       Calibrated PWM value in microseconds
     */
    uint16_t normalized_to_pwm(float norm, uint8_t ch_num) const;

    /**
     * @brief Convert a three-way switch state into PWM value
     * @param s Switch3State::UP/MID/DOWN
     * @return PWM in µs
     */
    uint16_t map_threeway_to_pwm(AirBoss_Switches::Switch3State s) const;

    struct {
        mavlink_message_t msg;
        mavlink_status_t status;
        uint32_t last_heartbeat_ms;
    } rc_channels;
    
    // Parameters
    AP_Int16 rc1_min;
    AP_Int16 rc1_max;
    AP_Int16 rc1_trim;

    AP_Int16 rc2_min;
    AP_Int16 rc2_max;
    AP_Int16 rc2_trim;

    AP_Int16 rc3_min;
    AP_Int16 rc3_max;
    AP_Int16 rc3_trim;

    AP_Int16 rc4_min;
    AP_Int16 rc4_max;
    AP_Int16 rc4_trim;

};
