/*
 Class to handle AirBoss Joystick via SPI based ADC
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#ifndef AP_PERIPH_AIRBOSS_JOYSTICK_ENABLED
#define AP_PERIPH_AIRBOSS_JOYSTICK_ENABLED 1
#endif


#if AP_PERIPH_AIRBOSS_JOYSTICK_ENABLED

class AirBoss_Joystick {
public:
    AirBoss_Joystick();

    void init();
    void configure();
    void adc_timer();

    bool healthy() { return _state.healthy; }
    uint32_t last_update_us() { return _state.last_update_us; }

    struct JoystickAxis {
        uint16_t raw;   // raw 12-bit ADC
        float norm;     // normalized -1..+1
    };

    struct JoystickStick {
        JoystickAxis x;
        JoystickAxis y;
    };

    struct JoystickState {
        JoystickStick left_thumb;
        JoystickStick right_thumb;
        JoystickStick left_index;
        JoystickStick right_index;

        uint8_t healthy;
        uint32_t last_update_us;
    };

    const JoystickState& get_state() { return _state; }
    void print_states();

    static const struct AP_Param::GroupInfo var_info[];

private:
    bool read_channel(uint16_t cmd, uint16_t &val);
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    bool _configured;

    JoystickState _state;

    // Parameters
    AP_Int16 rc_min[8];
    AP_Int16 rc_max[8];
    AP_Int16 rc_trim[8];

    // Logical order mapping: [left_thumb.x, left_thumb.y, right_thumb.x, right_thumb.y, left_index.x, left_index.y, right_index.x, right_index.y]
    // Each entry is the ADC channel index (0–7)
    uint8_t channel_map[8] = {
        0, 1,   // Left thumb
        4, 5,   // Right thumb
        2, 3,   // Left index
        6, 7    // Right index
    };

    // ArduPilot parameter channel mapping. This has to be different from channel_map because some axes are fixed like Throttle is always channel 3
    uint8_t parameter_channel_map[8] = {
        2, 3,   // Left thumb
        0, 1,   // Right thumb
        4, 5,   // Left index
        6, 7    // Right index
    };

    // Optional inversion flags (1 = invert, 0 = normal)
    bool invert_axis[8] = {
        false, false,   // Left thumb
        true,  false,   // Right thumb (invert X for example)
        false, true,    // Left index (invert Y for example)
        false, false    // Right index
    };

};

#endif // AP_PERIPH_AIRBOSS_JOYSTICK_ENABLED

