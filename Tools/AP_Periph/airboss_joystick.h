/*
 Class to handle AirBoss Joystick via SPI based ADC
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

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
        JoystickStick left_front;
        JoystickStick right_front;
        JoystickStick left_rear;
        JoystickStick right_rear;

        bool healthy;
        uint32_t last_update_us;
    };

    const JoystickState& get_state() { return _state; }

private:
    bool read_channel(uint16_t cmd, uint16_t &val);
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    bool _configured;


    JoystickState _state;


};

#endif // AP_PERIPH_AIRBOSS_JOYSTICK_ENABLED

