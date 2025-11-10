#pragma once

#include "AP_HAL/AP_HAL.h"

#define NUM_SWITCHES 12

extern const AP_HAL::HAL& hal;

class AirBoss_Switches {
public:
    AirBoss_Switches();

    void init();                         // setup and start debounce timer
    void update();                       // optional, 50 Hz slow loop
    bool get_state(uint8_t index) const; // debounced state (true = high)
    bool get_raw(uint8_t index) const;   // instant raw read

     // 3-way switch state
     enum class Switch3State : uint8_t {
        DOWN = 0,
        MID,
        UP
    };

    // Logical functionality mapping
    enum class Function : uint8_t {
        NONE = 0,
        KILL_SWITCH,
        MODE_SELECT,
        CAM_MODE,
        REC,
        CENTRE,
        UP,
        DOWN,
        LIGHTS, // special handling in code
        BEHIND_RIGHT,
        BEHIND_LEFT,
        EMERGENCY_KILL, // virtual function
        COUNT
    };

    // Check if a button-type function is pressed
    bool is_pressed(Function f) const;

    // Get 3-way switch position
    Switch3State get_switch_state(Function f) const;

    void print_states();

    uint16_t function_to_sbus(Function f) const;
    
    uint16_t compute_hid_buttons() const ;

private:
    struct Switch {
        int pin;               // integer GPIO ID from hwdef
        bool state;            // debounced
        bool last_raw;         // last sampled value
        uint64_t last_change;  // last raw change time
    };

    struct FunctionMapEntry {
        uint8_t low_pin;   // low position (or single input)
        uint8_t high_pin;  // high position (255 if unused)
    };
    FunctionMapEntry function_map[(uint8_t)Function::COUNT];


    Switch sw[NUM_SWITCHES];
    bool initialised = false;

    void timer_update();                 // 1 kHz debounce task

    Switch3State get_three_way_switch_state(uint8_t low_idx, uint8_t high_idx) const;

    Switch3State lights_state = Switch3State::DOWN;
    bool last_lights_pressed = false;
};
