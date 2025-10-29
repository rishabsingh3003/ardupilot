#include "airboss_switches.h"

#define DEBOUNCE_MS 50

AirBoss_Switches::AirBoss_Switches() {
    for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
        sw[i].pin = i + 1;     // GPIOs 1..12 from hwdef
        sw[i].state = false;
        sw[i].last_raw = false;
        sw[i].last_change = 0;
    }

    // Default logical mapping
    for (uint8_t i = 0; i < (uint8_t)Function::COUNT; i++) {
        function_map[i] = {255, 255}; // unused
    }

    function_map[(uint8_t)Function::KILL_SWITCH]   = {8, 9};   // GPIO1, GPIO2
    function_map[(uint8_t)Function::MODE_SELECT]   = {4, 5};   // GPIO3, GPIO4
    function_map[(uint8_t)Function::CAM_MODE]      = {11, 255}; // GPIO5
    function_map[(uint8_t)Function::REC]           = {3, 255}; // GPIO6
    function_map[(uint8_t)Function::CENTRE]        = {2, 255}; // GPIO7
    function_map[(uint8_t)Function::UP]            = {10, 255}; // GPIO8
    function_map[(uint8_t)Function::DOWN]          = {12, 255}; // GPIO9
    function_map[(uint8_t)Function::LIGHTS]        = {1, 255}; // GPIO10
    function_map[(uint8_t)Function::BEHIND_RIGHT]  = {7, 255}; // GPIO11
    function_map[(uint8_t)Function::BEHIND_LEFT]   = {6, 255}; // GPIO12
}

void AirBoss_Switches::init() {
    if (initialised) return;
    initialised = true;

    // Configure pins as input with pull-ups
    for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
        hal.gpio->pinMode(sw[i].pin, HAL_GPIO_INPUT);
        hal.gpio->write(sw[i].pin, 1); // enable pull-up

        const bool raw = hal.gpio->read(sw[i].pin);
        sw[i].last_raw = raw;
        sw[i].state = raw;
        sw[i].last_change = AP_HAL::millis64();
    }

    // Run debounce at 1 kHz
    hal.scheduler->register_timer_process(
        FUNCTOR_BIND_MEMBER(&AirBoss_Switches::timer_update, void)
    );
}

void AirBoss_Switches::update() {
    // optional slow logic here (e.g. MAVLink report or logging)
}

void AirBoss_Switches::timer_update() {
    const uint64_t now = AP_HAL::millis64();
    for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
        bool raw = hal.gpio->read(sw[i].pin);

        if (raw != sw[i].last_raw) {
            sw[i].last_raw = raw;
            sw[i].last_change = now;
        } else if ((now - sw[i].last_change) > DEBOUNCE_MS) {
            sw[i].state = raw;
        }
    }
}

bool AirBoss_Switches::get_state(uint8_t index) const {
    if (index >= NUM_SWITCHES) return false;
    return !sw[index].state;
}

bool AirBoss_Switches::get_raw(uint8_t index) const {
    if (index >= NUM_SWITCHES) return false;
    return sw[index].last_raw;
}

AirBoss_Switches::Switch3State
AirBoss_Switches::get_three_way_switch_state(uint8_t low_idx, uint8_t high_idx) const {
    bool low  = get_state(low_idx);
    bool high = get_state(high_idx);

    if (low && !high)  return Switch3State::DOWN;
    if (!low && high)  return Switch3State::UP;
    return Switch3State::MID;
}

bool AirBoss_Switches::is_pressed(Function f) const {
    const auto &m = function_map[(uint8_t)f];
    if (m.low_pin == 255) return false;
    return get_state(m.low_pin);
}

AirBoss_Switches::Switch3State
AirBoss_Switches::get_switch_state(Function f) const {
    const auto &m = function_map[(uint8_t)f];
    if (m.high_pin == 255)
        return get_state(m.low_pin) ? Switch3State::UP : Switch3State::DOWN;
    return get_three_way_switch_state(m.low_pin, m.high_pin);
}

static const char* function_name(AirBoss_Switches::Function f)
{
    switch (f) {
    case AirBoss_Switches::Function::KILL_SWITCH:     return "KILL_SWITCH";
    case AirBoss_Switches::Function::MODE_SELECT:     return "MODE_SELECT";
    case AirBoss_Switches::Function::CAM_MODE:        return "CAM_MODE";
    case AirBoss_Switches::Function::REC:             return "REC";
    case AirBoss_Switches::Function::CENTRE:          return "CENTRE";
    case AirBoss_Switches::Function::UP:              return "UP";
    case AirBoss_Switches::Function::DOWN:            return "DOWN";
    case AirBoss_Switches::Function::LIGHTS:          return "LIGHTS";
    case AirBoss_Switches::Function::BEHIND_RIGHT:    return "BEHIND_RIGHT";
    case AirBoss_Switches::Function::BEHIND_LEFT:     return "BEHIND_LEFT";
    default:                                          return "UNKNOWN";
    }
}

uint16_t AirBoss_Switches::function_to_sbus(Function f) const
{
    uint16_t sbus_val = 992; // neutral by default
    Switch3State s3 = get_switch_state(f);
    switch (s3) {
        case Switch3State::DOWN: sbus_val = 172;  break;
        case Switch3State::MID:  sbus_val = 992;  break;
        case Switch3State::UP:   sbus_val = 1811; break;
    }
    return sbus_val;
}

void AirBoss_Switches::print_states()
{
    hal.console->printf("=== AirBoss Switches ===\n");

    for (uint8_t i = 1; i < (uint8_t)Function::COUNT; i++) {
        Function f = static_cast<Function>(i);
        const char* name = function_name(f);
        auto state = get_switch_state(f);
        const char* s =
            (state == Switch3State::UP)   ? "UP" :
            (state == Switch3State::MID)  ? "MID" : "DOWN";
        hal.console->printf("%-14s : %s\n", name, s);
    }
    hal.console->printf("========================\n");
}
