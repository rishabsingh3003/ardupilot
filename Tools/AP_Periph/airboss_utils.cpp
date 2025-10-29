#include "airboss_utils.h"
#include "AP_Common/AP_Common.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;


AirBoss_Utils::AirBoss_Utils()
{
    memset(&rc_channels, 0, sizeof(rc_channels));
}

uint16_t AirBoss_Utils::map_threeway_to_pwm(AirBoss_Switches::Switch3State s) const
{
    switch (s) {
        case AirBoss_Switches::Switch3State::UP:   return 2000;
        case AirBoss_Switches::Switch3State::MID:  return 1500;
        case AirBoss_Switches::Switch3State::DOWN: return 1000;
        default: return 1500;
    }
}


uint16_t AirBoss_Utils::normalized_to_pwm(float norm, uint8_t ch_num) const
{
   return (uint16_t)linear_interpolate(1000, 2000, norm, -1.0f, 1.0f);
}

void AirBoss_Utils::send_rc_channels_mavlink(const AirBoss_Joystick::JoystickState &js_state,
    const AirBoss_Switches &switches)
{
    auto *uart = hal.serial(0); // typically USB
    if (uart == nullptr) {
    return;
    }

    mavlink_rc_channels_t rc{};
    rc.time_boot_ms = AP_HAL::millis();
    rc.chancount    = 8;
    rc.rssi         = 255;

    // ---- Joystick mapping ----
    rc.chan1_raw = normalized_to_pwm(js_state.right_thumb.x.norm, 1); // roll
    rc.chan2_raw = normalized_to_pwm(js_state.right_thumb.y.norm, 2); // pitch
    rc.chan3_raw = normalized_to_pwm(js_state.left_thumb.y.norm,  3); // throttle
    rc.chan4_raw = normalized_to_pwm(js_state.left_thumb.x.norm,  4); // yaw

    // ---- Switches mapping ----
    rc.chan5_raw = map_threeway_to_pwm(switches.get_switch_state(AirBoss_Switches::Function::MODE_SELECT));
    rc.chan6_raw = map_threeway_to_pwm(switches.get_switch_state(AirBoss_Switches::Function::KILL_SWITCH));
    rc.chan7_raw = switches.is_pressed(AirBoss_Switches::Function::LIGHTS)      ? 2000 : 1000;
    rc.chan8_raw = switches.is_pressed(AirBoss_Switches::Function::REC)         ? 2000 : 1000;

    // zero out unused
    rc.chan9_raw  = rc.chan10_raw = rc.chan11_raw = rc.chan12_raw = 0;
    rc.chan13_raw = rc.chan14_raw = rc.chan15_raw = rc.chan16_raw = 0;
    rc.chan17_raw = rc.chan18_raw = 0;

    mavlink_message_t msg;
    // encode using your generated header
    uint16_t len = mavlink_msg_rc_channels_encode_status(
        1,                        // system_id
        1,   // component_id
        &rc_channels.status, &msg, &rc);

    // send raw bytes over USB serial
    uart->write((uint8_t*)&msg.magic, len);

    uart->write(reinterpret_cast<uint8_t*>(&msg.magic), len);
}