#include "airboss_utils.h"
#include "AP_Common/AP_Common.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AirBoss_Utils::var_info[] = {
    // --- Channel 1 ---
    // @Param: RC1_MIN
    // @DisplayName: RC Channel 1 minimum PWM
    // @Description: Minimum input PWM value for channel 1
    // @Range: 800 1200
    AP_GROUPINFO("1_MIN", 1, AirBoss_Utils, rc1_min, 1000),

    // @Param: RC1_MAX
    // @DisplayName: RC Channel 1 maximum PWM
    // @Description: Maximum input PWM value for channel 1
    // @Range: 1800 2200
    AP_GROUPINFO("1_MAX", 2, AirBoss_Utils, rc1_max, 2000),

    // @Param: RC1_TRIM
    // @DisplayName: RC Channel 1 trim PWM
    // @Description: Trim value (center) for channel 1
    // @Range: 1300 1700
    AP_GROUPINFO("1_TRIM", 3, AirBoss_Utils, rc1_trim, 1500),

    // --- Channel 2 ---
    AP_GROUPINFO("2_MIN", 4, AirBoss_Utils, rc2_min, 1000),
    AP_GROUPINFO("2_MAX", 5, AirBoss_Utils, rc2_max, 2000),
    AP_GROUPINFO("2_TRIM", 6, AirBoss_Utils, rc2_trim, 1500),

    // --- Channel 3 ---
    AP_GROUPINFO("3_MIN", 7, AirBoss_Utils, rc3_min, 1000),
    AP_GROUPINFO("3_MAX", 8, AirBoss_Utils, rc3_max, 2000),
    AP_GROUPINFO("3_TRIM", 9, AirBoss_Utils, rc3_trim, 1500),

    // --- Channel 4 ---
    AP_GROUPINFO("4_MIN", 10, AirBoss_Utils, rc4_min, 1000),
    AP_GROUPINFO("4_MAX", 11, AirBoss_Utils, rc4_max, 2000),
    AP_GROUPINFO("4_TRIM", 12, AirBoss_Utils, rc4_trim, 1500),

    AP_GROUPEND
};


AirBoss_Utils::AirBoss_Utils()
{
    memset(&rc_channels, 0, sizeof(rc_channels));
    AP_Param::setup_object_defaults(this, var_info);
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
   // Clamp input range
   norm = MAX(-1.0f, MIN(1.0f, norm));

   // Load calibration
   int16_t minv = 1000, maxv = 2000, trim = 1500;
   switch (ch_num) {
       case 1: minv = rc1_min; maxv = rc1_max; trim = rc1_trim; break;
       case 2: minv = rc2_min; maxv = rc2_max; trim = rc2_trim; break;
       case 3: minv = rc3_min; maxv = rc3_max; trim = rc3_trim; break;
       case 4: minv = rc4_min; maxv = rc4_max; trim = rc4_trim; break;
       default: return 1500;
   }

   // Compute scaling factors for each half
   const float neg_span = (float)(trim - minv);
   const float pos_span = (float)(maxv - trim);

   if (neg_span < 1.0f || pos_span < 1.0f) {
       return 1500;
   }

   // First convert normalized value into calibrated PWM
   float pwm_calibrated;
   if (norm < 0.0f) {
       pwm_calibrated = trim + norm * neg_span;    // move down toward min
   } else {
       pwm_calibrated = trim + norm * pos_span;    // move up toward max
   }

   // Now remap [minv..maxv] to [1000..2000],
   // ensuring min→1000, trim→1500, max→2000
   float pwm;
   if (pwm_calibrated <= trim) {
       pwm = 1000.0f + ((pwm_calibrated - minv) / neg_span) * 500.0f;
   } else {
       pwm = 1500.0f + ((pwm_calibrated - trim) / pos_span) * 500.0f;
   }

   pwm = MAX(1000.0f, MIN(2000.0f, pwm));
   return float_to_uint16(pwm);
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
    rc.chan5_raw = map_threeway_to_pwm(switches.get_threeway(AirBoss_Switches::Function::MODE_SELECT));
    rc.chan6_raw = map_threeway_to_pwm(switches.get_threeway(AirBoss_Switches::Function::KILL_SWITCH));
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