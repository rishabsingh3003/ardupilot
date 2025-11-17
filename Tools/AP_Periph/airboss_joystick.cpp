/*
  Class to handle AirBoss Joystick via SPI based ADC
 */


#include "airboss_joystick.h"
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <utility>

#define ADC_SPI_JOYSTICK "joysticks_adc"

// ==== Register macros ==== (Copied from Airboss STM32 version)
#define JOYSTICK_0   (0x00 << 7)
#define JOYSTICK_1   (0x01 << 7)
#define JOYSTICK_2   (0x02 << 7)
#define JOYSTICK_3   (0x03 << 7)
#define JOYSTICK_4   (0x06 << 7)
#define JOYSTICK_5   (0x07 << 7)
#define JOYSTICK_6   (0x04 << 7)
#define JOYSTICK_7   (0x05 << 7)

#define ADS_MANUAL_MODE                (0x01 << 12)
#define ADS_ENABLE_PROGRAMM_BITS       (0x01 << 11)
#define ADS_5_INPUT_RANGE              (0x01 << 6)
#define ADS_NO_POWER_DOWN              (0x00 << 5)

#define JOYSTICK_0_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_0 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )
#define JOYSTICK_1_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_1 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )
#define JOYSTICK_2_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_2 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )
#define JOYSTICK_3_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_3 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )
#define JOYSTICK_4_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_4 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )
#define JOYSTICK_5_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_5 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )
#define JOYSTICK_6_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_6 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )
#define JOYSTICK_7_VOLTAGE_REG ( ADS_MANUAL_MODE + ADS_ENABLE_PROGRAMM_BITS + JOYSTICK_7 + ADS_5_INPUT_RANGE + ADS_NO_POWER_DOWN )

static const uint16_t joystick_cmds[8] = {
    JOYSTICK_0_VOLTAGE_REG,
    JOYSTICK_1_VOLTAGE_REG,
    JOYSTICK_2_VOLTAGE_REG,
    JOYSTICK_3_VOLTAGE_REG,
    JOYSTICK_4_VOLTAGE_REG,
    JOYSTICK_5_VOLTAGE_REG,
    JOYSTICK_6_VOLTAGE_REG,
    JOYSTICK_7_VOLTAGE_REG,
};

#define ADC_MAX_VALUE 4095.0f
#define ADC_MID_VALUE 2048.0f
#define ADC_MIN_VALUE 0.0f

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AirBoss_Joystick::var_info[] = {

    // ───────────────────────────────────────────────
    // Left Thumb X (Yaw)
    // ───────────────────────────────────────────────
    // @Param: LTX_MIN
    // @DisplayName: Left Thumb X Minimum
    // @Description: Minimum ADC value for Left Thumb X axis
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("LTX_MIN", 1, AirBoss_Joystick, rc_min[0], ADC_MIN_VALUE),

    // @Param: LTX_MAX
    // @DisplayName: Left Thumb X Maximum
    // @Description: Maximum ADC value for Left Thumb X axis
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("LTX_MAX", 2, AirBoss_Joystick, rc_max[0], ADC_MAX_VALUE),

    // @Param: LTX_TRIM
    // @DisplayName: Left Thumb X Trim
    // @Description: Center ADC value for Left Thumb X axis
    // @Range: 0 4095
    // @User: Standard
    AP_GROUPINFO("LTX_TRIM", 3, AirBoss_Joystick, rc_trim[0], ADC_MID_VALUE),

    // @Param: LTX_REV
    // @DisplayName: Left Thumb X Reverse
    // @Description: Reverse direction of Left Thumb X axis
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("LTX_REV", 4, AirBoss_Joystick, rc_rev[0], 0),

    // ───────────────────────────────────────────────
    // Left Thumb Y (Throttle)
    // ───────────────────────────────────────────────
    AP_GROUPINFO("LTY_MIN", 5, AirBoss_Joystick, rc_min[1], ADC_MIN_VALUE),
    AP_GROUPINFO("LTY_MAX", 6, AirBoss_Joystick, rc_max[1], ADC_MAX_VALUE),
    AP_GROUPINFO("LTY_TRIM", 7, AirBoss_Joystick, rc_trim[1], ADC_MID_VALUE),
    AP_GROUPINFO("LTY_REV", 8, AirBoss_Joystick, rc_rev[1], 0),

    // ───────────────────────────────────────────────
    // Right Thumb X (Roll)
    // ───────────────────────────────────────────────
    AP_GROUPINFO("RTX_MIN", 9, AirBoss_Joystick, rc_min[2], ADC_MIN_VALUE),
    AP_GROUPINFO("RTX_MAX", 10, AirBoss_Joystick, rc_max[2], ADC_MAX_VALUE),
    AP_GROUPINFO("RTX_TRIM", 11, AirBoss_Joystick, rc_trim[2], ADC_MID_VALUE),
    AP_GROUPINFO("RTX_REV", 12, AirBoss_Joystick, rc_rev[2], 0),

    // ───────────────────────────────────────────────
    // Right Thumb Y (Pitch)
    // ───────────────────────────────────────────────
    AP_GROUPINFO("RTY_MIN", 13, AirBoss_Joystick, rc_min[3], ADC_MIN_VALUE),
    AP_GROUPINFO("RTY_MAX", 14, AirBoss_Joystick, rc_max[3], ADC_MAX_VALUE),
    AP_GROUPINFO("RTY_TRIM", 15, AirBoss_Joystick, rc_trim[3], ADC_MID_VALUE),
    AP_GROUPINFO("RTY_REV", 16, AirBoss_Joystick, rc_rev[3], 0),

    // ───────────────────────────────────────────────
    // Left Index X
    // ───────────────────────────────────────────────
    AP_GROUPINFO("LIX_MIN", 17, AirBoss_Joystick, rc_min[4], ADC_MIN_VALUE),
    AP_GROUPINFO("LIX_MAX", 18, AirBoss_Joystick, rc_max[4], ADC_MAX_VALUE),
    AP_GROUPINFO("LIX_TRIM", 19, AirBoss_Joystick, rc_trim[4], ADC_MID_VALUE),
    AP_GROUPINFO("LIX_REV", 20, AirBoss_Joystick, rc_rev[4], 0),

    // ───────────────────────────────────────────────
    // Left Index Y
    // ───────────────────────────────────────────────
    AP_GROUPINFO("LIY_MIN", 21, AirBoss_Joystick, rc_min[5], ADC_MIN_VALUE),
    AP_GROUPINFO("LIY_MAX", 22, AirBoss_Joystick, rc_max[5], ADC_MAX_VALUE),
    AP_GROUPINFO("LIY_TRIM", 23, AirBoss_Joystick, rc_trim[5], ADC_MID_VALUE),
    AP_GROUPINFO("LIY_REV", 24, AirBoss_Joystick, rc_rev[5], 0),

    // ───────────────────────────────────────────────
    // Right Index X
    // ───────────────────────────────────────────────
    AP_GROUPINFO("RIX_MIN", 25, AirBoss_Joystick, rc_min[6], ADC_MIN_VALUE),
    AP_GROUPINFO("RIX_MAX", 26, AirBoss_Joystick, rc_max[6], ADC_MAX_VALUE),
    AP_GROUPINFO("RIX_TRIM", 27, AirBoss_Joystick, rc_trim[6], ADC_MID_VALUE),
    AP_GROUPINFO("RIX_REV", 28, AirBoss_Joystick, rc_rev[6], 0),

    // ───────────────────────────────────────────────
    // Right Index Y
    // ───────────────────────────────────────────────
    AP_GROUPINFO("RIY_MIN", 29, AirBoss_Joystick, rc_min[7], ADC_MIN_VALUE),
    AP_GROUPINFO("RIY_MAX", 30, AirBoss_Joystick, rc_max[7], ADC_MAX_VALUE),
    AP_GROUPINFO("RIY_TRIM", 31, AirBoss_Joystick, rc_trim[7], ADC_MID_VALUE),
    AP_GROUPINFO("RIY_REV", 32, AirBoss_Joystick, rc_rev[7], 0),

    // ───────────────────────────────────────────────
    // Miscellaneous
    // ───────────────────────────────────────────────
    // @Param: RESET
    // @DisplayName: Reset joystick calibration
    // @Description: Set to 1 to reset all min/max/trim/rev values to defaults
    // @Values: 0:Normal,1:Reset
    // @User: Standard
    AP_GROUPINFO("RESET", 33, AirBoss_Joystick, _reset, 0),

    // @Param: RATE_LIM
    // @DisplayName: Axis Rate Limit
    // @Description: Maximum rate of change for joystick axes in normalized units per second (0 = no limit)
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("RATE_LIM", 34, AirBoss_Joystick, _rate_limit, 0.0f),

    // @Param: EXPO
    // @DisplayName: Expo to apply to axes
    // @Description: Exponential curve to apply to joystick axes (0 = linear, 1 = max expo)
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("EXPO", 35, AirBoss_Joystick, _expo, 0.0f),

    AP_GROUPEND
};


AirBoss_Joystick::AirBoss_Joystick() :
    dev(nullptr),
    _configured(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AirBoss_Joystick::init()
{
    dev = std::move(hal.spi->get_device(ADC_SPI_JOYSTICK));
    if(!dev) {
        hal.console->println("AirBoss_Joystick: failed to get SPI device");
        return;
    }
    // dev->set_read_flag(0);

    // Run periodic reads at 100Hz
    dev->register_periodic_callback(
        10000,  // 10 ms
        FUNCTOR_BIND_MEMBER(&AirBoss_Joystick::adc_timer, void)
    );
}

bool AirBoss_Joystick::read_channel(uint16_t cmd, uint16_t &val)
{
    uint8_t tx[2] = { uint8_t(cmd >> 8), uint8_t(cmd & 0xFF) };
    uint8_t rx[2] = {0};

    if (!dev->transfer_fullduplex(tx, rx, 2)) {
        return false;
    }

    // 16-bit word from the wire
    const uint16_t raw = (uint16_t(rx[0]) << 8) | rx[1];

    // DI4==0 mode: DO15..12 = channel-id, DO11..0 = conversion
    val = raw & 0x0FFF;            // 12-bit result
    // (Optional) uint8_t ch_id = (raw >> 12) & 0xF; // returned channel tag

    return true;
}

void AirBoss_Joystick::update()
{
    if (_reset.get() != 0) {
        // Reset parameters to defaults
        for (uint8_t i = 0; i < 8; i++)
        {
            rc_min[i].set_and_save(ADC_MIN_VALUE);
            rc_max[i].set_and_save(ADC_MAX_VALUE);
            rc_trim[i].set_and_save(ADC_MID_VALUE);
            rc_rev[i].set_and_save(0);
        }
        hal.console->println("AirBoss_Joystick: calibration reset");
        _reset.set_and_save(0);
    }

    WITH_SEMAPHORE(_state_sem);
    _state.left_thumb.x.norm = normalize_adc_input(
        _state.left_thumb.x.filter.get(),
        static_cast<uint16_t>(rc_min[0].get()),
        static_cast<uint16_t>(rc_max[0].get()),
        static_cast<uint16_t>(rc_trim[0].get()),
        (rc_rev[0].get() != 0)
    );
    _state.left_thumb.y.norm = normalize_adc_input(
        _state.left_thumb.y.filter.get(),
        static_cast<uint16_t>(rc_min[1].get()),
        static_cast<uint16_t>(rc_max[1].get()),
        static_cast<uint16_t>(rc_trim[1].get()),
        (rc_rev[1].get() != 0)
    );
    _state.right_thumb.x.norm = normalize_adc_input(
        _state.right_thumb.x.filter.get(),
        static_cast<uint16_t>(rc_min[2].get()),
        static_cast<uint16_t>(rc_max[2].get()),
        static_cast<uint16_t>(rc_trim[2].get()),
        (rc_rev[2].get() != 0)
    );
    _state.right_thumb.y.norm = normalize_adc_input(
        _state.right_thumb.y.filter.get(),
        static_cast<uint16_t>(rc_min[3].get()),
        static_cast<uint16_t>(rc_max[3].get()),
        static_cast<uint16_t>(rc_trim[3].get()),
        (rc_rev[3].get() != 0)
    );
    _state.left_index.x.norm = normalize_adc_input(
        _state.left_index.x.filter.get(),
        static_cast<uint16_t>(rc_min[4].get()),
        static_cast<uint16_t>(rc_max[4].get()),
        static_cast<uint16_t>(rc_trim[4].get()),
        (rc_rev[4].get() != 0)
    );
    _state.left_index.y.norm = normalize_adc_input(
        _state.left_index.y.filter.get(),
        static_cast<uint16_t>(rc_min[5].get()),
        static_cast<uint16_t>(rc_max[5].get()),
        static_cast<uint16_t>(rc_trim[5].get()),
        (rc_rev[5].get() != 0)
    );
    _state.right_index.x.norm = normalize_adc_input(
        _state.right_index.x.filter.get(),
        static_cast<uint16_t>(rc_min[6].get()),
        static_cast<uint16_t>(rc_max[6].get()),
        static_cast<uint16_t>(rc_trim[6].get()),
        (rc_rev[6].get() != 0)
    );
    _state.right_index.y.norm = normalize_adc_input(
        _state.right_index.y.filter.get(),
        static_cast<uint16_t>(rc_min[7].get()),
        static_cast<uint16_t>(rc_max[7].get()),
        static_cast<uint16_t>(rc_trim[7].get()),
        (rc_rev[7].get() != 0)
    );
    rate_limit_axes(AP_HAL::millis());
}

// Normalizes a raw ADC joystick input (0–4095) into [-1.0, 1.0] range
// applying calibration (min/max/trim) and inversion (rev flag).
float AirBoss_Joystick::normalize_adc_input(uint16_t raw,
                                 uint16_t min,
                                 uint16_t max,
                                 uint16_t trim,
                                 bool reversed)
{
    // Prevent invalid ranges
    if (max <= min) {
        return 0.0f;
    }

    const float fmin  = static_cast<float>(min);
    const float fmax  = static_cast<float>(max);
    const float ftrim = static_cast<float>(trim);
    const float fraw  = static_cast<float>(raw);

    // Compute negative and positive ranges (trim acts as zero point)
    const float range_neg = (ftrim > fmin) ? (ftrim - fmin) : 1.0f;
    const float range_pos = (fmax  > ftrim) ? (fmax - ftrim) : 1.0f;

    float norm = 0.0f;

    if (fraw >= ftrim) {
        norm = (fraw - ftrim) / range_pos;
    } else {
        norm = -(ftrim - fraw) / range_neg;
    }

    // Clamp to [-1, 1]
    if (norm > 1.0f)  norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    // Apply inversion if required
    if (reversed) {
        norm = -norm;
    }

    // Apply expo if set
    norm = apply_expo(norm, _expo.get());

    return norm;
}

float AirBoss_Joystick::apply_expo(float x, float expo)
{
    // Clamp input
    if (x > 1.0f) {
        x = 1.0f;
    }
    if (x < -1.0f) {
        x = -1.0f;
    }

    // Expo must be 0..1
    if (expo < 0.0f) {
        expo = 0.0f;
    }

    if (expo > 1.0f) {
        expo = 1.0f;
    }

    // soft center, fast edges
    return (x * (1.0f - expo) + (x * x * x) * expo);
}

void AirBoss_Joystick::rate_limit_axes(float now_ms)
{
    if (_rate_limit.get() <= 0.0f) {
        // No rate limiting
        return;
    }

    const float signal_timeout_ms = 500;  // reset after inactivity
    const float reset_threshold = 1.5f;   // full reset if jump > 0.5 in norm space

    // Flatten access to all 8 normalized channels for simplicity
    float* axes[8] = {
        &_state.left_thumb.x.norm,
        &_state.left_thumb.y.norm,
        &_state.right_thumb.x.norm,
        &_state.right_thumb.y.norm,
        &_state.left_index.x.norm,
        &_state.left_index.y.norm,
        &_state.right_index.x.norm,
        &_state.right_index.y.norm
    };

    for (int i = 0; i < 8; i++) {
        float in = *axes[i];
        float prev = filter_norm_hist[i];
        float dt = now_ms - last_update_filter_time_ms[i];

        // Reset if first time or timeout
        if (dt > signal_timeout_ms || dt <= 0 || last_update_filter_time_ms[i] == 0) {
            filter_norm_hist[i] = in;
            last_update_filter_time_ms[i] = now_ms;
            continue;
        }

        // Reset on sudden jumps
        if (fabsf(in - prev) > reset_threshold) {
            filter_norm_hist[i] = in;
            last_update_filter_time_ms[i] = now_ms;
            continue;
        }

        // Rate limiting (clamp delta per dt)
        float max_delta = _rate_limit * (dt / 1000.0f);  // normalized units per sec
        if (max_delta < 0.01f) max_delta = 0.01f;

        float delta = in - prev;
        if (delta > max_delta) {
            delta = max_delta;
        } else if (delta < -max_delta) {
            delta = -max_delta;
        }
        filter_norm_hist[i] = prev + delta;
        *axes[i] = filter_norm_hist[i];  // write filtered value back
        last_update_filter_time_ms[i] = now_ms;
    }
}

void AirBoss_Joystick::adc_timer()
{
    WITH_SEMAPHORE(_state_sem);
    uint16_t raw_values[8];
    // float    norm_values[8];

    // Clear (portable)
    for (uint8_t i = 0; i < 8; i++) { raw_values[i] = 0xFFFF; }

    // ---- ADS7951 pipeline handling (N+2) ----
    uint16_t tmp;

    // Prime the pipeline with the first TWO commands (results not valid yet)
    read_channel(joystick_cmds[0], tmp);  // frame 0 -> no valid result for cmd0 yet
    read_channel(joystick_cmds[1], tmp);  // frame 1 -> still no valid result for cmd0

    // Now stream the 8 commands; on each frame i, we get the result for i-2
    for (uint8_t i = 0; i < 8; i++) {
        if (!read_channel(joystick_cmds[i], tmp)) {
            _state.healthy = false;
            return;
        }
        if (i >= 2) {
            raw_values[i - 2] = tmp;                       // aligned result
        }
    }

    // Flush last TWO results (for the last two commands sent)
    if (!read_channel(joystick_cmds[0], tmp)) { _state.healthy = false; return; }
    raw_values[6] = tmp;

    if (!read_channel(joystick_cmds[0], tmp)) { _state.healthy = false; return; }
    raw_values[7] = tmp;

    // // ---- Normalize all axes ----
    // for (uint8_t i = 0; i < 8; i++) {
    //     uint8_t channel_id = channel_map[i];
    //     norm_values[i] = normalize_adc_input(
    //         raw_values[channel_id],
    //         static_cast<uint16_t>(rc_min[channel_id].get()),
    //         static_cast<uint16_t>(rc_max[channel_id].get()),
    //         static_cast<uint16_t>(rc_trim[channel_id].get()),
    //         (rc_rev[channel_id].get() != 0)
    //     );
    // }

    // Map channels to sticks (keep your chosen wiring order)
    _state.left_thumb.x.raw  = raw_values[channel_map[0]];
    _state.left_thumb.x.filter.apply(_state.left_thumb.x.raw);
    // _state.left_thumb.x.norm = norm_values[0];

    _state.left_thumb.y.raw  = raw_values[channel_map[1]];
    _state.left_thumb.y.filter.apply(_state.left_thumb.y.raw);
    // _state.left_thumb.y.norm = norm_values[1];

    _state.right_thumb.x.raw  = raw_values[channel_map[2]];
    _state.right_thumb.x.filter.apply(_state.right_thumb.x.raw);
    // _state.right_thumb.x.norm = norm_values[2];

    _state.right_thumb.y.raw  = raw_values[channel_map[3]];
    _state.right_thumb.y.filter.apply(_state.right_thumb.y.raw);
    // _state.right_thumb.y.norm = norm_values[3];

    _state.left_index.x.raw  = raw_values[channel_map[4]];
    _state.left_index.x.filter.apply(_state.left_index.x.raw);
    // _state.left_index.x.norm = norm_values[4];

    _state.left_index.y.raw  = raw_values[channel_map[5]];
    _state.left_index.y.filter.apply(_state.left_index.y.raw);
    // _state.left_index.y.norm = norm_values[5];

    _state.right_index.x.raw  = raw_values[channel_map[6]];
    _state.right_index.x.filter.apply(_state.right_index.x.raw);
    // _state.right_index.x.norm = norm_values[6];

    _state.right_index.y.raw  = raw_values[channel_map[7]];
    _state.right_index.y.filter.apply(_state.right_index.y.raw);
    // _state.right_index.y.norm = norm_values[7];

    
    // _state.left_thumb.y   = { raw_values[channel_map[1]], norm_values[1] };

    // _state.right_thumb.x  = { raw_values[channel_map[2]], norm_values[2] };
    // _state.right_thumb.y  = { raw_values[channel_map[3]], norm_values[3] };

    // _state.left_index.x   = { raw_values[channel_map[4]], norm_values[4] };
    // _state.left_index.y   = { raw_values[channel_map[5]], norm_values[5] };

    // _state.right_index.x  = { raw_values[channel_map[6]], norm_values[6] };
    // _state.right_index.y  = { raw_values[channel_map[7]], norm_values[7] };


    _state.healthy = true;
    _state.last_update_us = AP_HAL::micros();
}

uint16_t AirBoss_Joystick::axis_to_sbus(float norm) const
{
    // Clamp normalized input
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    // Map -1..+1 → SBUS 172..1811 (≈1000–2000 µs)
    return static_cast<uint16_t>((norm + 1.0f) * 0.5f * (1811 - 172) + 172);
}

void AirBoss_Joystick::print_states()
{
    hal.console->printf("=== AirBoss Joystick ===\n");

    const JoystickState &js = _state;  // current state

    // Helper lambda to print both raw and normalized values for a stick
    auto print_stick = [&](const char *name, const JoystickStick &stick) {
        hal.console->printf("%-14s : X=%4u (%.3f)  Y=%4u (%.3f)\n",
            name,
            (unsigned)stick.x.raw, stick.x.norm,
            (unsigned)stick.y.raw, stick.y.norm
        );
    };

    print_stick("Left Thumb",  js.left_thumb);
    print_stick("Right Thumb", js.right_thumb);
    print_stick("Left Index",  js.left_index);
    print_stick("Right Index", js.right_index);

    hal.console->printf("%-14s : %s\n", "Healthy", js.healthy ? "YES" : "NO");
    hal.console->printf("%-14s : %lu us\n", "Last Update", (unsigned long)js.last_update_us);

    hal.console->printf("========================\n");
}

/**
 * @brief Compute hat direction based on right index stick normalized inputs.
 * 
 * @param x_norm  Right index X axis, normalized [-1.0, +1.0]
 * @param y_norm  Right index Y axis, normalized [-1.0, +1.0]
 * @return uint8_t Hat value (0–7 = direction, 8 = neutral)
 */
uint8_t AirBoss_Joystick::compute_hat_direction(float x_norm, float y_norm)
{
    const float HIGH =  0.39f;  // equivalent to >2848
    const float LOW  = -0.39f;  // equivalent to <1248
    uint8_t hat = 8;            // neutral (null state)

    if (y_norm > HIGH)
    {
        if (x_norm > HIGH)
            hat = 1;  // Up-Right
        else if (x_norm < LOW)
            hat = 7;  // Up-Left
        else
            hat = 0;  // Up
    }
    else if (y_norm < LOW)
    {
        if (x_norm > HIGH)
            hat = 3;  // Down-Right
        else if (x_norm < LOW)
            hat = 5;  // Down-Left
        else
            hat = 4;  // Down
    }
    else if (x_norm > HIGH)
    {
        hat = 2;      // Right
    }
    else if (x_norm < LOW)
    {
        hat = 6;      // Left
    }

    return hat;
}