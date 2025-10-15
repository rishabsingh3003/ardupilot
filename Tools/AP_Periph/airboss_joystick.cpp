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

AirBoss_Joystick::AirBoss_Joystick() :
    dev(nullptr),
    _configured(false)
{
    memset(&_state, 0, sizeof(_state));
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


void AirBoss_Joystick::adc_timer()
{
    uint16_t raw_values[8];
    float    norm_values[8];

    // Clear (portable)
    for (uint8_t i = 0; i < 8; i++) { raw_values[i] = 0xFFFF; norm_values[i] = 0.0f; }

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

    // Normalize to [-1, +1] (we'll likely add per-axis calibration later)
    for (uint8_t i = 0; i < 8; i++) {
        norm_values[i] = (raw_values[i] / 4095.0f) * 2.0f - 1.0f;
    }

    // Map channels to sticks (keep your chosen wiring order)
    _state.left_thumb.x   = { raw_values[0], norm_values[0] };
    _state.left_thumb.y   = { raw_values[1], norm_values[1] };

    _state.right_thumb.x  = { raw_values[4], norm_values[4] };
    _state.right_thumb.y  = { raw_values[5], norm_values[5] };

    _state.left_index.x    = { raw_values[2], norm_values[2] };
    _state.left_index.y    = { raw_values[3], norm_values[3] };

    _state.right_index.x   = { raw_values[6], norm_values[6] };
    _state.right_index.y   = { raw_values[7], norm_values[7] };

    _state.healthy = true;
    _state.last_update_us = AP_HAL::micros();
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
