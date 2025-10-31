/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  AP_Periph main firmware

  To flash this firmware on Linux use:

     st-flash write build/f103-periph/bin/AP_Periph.bin 0x8006000

 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <stdio.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <AP_HAL_ChibiOS/I2CDevice.h>
#endif

#ifndef HAL_PERIPH_HWESC_SERIAL_PORT
#define HAL_PERIPH_HWESC_SERIAL_PORT 3
#endif

// not only will the code not compile without features this enables,
// but it forms part of a series of measures to give a robust recovery
// mechanism on AP_Periph if a bad flash occurs.
#ifndef AP_CHECK_FIRMWARE_ENABLED
#error AP_CHECK_FIRMWARE_ENABLED must be enabled
#endif

extern const AP_HAL::HAL &hal;

AP_Periph_FW periph;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init() {}
void stm32_watchdog_pat() {}
#endif

void setup(void)
{
    periph.init();
}

void loop(void)
{
    periph.update();
}

static uint32_t start_ms;

AP_Periph_FW::AP_Periph_FW()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Periph_FW must be singleton");
    }
    _singleton = this;
}

#if HAL_LOGGING_ENABLED
const struct LogStructure AP_Periph_FW::log_structure[] = {
    LOG_COMMON_STRUCTURES,
};
#endif

void AP_Periph_FW::init()
{
    
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
#ifndef DISABLE_WATCHDOG
    stm32_watchdog_init();
#endif

    stm32_watchdog_pat();

#if !HAL_GCS_ENABLED
    hal.serial(0)->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 32);
#endif
    hal.serial(3)->begin(115200, 128, 256);

    load_parameters();

    stm32_watchdog_pat();

    can_start();

#if HAL_GCS_ENABLED
    stm32_watchdog_pat();
    gcs().init();
#endif
    serial_manager.init();

#if AP_PERIPH_NETWORKING_ENABLED
    networking_periph.init();
#endif

#if HAL_GCS_ENABLED
    gcs().setup_console();
    gcs().setup_uarts();
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Periph GCS Initialised!");
#endif

    stm32_watchdog_pat();

#ifdef HAL_BOARD_AP_PERIPH_ZUBAXGNSS
    // setup remapping register for ZubaxGNSS
    uint32_t mapr = AFIO->MAPR;
    mapr &= ~AFIO_MAPR_SWJ_CFG;
    mapr |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    AFIO->MAPR = mapr | AFIO_MAPR_CAN_REMAP_REMAP2 | AFIO_MAPR_SPI3_REMAP;
#endif

#if HAL_LOGGING_ENABLED
    logger.init(g.log_bitmask, log_structure, ARRAY_SIZE(log_structure));
#endif

    check_firmware_print();

    if (hal.util->was_watchdog_reset()) {
        printf("Reboot after watchdog reset\n");
    }

#if AP_STATS_ENABLED
    node_stats.init();
#endif

#if AP_PERIPH_SERIAL_OPTIONS_ENABLED
    serial_options.init();
#endif

#if AP_PERIPH_GPS_ENABLED
    gps.set_default_type_for_gps1(HAL_GPS1_TYPE_DEFAULT);
    if (gps.get_type(0) != AP_GPS::GPS_Type::GPS_TYPE_NONE && g.gps_port >= 0) {
        serial_manager.set_protocol_and_baud(g.gps_port, AP_SerialManager::SerialProtocol_GPS, AP_SERIALMANAGER_GPS_BAUD);
#if HAL_LOGGING_ENABLED
        #define MASK_LOG_GPS (1<<2)
        gps.set_log_gps_bit(MASK_LOG_GPS);
#endif
        gps.init();
    }
#endif  // AP_PERIPH_GPS_ENABLED

#if AP_DAC_ENABLED
    dac.init();
#endif

#if AP_PERIPH_MAG_ENABLED
    compass.init();
#endif

#if AP_PERIPH_BARO_ENABLED
    baro.init();
#endif

#if AP_PERIPH_IMU_ENABLED
    if (g.imu_sample_rate) {
        imu.init(g.imu_sample_rate);
        if (imu.get_accel_count() > 0 || imu.get_gyro_count() > 0) {
            hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Periph_FW::can_imu_update, void), "IMU_UPDATE", 16384, AP_HAL::Scheduler::PRIORITY_CAN, 0);
        }
    }
#endif

#if AP_PERIPH_BATTERY_ENABLED
    battery_lib.init();
#endif

#if AP_PERIPH_RCIN_ENABLED
    rcin_init();
#endif

#if defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) || AP_PERIPH_RC_OUT_ENABLED
    hal.rcout->init();
#endif

#ifdef HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY
    hal.rcout->set_serial_led_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY, AP_HAL::RCOutput::MODE_NEOPIXEL);
#endif

#if AP_PERIPH_RC_OUT_ENABLED
    rcout_init();
#endif

#if AP_PERIPH_ADSB_ENABLED
    adsb_init();
#endif

#if AP_PERIPH_EFI_ENABLED
    if (efi.enabled() && g.efi_port >= 0) {
        auto *uart = hal.serial(g.efi_port);
        if (uart != nullptr) {
            uart->begin(g.efi_baudrate);
            serial_manager.set_protocol_and_baud(g.efi_port, AP_SerialManager::SerialProtocol_EFI, g.efi_baudrate);
            efi.init();
        }
    }
#endif

#if AP_KDECAN_ENABLED
    kdecan.init();
#endif

#if AP_PERIPH_AIRSPEED_ENABLED
#if (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS) && (HAL_USE_I2C == TRUE)
    const bool pins_enabled = ChibiOS::I2CBus::check_select_pins(0x01);
    if (pins_enabled) {
        ChibiOS::I2CBus::set_bus_to_floating(0);
#ifdef HAL_GPIO_PIN_LED_CAN_I2C
        palWriteLine(HAL_GPIO_PIN_LED_CAN_I2C, 1);
#endif
    } else {
        // Note: logging of ARSPD is not enabled currently. To enable, call airspeed.set_log_bit(); here
        airspeed.init();
    }
#else
    // Note: logging of ARSPD is not enabled currently. To enable, call airspeed.set_log_bit(); here
    airspeed.init();
#endif

#endif

#if AP_PERIPH_RANGEFINDER_ENABLED
    bool have_rangefinder = false;
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        if ((rangefinder.get_type(i) != RangeFinder::Type::NONE) && (g.rangefinder_port[i] >= 0)) {
            // init uart for serial rangefinders
            auto *uart = hal.serial(g.rangefinder_port[i]);
            if (uart != nullptr) {
                uart->begin(g.rangefinder_baud[i]);
                serial_manager.set_protocol_and_baud(g.rangefinder_port[i], AP_SerialManager::SerialProtocol_Rangefinder, g.rangefinder_baud[i]);
                have_rangefinder = true;
            }
        }
    }
    if (have_rangefinder) {
        // Can only call rangefinder init once, subsequent inits are blocked
        rangefinder.init(ROTATION_NONE);
    }
#endif

#if AP_PERIPH_PROXIMITY_ENABLED
    if (proximity.get_type(0) != AP_Proximity::Type::None && g.proximity_port >= 0) {
        auto *uart = hal.serial(g.proximity_port);
        if (uart != nullptr) {
            uart->begin(g.proximity_baud);
            serial_manager.set_protocol_and_baud(g.proximity_port, AP_SerialManager::SerialProtocol_Lidar360, g.proximity_baud);
            proximity.init();
        }
    }
#endif

#if AP_PERIPH_PWM_HARDPOINT_ENABLED
    pwm_hardpoint_init();
#endif

#if AP_PERIPH_HOBBYWING_ESC_ENABLED
    hwesc_telem.init(hal.serial(HAL_PERIPH_HWESC_SERIAL_PORT));
#endif

#if AP_PERIPH_ESC_APD_ENABLED
    for (uint8_t i = 0; i < ESC_NUMBERS; i++) {
        const uint8_t port = g.esc_serial_port[i];
        if (port < SERIALMANAGER_NUM_PORTS) { // skip bad ports
            apd_esc_telem[i] = NEW_NOTHROW ESC_APD_Telem (hal.serial(port), g.pole_count[i]);
        }
    }
#endif

#if AP_PERIPH_MSP_ENABLED
    if (g.msp_port >= 0) {
        msp_init(hal.serial(g.msp_port));
    }
#endif
    
#if AP_TEMPERATURE_SENSOR_ENABLED
    temperature_sensor.init();
#endif

#if HAL_NMEA_OUTPUT_ENABLED
    nmea.init();
#endif

#if AP_PERIPH_RPM_ENABLED
    rpm_sensor.init();
#endif

#if AP_PERIPH_NOTIFY_ENABLED
    notify.init();
#endif

#if AP_PERIPH_RELAY_ENABLED
    relay.init();
#endif

#if AP_SCRIPTING_ENABLED
    scripting.init();
#endif

    airboss_joystick.init();

    airboss_switches.init();

    airboss_networking.init();
    
    start_ms = AP_HAL::millis();
}

#if (defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) && HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY == 8) || AP_PERIPH_NOTIFY_ENABLED
/*
  rotating rainbow pattern on startup
 */
void AP_Periph_FW::update_rainbow()
{
#if AP_PERIPH_NOTIFY_ENABLED
    if (notify.get_led_len() != 8) {
        return;
    }
#endif
    static bool rainbow_done;
    if (rainbow_done) {
        return;
    }
    uint32_t now = AP_HAL::millis();
    if (now - start_ms > 1500) {
        rainbow_done = true;
#if AP_PERIPH_NOTIFY_ENABLED
        periph.notify.handle_rgb(0, 0, 0);
#elif defined(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY)
        hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, -1, 0, 0, 0);
        hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY);
#endif
        return;
    }
    static uint32_t last_update_ms;
    const uint8_t step_ms = 30;
    if (now - last_update_ms < step_ms) {
        return;
    }
    const struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } rgb_rainbow[] = {
        { 255, 0, 0 },
        { 255, 127, 0 },
        { 255, 255, 0 },
        { 0,   255, 0 },
        { 0,   0,   255 },
        { 75,  0,   130 },
        { 143, 0,   255 },
        { 0,   0,   0 },
    };
    last_update_ms = now;
    static uint8_t step;
    const uint8_t nsteps = ARRAY_SIZE(rgb_rainbow);
    float brightness = 0.3;
    for (uint8_t n=0; n<8; n++) {
        uint8_t i = (step + n) % nsteps;
#if AP_PERIPH_NOTIFY_ENABLED
        periph.notify.handle_rgb(
#elif defined(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY)
        hal.rcout->set_serial_led_rgb_data(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, n,
#endif
                                        rgb_rainbow[i].red*brightness,
                                        rgb_rainbow[i].green*brightness,
                                        rgb_rainbow[i].blue*brightness);
    }
    step++;

#if defined(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY)
    hal.rcout->serial_led_send(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY);
#endif
}
#endif // AP_PERIPH_NOTIFY_ENABLED


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && CH_DBG_ENABLE_STACK_CHECK == TRUE
void AP_Periph_FW::show_stack_free()
{
    const uint32_t isr_stack_size = uint32_t((const uint8_t *)&__main_stack_end__ - (const uint8_t *)&__main_stack_base__);
    can_printf("ISR %u/%u", unsigned(stack_free(&__main_stack_base__)), unsigned(isr_stack_size));

    for (thread_t *tp = chRegFirstThread(); tp; tp = chRegNextThread(tp)) {
        uint32_t total_stack;
        if (tp->wabase == (void*)&__main_thread_stack_base__) {
            // main thread has its stack separated from the thread context
            total_stack = uint32_t((const uint8_t *)&__main_thread_stack_end__ - (const uint8_t *)&__main_thread_stack_base__);
        } else {
            // all other threads have their thread context pointer
            // above the stack top
            total_stack = uint32_t(tp) - uint32_t(tp->wabase);
        }
        can_printf("%s STACK=%u/%u\n", tp->name, unsigned(stack_free(tp->wabase)), unsigned(total_stack));
    }
}
#endif



void AP_Periph_FW::update()
{
#if AP_STATS_ENABLED
    node_stats.update();
#endif

    static uint32_t last_led_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_led_ms > 1000) {
        last_led_ms = now;
#ifdef HAL_GPIO_PIN_LED
        if (!no_iface_finished_dna) {
            palToggleLine(HAL_GPIO_PIN_LED);
        }
#endif
#if 0
#if AP_PERIPH_GPS_ENABLED
        hal.serial(0)->printf("GPS status: %u\n", (unsigned)gps.status());
#endif
#if AP_PERIPH_MAG_ENABLED
        const Vector3f &field = compass.get_field();
        hal.serial(0)->printf("MAG (%d,%d,%d)\n", int(field.x), int(field.y), int(field.z));
#endif
#if AP_PERIPH_BARO_ENABLED
        hal.serial(0)->printf("BARO H=%u P=%.2f T=%.2f\n", baro.healthy(), baro.get_pressure(), baro.get_temperature());
#endif
#if AP_PERIPH_RANGEFINDER_ENABLED
        hal.serial(0)->printf("Num RNG sens %u\n", rangefinder.num_sensors());
        for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
            AP_RangeFinder_Backend *backend = rangefinder.get_backend(i);
            if (backend == nullptr) {
                continue;
            }
            hal.serial(0)->printf("RNG %u %ucm\n", i, uint16_t(backend->distance()*100));
        }
#endif
        hal.scheduler->delay(1);
#endif
#ifdef HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY
        hal.rcout->set_serial_led_num_LEDs(HAL_PERIPH_NEOPIXEL_CHAN_WITHOUT_NOTIFY, HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY, AP_HAL::RCOutput::MODE_NEOPIXEL);
#endif

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
        check_for_serial_reboot_cmd(HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT);
#endif

#if AP_PERIPH_RC_OUT_ENABLED
        rcout_init_1Hz();
#endif

#if AP_DAC_ENABLED
        dac.update();
#endif

        GCS_SEND_MESSAGE(MSG_HEARTBEAT);
        GCS_SEND_MESSAGE(MSG_SYS_STATUS);
    }

    static uint32_t last_error_ms;
    const auto &ierr = AP::internalerror();
    if (now - last_error_ms > 5000 && ierr.errors()) {
        // display internal errors as DEBUG every 5s
        last_error_ms = now;
        can_printf("IERR 0x%x %u", unsigned(ierr.errors()), unsigned(ierr.last_error_line()));
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && CH_DBG_ENABLE_STACK_CHECK == TRUE
    static uint32_t last_debug_ms;
    if (debug_option_is_set(DebugOptions::SHOW_STACK) && now - last_debug_ms > 5000) {
        last_debug_ms = now;
        show_stack_free();
    }
#endif

    if (debug_option_is_set(DebugOptions::AUTOREBOOT) && AP_HAL::millis() > 15000) {
        // attempt reboot with HOLD after 15s
        periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        set_fast_reboot((rtc_boot_magic)(RTC_BOOT_HOLD));
        NVIC_SystemReset();
#endif
    }

#if AP_PERIPH_BATTERY_ENABLED
    if (now - battery.last_read_ms >= 100) {
        // update battery at 10Hz
        battery.last_read_ms = now;
        battery_lib.read();
    }
#endif

#if AP_PERIPH_RCIN_ENABLED
    rcin_update();
#endif

#if AP_PERIPH_BATTERY_BALANCE_ENABLED
    batt_balance_update();
#endif
    
    static uint32_t fiftyhz_last_update_ms;
    if (now - fiftyhz_last_update_ms >= 20) {
        // update at 50Hz
        fiftyhz_last_update_ms = now;
#if AP_PERIPH_NOTIFY_ENABLED
        notify.update();
#endif
#if HAL_GCS_ENABLED
        gcs().update_receive();
        gcs().update_send();
#endif
    }

#if HAL_NMEA_OUTPUT_ENABLED
    nmea.update();
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
    temperature_sensor.update();
#endif

#if AP_PERIPH_RPM_ENABLED
    if (now - rpm_last_update_ms >= 100) {
        rpm_last_update_ms = now;
        rpm_sensor.update();
    }
#endif

#if HAL_LOGGING_ENABLED
    logger.periodic_tasks();
#endif

    can_update();

#if AP_PERIPH_NETWORKING_ENABLED
    networking_periph.update();
#endif

#if (defined(HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY) && HAL_PERIPH_NEOPIXEL_COUNT_WITHOUT_NOTIFY == 8) || AP_PERIPH_NOTIFY_ENABLED
    update_rainbow();
#endif
#if AP_PERIPH_ADSB_ENABLED
    adsb_update();
#endif
#if AP_PERIPH_BATTERY_TAG_ENABLED
    battery_tag.update();
#endif

    // airboss_joystick_update();
    static uint32_t airboss_joystick_last_ms;
    if (now - airboss_joystick_last_ms > 10) {
        // run at 100Hz
        airboss_joystick_last_ms = now;
        airboss_joystick.update();
        airboss_switches.update();
    }
    const AirBoss_Joystick::JoystickState js_state = airboss_joystick.get_state();
    
    static uint32_t last_status_ms;
    if (now - last_status_ms > 20) {
        last_status_ms = now;
        airboss_networking.send_airboss_state(js_state);
    //     bool temp_switches[NUM_SWITCHES];
    //     for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
    //         // airboss_networking.send_airboss_switch_state(i, airboss_switches.get_state(i));
    //         temp_switches[i] = airboss_switches.get_state(i);
    //     }
    //     hal.console->printf("AirBoss Switches: %u%u%u%u%u%u%u%u%u%u%u%u\n",
    //         (unsigned)temp_switches[0], (unsigned)temp_switches[1], (unsigned)temp_switches[2], (unsigned)temp_switches[3],
    //         (unsigned)temp_switches[4], (unsigned)temp_switches[5], (unsigned)temp_switches[6], (unsigned)temp_switches[7],
    //         (unsigned)temp_switches[8], (unsigned)temp_switches[9], (unsigned)temp_switches[10], (unsigned)temp_switches[11]);
    }

    static uint32_t rc_mav_last_ms;
    if (now - rc_mav_last_ms > 250) {
        rc_mav_last_ms = now;
        // send RC_CHANNELS message at 1Hz
        // airboss_utils.send_rc_channels_mavlink(js_state, airboss_switches);
        auto *uart = hal.serial(0);
        if (uart != nullptr) {
            uart->usb_hid_send_joystick(65535, js_state.right_thumb.x.norm*127, js_state.right_thumb.y.norm*127);
        }
        airboss_switches.print_states();
        // airboss_joystick.print_states();
    }

    static uint32_t sbus_last_ms;
    if (now - sbus_last_ms > 20) {
        sbus_last_ms = now;
        uint8_t sbus_frame[25];
        size_t len = pack_sbus_from_joystick_and_switches(
            airboss_joystick.get_state(),
            airboss_switches,
            sbus_frame);

        airboss_networking.send_sbus_packet(sbus_frame, len);
    }
}

size_t AP_Periph_FW::pack_sbus_from_joystick_and_switches(
    const AirBoss_Joystick::JoystickState& js,
    const AirBoss_Switches& switches,
    uint8_t* sbus_out)
{
    if (sbus_out == nullptr) {
        return 0;
    }

    uint16_t ch[16] = {992};
    // ───────────────────────────────────────────────
    // Joystick → CH1-CH8

    // ───────────────────────────────────────────────
    ch[0] = airboss_joystick.axis_to_sbus(js.right_thumb.x.norm);   // roll
    ch[1] = airboss_joystick.axis_to_sbus(js.right_thumb.y.norm);   // pitch
    ch[2] = airboss_joystick.axis_to_sbus(js.left_thumb.y.norm);  // throttle
    ch[3] = airboss_joystick.axis_to_sbus(js.left_thumb.x.norm);  // yaw

    // ───────────────────────────────────────────────
    // Buttons / Switches
    // Each logical Function is mapped to one channel.
    // Three-way switches use 172/992/1811.
    // Two-way buttons use 172/1811.
    // ───────────────────────────────────────────────
    ch[4] = switches.function_to_sbus(AirBoss_Switches::Function::MODE_SELECT);
    ch[6] = switches.function_to_sbus(AirBoss_Switches::Function::KILL_SWITCH);
    ch[7] = switches.function_to_sbus(AirBoss_Switches::Function::EMERGENCY_KILL);
    ch[11] = switches.function_to_sbus(AirBoss_Switches::Function::LIGHTS);

    // ───────────────────────────────────────────────
    // Optional: mark joystick health in last channel
    // ───────────────────────────────────────────────
    ch[15] = js.healthy ? 1811 : 172;

    // ───────────────────────────────────────────────
    // Pack into SBUS byte frame
    // ───────────────────────────────────────────────
    uint8_t SBUS_FRAME_SIZE = 25;
    memset(sbus_out, 0, SBUS_FRAME_SIZE);
    sbus_out[0] = 0x0F; // SBUS header

    uint32_t bit_ofs = 0;
    for (uint8_t ch_i = 0; ch_i < 16; ch_i++) {
        uint16_t val = ch[ch_i];
        if (val > 2047) val = 2047;
        uint16_t byte_idx = 1 + (bit_ofs / 8);
        uint8_t bit_idx = bit_ofs % 8;
        uint32_t word = static_cast<uint32_t>(val) << bit_idx;

        sbus_out[byte_idx + 0] |= (word & 0xFF);
        sbus_out[byte_idx + 1] |= ((word >> 8) & 0xFF);
        sbus_out[byte_idx + 2] |= ((word >> 16) & 0xFF);
        bit_ofs += 11;
    }

    sbus_out[23] = 0x00; // Flags (frame lost/failsafe bits)
    sbus_out[24] = 0x00; // End byte
    return SBUS_FRAME_SIZE;
}

// void AP_Periph_FW::rc_send_mavlink()
// {
//     auto *uart = hal.serial(0);   // usually USB == 0
//     if (uart == nullptr) {
//         return;
//     }

//     const AirBoss_Joystick::JoystickState js_state = airboss_joystick.get_state();
//     mavlink_rc_channels_t rc{};
//     rc.time_boot_ms = AP_HAL::millis();
//     rc.chancount    = 4;   // number of channels you want to send
//     rc.rssi         = 255; // optional: 0–254 valid, 255 = unknown


//     rc.chan1_raw = linear_interpolate(1000, 2000, js_state.right_thumb.x.norm, -1, 1);
//     rc.chan2_raw = linear_interpolate(1000, 2000, js_state.right_thumb.y.norm, -1, 1);
//     rc.chan3_raw = linear_interpolate(1000, 2000, js_state.left_thumb.y.norm, -1, 1);
//     rc.chan4_raw = linear_interpolate(1000, 2000, js_state.left_thumb.x.norm, -1, 1);

//     // fill in channel values (example: midpoint)
//     rc.chan5_raw = 1500;
//     rc.chan6_raw = 1500;
//     rc.chan7_raw = 1500;
//     rc.chan8_raw = 1500;
//     rc.chan9_raw = 0;
//     rc.chan10_raw = 0;
//     rc.chan11_raw = 0;
//     rc.chan12_raw = 0;
//     rc.chan13_raw = 0;
//     rc.chan14_raw = 0;
//     rc.chan15_raw = 0;
//     rc.chan16_raw = 0;
//     rc.chan17_raw = 0;
//     rc.chan18_raw = 0;

//     mavlink_message_t msg;
//     // encode using your generated header
//     uint16_t len = mavlink_msg_rc_channels_encode_status(
//         1,                        // system_id
//         1,   // component_id
//         &rc_channels.status, &msg, &rc);

//     // send raw bytes over USB serial
//     uart->write((uint8_t*)&msg.magic, len);
// }

void AP_Periph_FW::airboss_joystick_update()
{
    AirBoss_Joystick::JoystickState state = airboss_joystick.get_state();
    // gcs print joystick vals every 1 second
    static uint32_t last_print_ms1;
    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms1 > 10) {
        last_print_ms1 = now;
#if HAL_GCS_ENABLED
gcs().send_text(MAV_SEVERITY_INFO,
    "AirBoss Joystick: LF(%u,%u) RF(%u,%u) LR(%u,%u) RR(%u,%u)",
    (unsigned)state.left_thumb.x.raw, (unsigned)state.left_thumb.y.raw,
    (unsigned)state.right_thumb.x.raw, (unsigned)state.right_thumb.y.raw,
    (unsigned)state.left_index.x.raw, (unsigned)state.left_index.y.raw,
    (unsigned)state.right_index.x.raw, (unsigned)state.right_index.y.raw);
        gcs().send_text(MAV_SEVERITY_INFO, "AirBoss health =%u last update=%.3f sec ago", state.healthy ? 1 : 0, (double)(AP_HAL::micros() - state.last_update_us)/1e6);
#endif
    }
}

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
// check for uploader.py reboot command
void AP_Periph_FW::check_for_serial_reboot_cmd(const int8_t serial_index)
{
    // These are the string definitions in uploader.py
    //            NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
    //            NSH_REBOOT_BL   = b"reboot -b\n"
    //            NSH_REBOOT      = b"reboot\n"

    // This is the command sequence that is sent from uploader.py
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT_BL)
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT)

    for (uint8_t i=0; i<hal.num_serial; i++) {
        if (serial_index >= 0 && serial_index != i) {
            // a specific serial port was selected but this is not it
            continue;
        }

        auto *uart = hal.serial(i);
        if (uart == nullptr || !uart->is_initialized()) {
            continue;
        }

        uint32_t available = MIN(uart->available(), 1000U);
        while (available-- > 0) {
            const char reboot_string[] = "\r\r\rreboot -b\n\r\r\rreboot\n";
            const char reboot_string_len = sizeof(reboot_string)-1; // -1 is to remove the null termination
            static uint16_t index[hal.num_serial];

            uint8_t data;
            if (!uart->read(data)) {
                // read error
                continue;
            }
            if (index[i] >= reboot_string_len || (uint8_t)data != reboot_string[index[i]]) {
                // don't have a perfect match, start over
                index[i] = 0;
                continue;
            }
            index[i]++;
            if (index[i] == reboot_string_len) {
                // received reboot msg. Trigger a reboot and stay in the bootloader
                prepare_reboot();
                hal.scheduler->reboot(true);
            }
        }
    }
}
#endif // HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT

// prepare for a safe reboot where PWMs and params are gracefully disabled
// This is copied from AP_Vehicle::reboot(bool hold_in_bootloader) minus the actual reboot
void AP_Periph_FW::prepare_reboot()
{
#if AP_PERIPH_RC_OUT_ENABLED
        // force safety on
        hal.rcout->force_safety_on();
#endif

        // flush pending parameter writes
        AP_Param::flush();

        // do not process incoming mavlink messages while we delay:
        hal.scheduler->register_delay_callback(nullptr, 5);

        // delay to give the ACK a chance to get out, the LEDs to flash,
        // the IO board safety to be forced on, the parameters to flush,
        hal.scheduler->expect_delay_ms(100);
        hal.scheduler->delay(40);
        hal.scheduler->expect_delay_ms(0);
}

/*
  reboot, optionally holding in bootloader. For scripting
 */
void AP_Periph_FW::reboot(bool hold_in_bootloader)
{
    prepare_reboot();
    hal.scheduler->reboot(hold_in_bootloader);
}

AP_Periph_FW *AP_Periph_FW::_singleton;

AP_Periph_FW& AP::periph()
{
    return *AP_Periph_FW::get_singleton();
}

AP_HAL_MAIN();
