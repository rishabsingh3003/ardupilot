#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if AP_PERIPH_NETWORKING_ENABLED

#include <AP_Networking/AP_Networking.h>

#ifndef HAL_PERIPH_NETWORK_NUM_PASSTHRU
#define HAL_PERIPH_NETWORK_NUM_PASSTHRU 1
#endif

#ifndef AP_PERIPH_NET_PPP_PORT_DEFAULT
#define AP_PERIPH_NET_PPP_PORT_DEFAULT -1
#endif

#ifndef AP_PERIPH_NET_PPP_BAUD_DEFAULT
#define AP_PERIPH_NET_PPP_BAUD_DEFAULT 12500000
#endif

#ifndef HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS
#define HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS 0
#endif

#define SBUS_INPUT_CHANNELS	16

class Networking_Periph {
public:
    Networking_Periph() {
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    uint32_t get_valid_sbus_packets();
    uint32_t get_invalid_sbus_packets();
    uint32_t get_lost_frame_counter();

    bool get_latest_sbus_can_frame(uint16_t decoded_rc[SBUS_INPUT_CHANNELS], uint16_t decoded_rc_filtered[SBUS_INPUT_CHANNELS], uint16_t &num_values, bool &sbus_failsafe, int8_t &primary_id);

private:

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    class Passthru {
    public:
        friend class Networking_Periph;

        CLASS_NO_COPY(Passthru);

        Passthru() {
            AP_Param::setup_object_defaults(this, var_info);
        }

        void init();
        void update();

        static const struct AP_Param::GroupInfo var_info[];

        // Getters for outer class to read stats
        uint32_t get_valid_sbus_packets() const { return valid_sbus_packets; }
        uint32_t get_invalid_sbus_packets() const { return invalid_sbus_packets; }
        uint32_t get_total_sbus_bytes() const { return total_sbus_bytes; }
        uint32_t get_lost_frame_counter() const { return lost_frames; }
        bool get_latest_rc_can_frame(uint16_t decoded_rc[SBUS_INPUT_CHANNELS], uint16_t decoded_rc_filtered[SBUS_INPUT_CHANNELS], uint16_t &num_values, bool &failsafe
        , int8_t &primary_id) {
            if (new_sbus_can_frame) {
                memcpy(decoded_rc, decoded_rc_values, sizeof(decoded_rc_values));
                memcpy(decoded_rc_filtered, filtered_rc_values, sizeof(filtered_rc_values));
                num_values = rc_num_values;
                failsafe = _sbus_failsafe;
                new_sbus_can_frame = false;
                primary_id = rc_id;
                return true;
            }
            return false;
        }

    private:
        // configure a serial port for passthru
        void configure_serial_port(AP_HAL::UARTDriver *port, uint32_t baud, uint32_t options, int8_t parity, int8_t stop_bits);
        void process_sbus_buffer(const uint8_t *buf, uint32_t nbytes);
        uint8_t sbus_crc8(const uint8_t *data, size_t len);
        bool sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
                                     bool &sbus_failsafe, uint16_t max_values);
        void decode_11bit_channels(const uint8_t* data, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset);

        void rate_limit_sbus_channels(const uint16_t *input, uint16_t *output, uint32_t now_ms);

        struct Channels11Bit_8Chan {
        uint32_t ch0 : 11;
        uint32_t ch1 : 11;
        uint32_t ch2 : 11;
        uint32_t ch3 : 11;
        uint32_t ch4 : 11;
        uint32_t ch5 : 11;
        uint32_t ch6 : 11;
        uint32_t ch7 : 11;
    } PACKED;

        uint8_t latest_sbus_frame[25];  // Latest SBUS frame
        bool new_sbus_frame = false;  // Flag to indicate if a new SBUS frame is available
        bool new_sbus_can_frame = false;

        uint16_t decoded_rc_values[SBUS_INPUT_CHANNELS];
        uint16_t rc_num_values=0;
        bool _sbus_failsafe = false;

        uint8_t carry_buffer[50];  // Carry buffer for partial SBUS frames
        uint32_t carry_buffer_len = 0;

        uint32_t valid_sbus_packets = 0;
        uint32_t invalid_sbus_packets = 0;
        uint32_t total_sbus_bytes = 0;
        uint32_t last_sbus_timestamp = 0;
        uint32_t lost_frames = 0;  // Count frames lost due to timing

        AP_Int8 enabled;
        AP_Int8 ep1;
        AP_Int8 ep2;
        AP_Int32 baud1;
        AP_Int32 baud2;
        AP_Int32 options1;
        AP_Int32 options2;
#if HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS
        AP_Int8 parity1;
        AP_Int8 stop_bits1;
        AP_Int8 parity2;
        AP_Int8 stop_bits2;
#endif

        AP_HAL::UARTDriver *port1;
        AP_HAL::UARTDriver *port2;
        AP_Int8 check_crc;
        AP_Int8 rc_mode;
        AP_Float max_rate;
        AP_Int8 rc_id;

        // Persistent filter state
        uint16_t filtered_rc_values[16];
        uint16_t filter_rc_hist[4];
        uint32_t last_update_filter_time_ms[4];

    } passthru[HAL_PERIPH_NETWORK_NUM_PASSTHRU];
#endif // HAL_PERIPH_NETWORK_NUM_PASSTHRU

    AP_Networking networking_lib;
    bool got_addresses;

#if AP_NETWORKING_BACKEND_PPP
    AP_Int8 ppp_port;
    AP_Int32 ppp_baud;
#endif
    Passthru *selected_passthru_sbus = nullptr;

};

#endif // AP_PERIPH_NETWORKING_ENABLED
