#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED

#include "AP_RangeFinder_LightWare_GRF.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>

#define GRF_UPDATE_RATE 50 // Hz
#define GRF_STREAM_CM_DISTANCES 5

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RangeFinder_LightWareGRF::var_info[] = {
    // @Param: GRF_RET
    // @DisplayName: LightWare GRF Distance Return Type
    // @Description: Selects which single return to use.
    // @Values: 0:FirstRaw,1:FirstFiltered,2:LastRaw,3:LastFiltered
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GRF_RET", 12, AP_RangeFinder_LightWareGRF, return_selection, (uint8_t)GRF_ReturnSelection::FIRST_RAW),

    // @Param: GRF_ST
    // @DisplayName: LightWare GRF Minimum Return Strength
    // @Description: Minimum acceptable return signal strength in dB. Returns weaker than this will be ignored. Set to 0 to disable filtering.
    // @Range: 0 255
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GRF_ST", 13, AP_RangeFinder_LightWareGRF, minimum_return_strength, 0),

    // @Param: GRF_RATE
    // @DisplayName: LightWare GRF Update Rate
    // @Description: The update rate of the sensor in Hz. Must match the
    // @Range: 1 50
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GRF_RATE", 14, AP_RangeFinder_LightWareGRF, update_rate, GRF_UPDATE_RATE),

    AP_GROUPEND
};

AP_RangeFinder_LightWareGRF::AP_RangeFinder_LightWareGRF(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params)
    : AP_RangeFinder_Backend_Serial(_state, _params),
    AP_LightWareSerial(AP_RangeFinder_Backend_Serial::uart)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

// Attempts to parse a single streaming measurement
bool AP_RangeFinder_LightWareGRF::try_parse_stream_packet(float &reading_m)
{
    MessageID cmd_id;
    uint8_t payload[LIGHTWARE_PAYLOAD_LEN_MAX];
    uint16_t payload_len = 0;

    float sum_m = 0.0f;
    uint16_t count = 0;

    // Loop through all packets in the buffer
    if (read_and_parse_response(cmd_id, payload, payload_len)) {
        // Check for correct message type and length
        if (cmd_id != MessageID::DISTANCE_DATA_CM || payload_len < 8) {
            return false;
        }

        // Extract distance and strength
        uint32_t distance_cm = UINT32_VALUE(payload[3], payload[2], payload[1], payload[0]) * 10;
        uint32_t strength_db = UINT32_VALUE(payload[7], payload[6], payload[5], payload[4]);

        if (minimum_return_strength == 0 || (int8_t)strength_db >= minimum_return_strength) {
            float dist_m = distance_cm * 0.01f;
            sum_m += dist_m;
            count++;
        }
    }

    if (count > 0) {
        reading_m = sum_m / count;
        return true;
    }

    return false;
}

// Checks if PRODUCT_NAME payload matches expected GRF signature
bool AP_RangeFinder_LightWareGRF::matches_product_name(const uint8_t *buf, uint16_t len)
{
    // Must be at least "GRFXXX\0" = 7 bytes
    if (len < 7) {
        return false;
    }

    const char expected[] = "GRF";
    for (uint8_t i = 0; i < 3; i++) { // Only compare "GRF" for compatibility with GRF250, GRF500
        if (buf[i] != expected[i]) {
            return false;
        }
    }
    return buf[6] == '\0'; // Check null after that
}

// Parses config responses and advances setup step
void AP_RangeFinder_LightWareGRF::check_config()
{
    MessageID resp_cmd_id;
    uint8_t response_buf[LIGHTWARE_PAYLOAD_LEN_MAX];
    uint16_t response_len = 0;

    if (read_and_parse_response(resp_cmd_id, response_buf, response_len)) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "GRF: received cmd %d (expecting %d)", (uint8_t)resp_cmd_id, (uint8_t)grf.config_step);

        bool valid = false;

        switch (grf.config_step) {
        case ConfigStep::HANDSHAKE:
            valid = (resp_cmd_id == MessageID::PRODUCT_NAME &&
                     matches_product_name(response_buf, response_len));
            if (valid) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LightWare %s detected", (const char*)response_buf);
            }
            break;

        case ConfigStep::UPDATE_RATE:
            if (resp_cmd_id == MessageID::UPDATE_RATE && response_len >= 4) {
                const uint8_t response_update_rate = (uint8_t)UINT32_VALUE(response_buf[3], response_buf[2], response_buf[1], response_buf[0]);
                valid = (response_update_rate == update_rate);
            }
            break;

        case ConfigStep::DISTANCE_OUTPUT:
            valid = (resp_cmd_id == MessageID::DISTANCE_OUTPUT);
            break;

        case ConfigStep::STREAM:
            valid = (resp_cmd_id == MessageID::STREAM);
            break;

        case ConfigStep::DONE:
            break;
        }

        if (valid) {
            grf.config_step = static_cast<ConfigStep>(static_cast<uint8_t>(grf.config_step) + 1);
            return;
        }
    }
}

// Configure the rangefinder
void AP_RangeFinder_LightWareGRF::configure_rangefinder()
{
    if (grf.config_step == ConfigStep::DONE) {
        return;
    }

    check_config();

    const uint32_t now = AP_HAL::millis();
    if (now - grf.last_init_ms < 100) {
        return;
    }

    switch (grf.config_step) {
    case ConfigStep::HANDSHAKE:
        uart->write((uint8_t*)"UUU", 3); // Try to switch GRF sensor to serial mode
        send_message((uint8_t)MessageID::PRODUCT_NAME, false, nullptr, 0);
        break;

    case ConfigStep::UPDATE_RATE: {
        const uint8_t payload[4] = {(uint8_t)update_rate, 0, 0, 0};
        send_message((uint8_t)MessageID::UPDATE_RATE, true, payload, 4);
        break;
    }

    case ConfigStep::DISTANCE_OUTPUT: {
        uint8_t data_bit = 1, strength_bit = 2;
        switch (GRF_ReturnSelection(return_selection.get())) {
            case GRF_ReturnSelection::FIRST_RAW:
                data_bit = 0;
                break;
            case GRF_ReturnSelection::LAST_RAW:
                data_bit = 3;
                strength_bit = 5;
                break;
            case GRF_ReturnSelection::LAST_FILTERED:
                data_bit = 4;
                strength_bit = 5;
                break;
            case GRF_ReturnSelection::FIRST_FILTERED:
                break;
        }
        const uint8_t payload[4] = {
            static_cast<uint8_t>((1U << data_bit) | (1U << strength_bit)), 0, 0, 0
        };
        send_message((uint8_t)MessageID::DISTANCE_OUTPUT, true, payload, 4);
        break;
    }

    case ConfigStep::STREAM: {
        const uint8_t payload[4] = {GRF_STREAM_CM_DISTANCES, 0, 0, 0};
        send_message((uint8_t)MessageID::STREAM, true, payload, 4);
        break;
    }

    case ConfigStep::DONE:
        break;
    }

    grf.last_init_ms = now;
}

// Reads UART and tries to parse a complete response
bool AP_RangeFinder_LightWareGRF::read_and_parse_response(MessageID& cmd_id_out,
                                                          uint8_t* payload_out,
                                                          uint16_t& payload_len_out)
{
    if (uart == nullptr) {
        return false;
    }

    // process up to 1K of characters per iteration
    uint32_t nbytes = MIN(uart->available(), 1024U);
    while (nbytes-- > 0) {
        uint8_t c;
        if (!uart->read(c)) {
            continue;
        }
        if (parse_byte(c)) {
            cmd_id_out = static_cast<MessageID>(_msg.msgid);
            payload_len_out = _msg.payload_len;
            if (payload_len_out > 0) {
                memcpy(payload_out, &_msg.payload, payload_len_out);
            }
            return true;
        }
    }

    return false;
}

// Called periodically to fetch a new range reading
bool AP_RangeFinder_LightWareGRF::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    if (grf.config_step != ConfigStep::DONE) {
        configure_rangefinder();
        return false;
    }

    return try_parse_stream_packet(reading_m);
}

#endif // AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED
