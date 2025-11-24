/*
  Class to handle AirBoss_Networking Joystick via SPI based ADC
 */


#include "airboss_networking.h"
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <utility>
#include <cstdio>
#include <cstring>
#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL& hal;

bool UDPChannel::open_client(const char* ip_str, uint16_t port)
{
    close();

    auto s = NEW_NOTHROW SocketAPM(true);
    if (s == nullptr) {
        return false;
    }

    if (!s->connect(ip_str, port)) {
        delete s;
        return false;
    }

    s->set_blocking(false);
    sock = s;
    connected = true;
    return true;
}

bool UDPChannel::open_server(uint16_t port)
{
    close();

    auto s = NEW_NOTHROW SocketAPM(true);
    if (s == nullptr) {
        return false;
    }

    if (!s->bind("0.0.0.0", port)) {
        delete s;
        return false;
    }

    s->set_blocking(false);
    sock = s;
    connected = true;
    return true;
}

void UDPChannel::close()
{
    if (sock != nullptr) {
        delete sock;
        sock = nullptr;
    }
    connected = false;
}

size_t UDPChannel::write(const uint8_t* buf, size_t len, uint8_t* error)
{
    if (sock == nullptr) {
        if (error) *error = EINVAL;
        return 0;
    }

    const ssize_t ret = sock->send(buf, len);
    if (ret <= 0) {
        if (error) *error = errno;
        connected = false;
        return 0;
    }

    tx_bytes_total += ret;
    if (error) *error = 0;
    return ret;
}

size_t UDPChannel::read(uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    if (sock == nullptr) {
        if (error) *error = EINVAL;
        return 0;
    }

    const ssize_t ret = sock->recv(buf, len, timeout_ms);
    if (ret <= 0) {
        if (error) *error = errno;
        return 0;
    }

    rx_bytes_total += ret;
    if (error) *error = 0;
    return ret;
}


AirBoss_Networking::AirBoss_Networking() {}

bool AirBoss_Networking::init()
{
//     // RC input (joystick -> UAV)
//     if (!rc_in.open_server(5600)) {
//         hal.console->printf("RC_IN open failed\n");
//         return false;
//     }

    _cmd_queue = new ObjectArray<CommandQueueItem>(30);
    if (_cmd_queue == nullptr) {
        hal.console->printf("cmd_queue alloc failed\n");
        return false;
    }

    // RC output (UAV -> joystick/GCS)
    if (!telem_out.open_client("192.168.17.9", 14559)) {
        hal.console->printf("telem_out open failed\n");
        return false;
    }

    // Telemetry output (UAV -> dashboard)
    if (!rc_out.open_client("192.168.17.202", 14551)) {
        hal.console->printf("rc out failed\n");
        return false;
    }

    // // Debug output (optional)
    // debug_out.open_client("127.0.0.1", 6000);

    // Command input (Flutter → AirBoss)
    if (!cmd_in.open_server(14560)) {
        hal.console->printf("cmd_in open failed\n");
    } else {
        hal.console->printf("cmd_in listening OK on port 14560\n");
    }

    hal.console->printf("AirBoss_Networking init OK\n");
    return true;
}

void AirBoss_Networking::loop_50hz()
{
    // send telemetry JSON at 20 Hz – 50 Hz
    // send_airboss_state();
    read_command_packet();
    process_next_command();
}

void AirBoss_Networking::read_command_packet()
{
    uint8_t buf[64];
    uint8_t err = 0;

    const size_t n = cmd_in.read(buf, sizeof(buf), 0, &err);
    if (n == 0) {
        return; // nothing received
    }

    if (n != sizeof(CommandPacket)) {
        hal.console->printf("Bad command size: %u\n", unsigned(n));
        return;
    }

    CommandPacket pkt;
    memcpy(&pkt, buf, sizeof(pkt));

    if (pkt.magic != 0xDDCCBBAA) {
        hal.console->printf("Bad command magic: %f\n", (double)pkt.magic);
        return;
    }

    uint16_t expected_crc = compute_crc16((uint8_t*)&pkt,
                                          sizeof(pkt) - sizeof(pkt.crc));

    if (expected_crc != pkt.crc) {
        hal.console->printf("Bad CRC: got %u expected %u\n", pkt.crc, expected_crc);
        return;
    }

    // Push to queue
    CommandQueueItem item {};
    item.cmd = pkt;

    if (_cmd_queue == nullptr) {
        hal.console->printf("cmd_queue is null!\n");
        return;
    }

    _cmd_queue->push(item);

    hal.console->printf("Queued command: %s = %.3f\n",
                        pkt.cmd_name, pkt.value);
}

uint16_t AirBoss_Networking::compute_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ ((crc ^ data[i]) & 0xFFu);
    }
    return crc;
}

bool AirBoss_Networking::process_next_command()
{
    if (_cmd_queue == nullptr) {
        return false;
    }

    // Pop first item
    CommandQueueItem item;
    if (!_cmd_queue->pop(item)) {
        return false; // queue empty
    }

    const CommandPacket &cmd = item.cmd;

    // Validate magic
    if (cmd.magic != 0xDDCCBBAA) {
        hal.console->printf("CMD: bad magic");
        return false;
    }

    // Ensure name is null-terminated
    char name_buf[17] = {};
    memcpy(name_buf, cmd.cmd_name, 16);
    name_buf[16] = '\0';

    // Trim trailing garbage
    for (int i = 15; i >= 0; i--) {
        if (name_buf[i] == '\0') break;
        if ((uint8_t)name_buf[i] < 32 || (uint8_t)name_buf[i] > 126) {
            name_buf[i] = '\0';
            break;
        }
    }

    // Find AP_Param by name
    AP_Param::ParamToken token = {};
    ap_var_type type;
    AP_Param* param = AP_Param::find_by_name(name_buf, &type, &token);

    if (param == nullptr) {
        hal.console->printf("CMD: param '%s' not found\n", name_buf);
        return false;
    }

    // Write based on param type
    bool ok = false;
    switch (type) {
    case AP_PARAM_FLOAT: {
        AP_Float *pf = (AP_Float*)param;
        pf->set_and_save(cmd.value);
        ok = true;
        break;
    }

    case AP_PARAM_INT32: {
        AP_Int32 *pi = (AP_Int32*)param;
        pi->set_and_save((int32_t)cmd.value);
        ok = true;
        break;
    }

    case AP_PARAM_INT16: {
        AP_Int16 *pi = (AP_Int16*)param;
        pi->set_and_save((int16_t)cmd.value);
        ok = true;
        break;
    }

    case AP_PARAM_INT8: {
        AP_Int8 *pi = (AP_Int8*)param;
        pi->set_and_save((int8_t)cmd.value);
        ok = true;
        break;
    }

    default:
        hal.console->printf("CMD: unsupported type for '%s'\n", name_buf);
        return false;
    }

    if (ok) {
        hal.console->printf("CMD: set %s = %.3f\n", name_buf, (double)cmd.value);
    }

    return ok;
}

void AirBoss_Networking::send_airboss_state(const AirBoss_Joystick::JoystickState& js, const AirBoss_Switches& switches, uint16_t voltage, uint16_t charging_current)
{
    if (!telem_out.is_open()) {
        return;
    }

    // Define a packed binary frame for network transmission
    struct PACKED JoystickPacket {
        uint32_t magic;
        uint32_t timestamp_us;
        uint16_t raw[8];    // 16 bytes
        float    norm[8];   // 32 bytes
        uint8_t  healthy;
        uint16_t buttons;     // buttons packed in 12 bits (uses 16 bits)
        uint8_t  switches;    // 2×3-way packed into 4 bits
        uint16_t batt_mv;     // battery voltage in millivolts
        uint16_t charge_ma;   // charging current in milliamps

        uint16_t crc;
    };

    JoystickPacket pkt {};
    pkt.magic = 0xAABBCCDD;
    pkt.timestamp_us = js.last_update_us;
    pkt.raw[0] = js.left_thumb.x.raw;
    pkt.raw[1] = js.left_thumb.y.raw;
    pkt.raw[2] = js.right_thumb.x.raw;
    pkt.raw[3] = js.right_thumb.y.raw;
    pkt.raw[4] = js.left_index.x.raw;
    pkt.raw[5] = js.right_index.x.raw;
    pkt.raw[6] = js.left_index.y.raw;
    pkt.raw[7] = js.right_index.y.raw;

    pkt.norm[0] = js.left_thumb.x.norm;
    pkt.norm[1] = js.left_thumb.y.norm;
    pkt.norm[2] = js.right_thumb.x.norm;
    pkt.norm[3] = js.right_thumb.y.norm;
    pkt.norm[4] = js.left_index.x.norm;
    pkt.norm[5] = js.right_index.x.norm;
    pkt.norm[6] = js.left_index.y.norm;
    pkt.norm[7] = js.right_index.y.norm;

    pkt.buttons = switches.compute_button_mask();
    pkt.switches = switches.pack_three_way_switches();
    pkt.batt_mv = voltage;
    pkt.charge_ma = charging_current;

    pkt.healthy = js.healthy ? 1 : 0;

    // Simple CRC16
    uint16_t crc = 0;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&pkt);
    for (size_t i = 0; i < sizeof(pkt) - sizeof(pkt.crc); i++) {
        crc = (crc >> 8) ^ ((crc ^ p[i]) & 0xFFu);
    }
    pkt.crc = crc;

    telem_out.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    // hal.console->printf("Sent joystick: LT(%u,%u) RT(%u,%u) LI(%u,%u) RI(%u,%u) healthy=%u\n",
    //                     pkt.raw[0], pkt.raw[1], pkt.raw[2], pkt.raw[3],
    //                     pkt.raw[4], pkt.raw[6], pkt.raw[5], pkt.raw[7],
    //                     pkt.healthy);
}

void AirBoss_Networking::send_sbus_packet(const uint8_t* sbus_packet, size_t length)
{
    // Validate
    if (!rc_out.is_open() || sbus_packet == nullptr) {
        return;
    }

    // Ensure correct SBUS size (25 bytes typical)
    if (length == 0) {
        return;
    }

    // Optionally clamp size if caller passed more than 25
    if (length > 25) {
        length = 25;
    }

    // Transmit the full pre-built SBUS frame
    rc_out.write(sbus_packet, length);
}

// void AirBoss_Networking::handle_rc_rx()
// {
//     uint8_t buf[128];
//     const size_t n = rc_in.read(buf, sizeof(buf));
//     if (n == 0) return;

//     if (n == sizeof(RCFrame)) {
//         const RCFrame* frame = reinterpret_cast<const RCFrame*>(buf);
//         // TODO: validate CRC and use RC values
//         hal.console->printf("RC received: ch1=%u ch2=%u\n",
//                             frame->channels[0], frame->channels[1]);
//     }
// }

// void AirBoss_Networking::send_rc(const RCFrame& frame)
// {
//     telem_out.write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
// }

// void AirBoss_Networking::send_debug(const char* msg)
// {
//     debug_out.write(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
// }

