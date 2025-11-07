/*
  Class to handle AirBoss_Networking Joystick via SPI based ADC
 */


#include "airboss_networking.h"
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <utility>
#include <cstdio>
#include <cstring>

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

    hal.console->printf("AirBoss_Networking init OK\n");
    return true;
}

void AirBoss_Networking::loop_50hz()
{
    // send telemetry JSON at 20 Hz – 50 Hz
    // send_airboss_state();

}

void AirBoss_Networking::send_airboss_state(const AirBoss_Joystick::JoystickState& js)
{
    if (!telem_out.is_open()) {
        return;
    }

    // Define a packed binary frame for network transmission
    struct PACKED JoystickPacket {
        uint32_t magic;           // 0xAABBCCDD for validation
        uint32_t timestamp_us;
        uint16_t raw[8];          // raw ADCs: LT.x, LT.y, RT.x, RT.y, LI.x, RI.x, LI.y, RI.y
        float norm[8];            // normalized -1..+1
        uint8_t healthy;
        uint8_t reserved[3];      // alignment padding
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

    pkt.healthy = js.healthy ? 1 : 0;

    // Simple CRC16 (optional — can be replaced with your utility)
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

