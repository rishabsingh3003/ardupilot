/*
 Class to handle AirBoss_Networking Joystick via SPI based ADC
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/Socket.h>
#include <AP_Networking/AP_Networking_address.h>
#include "airboss_joystick.h"

#define AP_AIRBOSS_NETWORKING_DEFAULT_UDP_IP_ADDR "192.168.144.2"

#ifndef AP_PERIPH_AIRBOSS_NETWORKING_ENABLED
#define AP_PERIPH_AIRBOSS_NETWORKING_ENABLED 1
#endif


#if AP_PERIPH_AIRBOSS_NETWORKING_ENABLED


// Lightweight, self-contained UDP channel abstraction.
// Works as client (connect) or server (bind).
class UDPChannel {
    public:
        UDPChannel() = default;
        ~UDPChannel() { close(); }
    
        // --- Lifecycle ----------------------------------------------------------
        bool open_client(const char* ip_str, uint16_t port);
        bool open_server(uint16_t port);
        void close();
    
        // --- I/O ---------------------------------------------------------------
        size_t write(const uint8_t* buf, size_t len, uint8_t* error = nullptr);
        size_t read(uint8_t* buf, size_t len, int timeout_ms = 0, uint8_t* error = nullptr);
    
        // --- Status -------------------------------------------------------------
        bool is_open() const { return sock != nullptr; }
        bool is_connected() const { return connected; }
    
        uint32_t tx_bytes() const { return tx_bytes_total; }
        uint32_t rx_bytes() const { return rx_bytes_total; }
    
    private:
        SocketAPM* sock = nullptr;
        bool connected = false;
    
        uint32_t tx_bytes_total = 0;
        uint32_t rx_bytes_total = 0;
    };
    

// Structure of an example RC frame (customize as needed)
#pragma pack(push, 1)
struct RCFrame {
    uint32_t timestamp_us;
    uint16_t channels[8];
    uint16_t crc;
};
#pragma pack(pop)

// AirBoss_Networking handles multiple UDP channels:
//   - RC input  (server mode, joystick → UAV)
//   - RC output (client mode, UAV → joystick/GCS)
//   - Telemetry output (client mode)
//   - Debug/log output (client mode)
class AirBoss_Networking {
public:
    AirBoss_Networking();
    ~AirBoss_Networking() = default;

    bool init();

    void loop_50hz();            // telemetry sender
    void handle_rc_rx();         // process incoming RC packets
    void send_rc(const RCFrame& frame);
    void send_debug(const char* msg);

    void send_airboss_state(const AirBoss_Joystick::JoystickState& js);

private:
    UDPChannel rc_in;
    UDPChannel rc_out;
    UDPChannel telem_out;
    UDPChannel debug_out;

    void send_telem_json();
};


#endif // AP_PERIPH_AIRBOSS_NETWORKING_JOYSTICK_ENABLED

