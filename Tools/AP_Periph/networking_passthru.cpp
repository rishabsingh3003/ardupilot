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

#include "AP_Periph.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

#include <dronecan_msgs.h>
    
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200
#define SBUS_RANGE_MAX 1800
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

#define SBUS_TARGET_MIN 1000
#define SBUS_TARGET_MAX 2000
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

// this is 875
#define SBUS_SCALE_OFFSET (SBUS_TARGET_MIN - ((SBUS_TARGET_RANGE * SBUS_RANGE_MIN / SBUS_RANGE_RANGE)))

#ifndef HAL_SBUS_FRAME_GAP
#define HAL_SBUS_FRAME_GAP 2000U
#endif

#if AP_PERIPH_NETWORKING_ENABLED && HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0

#include <AP_SerialManager/AP_SerialManager.h>

const AP_Param::GroupInfo Networking_Periph::Passthru::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Passthrough
    // @Description: Enable Passthrough of any UART, Network, or CAN ports to any UART, Network, or CAN ports.
    // @Values: 0:Disabled, 1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1,  Networking_Periph::Passthru, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: EP1
    // @DisplayName: Endpoint 1
    // @Description: Passthrough Endpoint 1. This can be a serial port UART, a Network port, or a CAN port. The selected port will route to Endport 2.
    // @Values: -1:Disabled, 0:Serial0(usually USB), 1:Serial1, 2:Serial2, 3:Serial3, 4:Serial4, 5:Serial5, 6:Serial6, 7:Serial7, 8:Serial8, 9:Serial9, 21:Network Port1, 22:Network Port2, 23:Network Port3, 24:Network Port4, 25:Network Port5, 26:Network Port6, 27:Network Port7, 28:Network Port8, 29:Network Port9, 41:CAN1 Port1, 42:CAN1 Port2, 43:CAN1 Port3, 44:CAN1 Port4, 45:CAN1 Port5, 46:CAN1 Port6, 47:CAN1 Port7, 48:CAN1 Port8, 49:CAN1 Port9, 51:CAN2 Port1, 52:CAN2 Port2, 53:CAN2 Port3, 54:CAN2 Port4, 55:CAN2 Port5, 56:CAN2 Port6, 57:CAN2 Port7, 58:CAN2 Port8, 59:CAN2 Port9
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EP1", 2,  Networking_Periph::Passthru, ep1, -1),

    // @Param: EP2
    // @DisplayName: Endpoint 2
    // @Description: Passthrough Endpoint 2. This can be a serial port UART, a Network port, or a CAN port. The selected port will route to Endport 1.
    // @CopyFieldsFrom: NET_PASS1_EP1
    AP_GROUPINFO("EP2", 3,  Networking_Periph::Passthru, ep2, -1),

    // @Param: BAUD1
    // @DisplayName: Endpoint 1 Baud Rate
    // @Description: The baud rate used for Endpoint 1. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_BAUD
    AP_GROUPINFO("BAUD1", 4,  Networking_Periph::Passthru, baud1, 115200),

    // @Param: BAUD2
    // @DisplayName: Endpoint 2 Baud Rate
    // @Description: The baud rate used for Endpoint 2. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_BAUD
    AP_GROUPINFO("BAUD2", 5,  Networking_Periph::Passthru, baud2, 115200),

    // @Param: OPT1
    // @DisplayName: Serial Port Options EP1
    // @Description: Control over UART options for Endpoint 1. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_OPTIONS
    AP_GROUPINFO("OPT1", 6,  Networking_Periph::Passthru, options1, 0),

    // @Param: OPT2
    // @DisplayName: Serial Port Options EP2
    // @Description: Control over UART options for Endpoint 2. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_OPTIONS
    AP_GROUPINFO("OPT2", 7,  Networking_Periph::Passthru, options2, 0),

#if HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS
    // @Param: PAR1
    // @DisplayName: Serial Port Parity
    // @Description: UART Parity Configuration for Endpoint 1.Only applies to serial ports.
    // @Values: 0: No parity, 1: Odd parity, 2: Even parity
    AP_GROUPINFO("PAR1", 8, Networking_Periph::Passthru, parity1, -1),

    // @Param: ST_BT1
    // @DisplayName: Serial Port Stop Bits
    // @Description: UART Stop Bits Configuration for Endpoint 1. Only applies to serial ports.
    // @Values: 0: 0 stop bit, 1: 1 stop bits, 2: 2 stop bits
    AP_GROUPINFO("ST_BT1", 9, Networking_Periph::Passthru, stop_bits1, -1),

    // @Param: PAR2
    // @DisplayName: Serial Port Parity
    // @Description: UART Parity Configuration for Endpoint 2. Only applies to serial ports.
    // @CopyFieldsFrom: NET_PASS1_PAR1
    AP_GROUPINFO("PAR2", 10, Networking_Periph::Passthru, parity2, -1),

    // @Param: ST_BT2
    // @DisplayName: Serial Port Stop Bits
    // @Description: UART Stop Bits Configuration for Endpoint 2. Only applies to serial ports.
    // @CopyFieldsFrom: NET_PASS1_ST_BT1
    AP_GROUPINFO("ST_BT2", 11, Networking_Periph::Passthru, stop_bits2, -1),
#endif

    // @Param: CHK_CRC
    // @DisplayName: Check CRC
    // @Description: Check CRC for SBUS frames. This is a simple checksum to verify the integrity of the data.
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO("CRC", 12, Networking_Periph::Passthru, check_crc, 1),

    // @Param: RC_MODE
    // @DisplayName: Send RC over CAN or UART
    // @Discription: Send RC over CAN or UART. This is used to send RC data over the selected endpoint.
    // @Values: 1:UART, 2:CAN, 3:Both
    AP_GROUPINFO("RC", 13, Networking_Periph::Passthru, rc_mode, 1),


    AP_GROUPEND
};


void Networking_Periph::Passthru::init()
{
    if (enabled == 0) {
        // Feature is disabled
        return;
    }

    if (port1 != nullptr || port2 != nullptr) {
        // The ports have already been initialized, nothing to do.
        return;
    }

    if (ep1 <= -1 || ep2 <= -1 || ep1 == ep2) {
        // end points are not set or are the same. Can't route to self
        return;
    }

    port1 = AP::serialmanager().get_serial_by_id(ep1);
    port2 = AP::serialmanager().get_serial_by_id(ep2);

#if HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS
    configure_serial_port(port1, baud1, options1, 2, 2);
    configure_serial_port(port2, baud2, options2, 2, 2);
#else
    configure_serial_port(port1, baud1, options1, -1, -1);
    configure_serial_port(port2, baud2, options2, -1, -1);
#endif
}

void Networking_Periph::Passthru::configure_serial_port(AP_HAL::UARTDriver *port, uint32_t baud, uint32_t options, int8_t parity, int8_t stop_bits)
{
    if (port == nullptr) {
        return;
    }

    port->set_options(options);
    port->begin(baud);

#if HAL_PERIPH_NETWORK_PASSTHRU_ENABLE_PARITY_STOP_BITS
    // Configure parity and stop bits if enabled
    if (parity >= 0) {
        port->configure_parity(parity);
    }
    if (stop_bits >= 0) {
        port->set_stop_bits(stop_bits);
    }
#endif
}

void Networking_Periph::Passthru::update()
{
    if (enabled == 0 || port1 == nullptr || port2 == nullptr) {
        return;
    }

    // Fastest possible connection is 3Mbps serial port, which is roughly 300KB/s payload and we service this at <= 1kHz
    // Raising this any higher just causes excess stack usage which never gets used.
    uint8_t buf[300];

    // read from port1, and write to port2
    auto avail = port1->available();
    if (avail > 0) {
        auto space = port2->txspace();
        const uint32_t n = MIN(space, sizeof(buf));
        // can_printf("P1->P2: %f bytes\n", (double)n);
        const auto nbytes = port1->read(buf, n);
        if (nbytes > 0) {
            process_sbus_buffer(buf, nbytes);
            if (new_sbus_frame) {
                // Send the latest SBUS frame to port2
                if (rc_mode == 1 || rc_mode == 3) {
                    port2->write(latest_sbus_frame, sizeof(latest_sbus_frame));
                }
                new_sbus_frame = false;  // Reset the flag after sending
                if (sbus_decode(latest_sbus_frame, decoded_rc_values, &rc_num_values,
                                _sbus_failsafe, SBUS_INPUT_CHANNELS) &&
                    rc_num_values >= MIN_RCIN_CHANNELS) {
                    if(rc_mode == 2 || rc_mode == 3) {
                        // Send the decoded RC values over CAN
                        new_sbus_can_frame = true;
                    } else {
                        new_sbus_can_frame = false;
                    }

                    // can_send_RCInput(
                    //     0, decoded_rc_values, rc_num_values,
                    //     sbus_failsafe, false);
                }
            }
        }
    }

    // read from port2, and write to port1
    avail = port2->available();
    if (avail > 0) {
        auto space = port1->txspace();
        const uint32_t n = MIN(space, sizeof(buf));
        // can_printf("P2->P1: %f bytes\n", (double)n);
        const auto nbytes = port2->read(buf, n);
        if (nbytes > 0) {
            port1->write(buf, nbytes);
        }
    }
}

// CRC-8-Dallas/Maxim, polynomial 0x31
uint8_t Networking_Periph::Passthru::sbus_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void Networking_Periph::Passthru::process_sbus_buffer(const uint8_t *buf, uint32_t nbytes)
{
    const uint32_t SBUS_FRAME_SIZE = 25;

    // Append new data to carry buffer
    uint32_t total_len = carry_buffer_len + nbytes;
    if (total_len > sizeof(carry_buffer)) {
        total_len = sizeof(carry_buffer);  // prevent overflow
    }

    // Copy new data to carry buffer
    memcpy(&carry_buffer[carry_buffer_len], buf, total_len - carry_buffer_len);
    carry_buffer_len = total_len;

    uint32_t offset = 0;

    // Parse complete SBUS frames
    while (carry_buffer_len - offset >= SBUS_FRAME_SIZE) {
        const uint8_t *packet = &carry_buffer[offset];

        // Look for SBUS header
        if (packet[0] == 0x0F) {
            // Found header, check footer
            bool is_valid = (packet[24] == 0x00 || packet[24] == 0x04);
            // Check CRC over bytes 0–22
            uint8_t computed_crc = sbus_crc8(packet, 23);  // Exclude byte 23 (CRC) and 24 (footer)
            // can_printf("CRC: %02X %02X\n", computed_crc, packet[23]);
            bool crc_valid = true;
            if (check_crc) {
                crc_valid = computed_crc == packet[23];
            }
            if (is_valid && crc_valid) {
                // Valid SBUS packet
                valid_sbus_packets++;
                total_sbus_bytes += SBUS_FRAME_SIZE;

                // Check for timing/jitter
                uint32_t now = AP_HAL::millis();
                uint32_t frame_interval = now - last_sbus_timestamp;
                if (last_sbus_timestamp != 0 && (frame_interval > 30)) {
                    uint32_t missed_frames = (frame_interval / 20) - 1; // compute how many were missed
                    lost_frames += missed_frames;
                }
                last_sbus_timestamp = now;
                // Copy the valid SBUS frame to latest_sbus_frame
                memcpy(latest_sbus_frame, packet, SBUS_FRAME_SIZE);
                latest_sbus_frame[23] = 0x00;  // Set the footer to 0x00
                // Set the flag to indicate a new SBUS frame is available
                new_sbus_frame = true;
            } else {
                // Invalid SBUS packet (bad footer)
                invalid_sbus_packets++;
            }

            // Move forward by one full frame
            offset += SBUS_FRAME_SIZE;
        } else {
            // Not a valid SBUS header, move forward 1 byte to search again
            offset += 1;
        }
    }

    // Keep leftover bytes for next round
    carry_buffer_len -= offset;
    if (carry_buffer_len > 0 && offset > 0) {
        memmove(carry_buffer, &carry_buffer[offset], carry_buffer_len);
    }
}

// decode a full SBUS frame
bool Networking_Periph::Passthru::sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
                                     bool &sbus_failsafe, uint16_t max_values)
{
    /* check frame boundary markers to avoid out-of-sync cases */
    if ((frame[0] != 0x0f)) {
        return false;
    }

    uint16_t chancount = SBUS_INPUT_CHANNELS;

    decode_11bit_channels((const uint8_t*)(&frame[1]), max_values, values,
        SBUS_TARGET_RANGE, SBUS_RANGE_RANGE, SBUS_SCALE_OFFSET);

    /* decode switch channels if data fields are wide enough */
    if (max_values > 17 && SBUS_INPUT_CHANNELS > 15) {
        chancount = 18;

        /* channel 17 (index 16) */
        values[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0))?1998:998;
        /* channel 18 (index 17) */
        values[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1))?1998:998;
    }

    /* note the number of channels decoded */
    *num_values = chancount;

    /*
      as SBUS is such a weak protocol we additionally check if any of
      the first 4 channels are at or below the minimum value of
      875. We consider the frame as a failsafe in that case, which
      means we log the data but won't use it
     */
    bool invalid_data = false;
    for (uint8_t i=0; i<4; i++) {
        if (values[i] <= SBUS_SCALE_OFFSET) {
            invalid_data = true;
        }
    }

    if (invalid_data) {
        sbus_failsafe = true;
    }

    /* decode and handle failsafe and frame-lost flags */
    // if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
    //     /* report that we failed to read anything valid off the receiver */
    //     sbus_failsafe = true;
    // } else if (invalid_data) {
    //     sbus_failsafe = true;
    // } else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
    //     /* set a special warning flag
    //      *
    //      * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
    //      * condition as fail-safe greatly reduces the reliability and range of the radio link,
    //      * e.g. by prematurely issuing return-to-launch!!! */

    //     sbus_failsafe = false;
    // } else {
    //     sbus_failsafe = false;
    // }

    return true;
}

/*
  decode channels from the standard 11bit format (used by CRSF, SBUS, FPort and FPort2)
  must be used on multiples of 8 channels
*/
void Networking_Periph::Passthru::decode_11bit_channels(const uint8_t* data, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset)
{
#define CHANNEL_SCALE(x) ((int32_t(x) * mult) / div + offset)
    while (nchannels >= 8) {
        const Channels11Bit_8Chan* channels = (const Channels11Bit_8Chan*)data;
        values[0] = CHANNEL_SCALE(channels->ch0);
        values[1] = CHANNEL_SCALE(channels->ch1);
        values[2] = CHANNEL_SCALE(channels->ch2);
        values[3] = CHANNEL_SCALE(channels->ch3);
        values[4] = CHANNEL_SCALE(channels->ch4);
        values[5] = CHANNEL_SCALE(channels->ch5);
        values[6] = CHANNEL_SCALE(channels->ch6);
        values[7] = CHANNEL_SCALE(channels->ch7);

        nchannels -= 8;
        data += sizeof(*channels);
        values += 8;
    }
}


/*
//   send an RCInput CAN message
//  */
// void Networking_Periph::Passthru::can_send_RCInput(uint8_t quality, uint16_t *values, uint8_t nvalues, bool in_failsafe, bool quality_valid)
// {
//     uint16_t status = 0;
//     if (quality_valid) {
//         status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
//     }
//     if (in_failsafe) {
//         status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;
//     }

//     // assemble packet
//     dronecan_sensors_rc_RCInput pkt {};
//     pkt.quality = quality;
//     pkt.status = status;
//     pkt.rcin.len = nvalues;
//     for (uint8_t i=0; i<nvalues; i++) {
//         pkt.rcin.data[i] = values[i];
//     }

//     // encode and send message:
//     uint8_t buffer[DRONECAN_SENSORS_RC_RCINPUT_MAX_SIZE];

//     uint16_t total_size = dronecan_sensors_rc_RCInput_encode(&pkt, buffer, !periph.canfdout());

//     canard_broadcast(DRONECAN_SENSORS_RC_RCINPUT_SIGNATURE,
//                      DRONECAN_SENSORS_RC_RCINPUT_ID,
//                      CANARD_TRANSFER_PRIORITY_HIGH,
//                      buffer,
//                      total_size);
// }



#endif  // AP_PERIPH_NETWORKING_ENABLED && HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0

