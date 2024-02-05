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

   The Lightware SF45B serial interface is described on this wiki page
   http://support.lightware.co.za/sf45/#/commands
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED

#include "AP_Proximity_LightWareSF45B_I2C.h"
#include <utility>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/Semaphores.h>

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>

#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

static const uint32_t PROXIMITY_SF45B_TIMEOUT_MS = 200;
static const uint32_t PROXIMITY_SF45B_REINIT_INTERVAL_MS = 5000;    // re-initialise sensor after this many milliseconds
static const float PROXIMITY_SF45B_COMBINE_READINGS_DEG = 5.0f;     // combine readings from within this many degrees to improve efficiency
static const uint32_t PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM = 5;
static const uint8_t PROXIMITY_SF45B_DESIRED_UPDATE_RATE = 6;       // 1:48hz, 2:55hz, 3:64hz, 4:77hz, 5:97hz, 6:129hz, 7:194hz, 8:388hz
static const uint32_t PROXIMITY_SF45B_DESIRED_FIELDS = ((uint32_t)1 << 0 | (uint32_t)1 << 8);   // first return (unfiltered), yaw angle
static const uint16_t PROXIMITY_SF45B_DESIRED_FIELD_COUNT = 2;      // DISTANCE_DATA_CM message should contain two fields



AP_Proximity_LightWareSF45B_I2C::AP_Proximity_LightWareSF45B_I2C(AP_Proximity &_frontend,
                                                              AP_Proximity::Proximity_State &_state,
															  AP_Proximity_Params  &_params,
                                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_Proximity_Backend(_frontend, _state, _params)
    , _dev(std::move(dev))
{
}


// detect if a TOFSenseP rangefinder is connected. We'll detect by
// trying to take a reading on I2C. If we get a result the sensor is
// there.
AP_Proximity_Backend *AP_Proximity_LightWareSF45B_I2C::detect(AP_Proximity &_frontend,
                                                              AP_Proximity::Proximity_State &_state,
															  AP_Proximity_Params  &_params,
                                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)4);

        return nullptr;
    }

    AP_Proximity_LightWareSF45B_I2C *sensor
        = new AP_Proximity_LightWareSF45B_I2C(_frontend, _state, _params, std::move(dev));
    if (!sensor) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)5);
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)6);
        return nullptr;
    }

            gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)8);


    return sensor;
}


// initialise sensor
bool AP_Proximity_LightWareSF45B_I2C::init(void)
{
    _dev->get_semaphore()->take_blocking();

    if (!configure_sensor(0x1FF)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)99);
        _dev->get_semaphore()->give();
        return false;
    }

    gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)1);

    // give time for the sensor to process the request
    hal.scheduler->delay(100);

    get_configuration(_configuration);

    gcs().send_text(MAV_SEVERITY_CRITICAL, "config world! %5.3f", (double)_configuration);


    uint16_t reading_cm;
    uint16_t yaw;
    uint16_t signal_strength;

    if (!get_reading(reading_cm, yaw, signal_strength)) {
        _dev->get_semaphore()->give();
        return false;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)2);


    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_Proximity_LightWareSF45B_I2C::timer, void));

    return true;
}

bool AP_Proximity_LightWareSF45B_I2C::configure_sensor(uint32_t configuration)
{
    // for (uint i = 0; i < 100; i++) {
    //     uint8_t cmd_1[] = {80, 0xAA, 0xAA};

    //     // send command to get configuration
    //     if (!_dev->transfer((cmd_1), sizeof(cmd_1), nullptr, 0)) {
    //         return false;
    //     }
    // }
    // uint8_t cmd_1[] = {80};

    // // send command to get configuration
    // if (!_dev->transfer((cmd_1), sizeof(cmd_1), nullptr, 0)) {
    //     return false;
    // }

    // send command to set configuration
    uint8_t cmd[5] = {27,(uint8_t)((configuration >> 0) & 0xFF),(uint8_t)((configuration >> 8) & 0xFF),(uint8_t)((configuration >> 16) & 0xFF),(uint8_t)((configuration >> 24) & 0xFF)};
    if (!_dev->transfer((uint8_t *) &cmd, sizeof(cmd), nullptr, 0)) {
        return false;
    }
        gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)200);

        return true;
}

bool AP_Proximity_LightWareSF45B_I2C::get_configuration(uint32_t &configuration)
{
    uint8_t cmd[] = {27};

    // send command to get configuration
    if (!_dev->transfer(cmd, sizeof(cmd), nullptr, 0)) {
        return false;
    }
    hal.scheduler->delay(100);
uint8_t cmd_b[4];
    // read back results
    if( _dev->transfer(nullptr, 0, (uint8_t *) &cmd_b, sizeof(cmd_b))) {
        configuration = (uint32_t)cmd_b[0] | (uint32_t)cmd_b[1] << 8 | (uint32_t)cmd_b[2] << 16 | (uint32_t)cmd_b[3] << 24;
        return true;
    }
    return false;
}

// start_reading() - ask sensor to make a range reading
bool AP_Proximity_LightWareSF45B_I2C::start_reading()
{
    uint8_t cmd = 44;

   // read back results
    return _dev->transfer((uint8_t *) &cmd, sizeof(cmd), nullptr, 0);
}

// read - return last value measured by sensor
bool AP_Proximity_LightWareSF45B_I2C::get_reading(uint16_t &reading_cm, uint16_t &yaw_angle_deg, uint16_t &status)
{
    // trigger a new reading
    start_reading();

    struct PACKED {
        uint16_t first_distance_cm;
        uint16_t first_filtered_distance_cm;
        uint16_t firstStrength;
        uint16_t last;
        uint16_t last_filtered;
        uint16_t lastStrength;
        uint16_t noise;
        uint16_t noise_1;
        uint16_t yaw_angle_deg;
    } packet;

    // take range reading and read back results
    const bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &packet, sizeof(packet));

    if (ret) {
        // combine results into distance
        reading_cm = packet.first_distance_cm;
        yaw_angle_deg = packet.yaw_angle_deg/100;
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "hello world! %5.3f", (double)yaw_angle_deg);

    }


    return ret;
}

//  timer called at 10Hz
void AP_Proximity_LightWareSF45B_I2C::timer(void)
{
    uint16_t dist_cm;
    uint16_t status;
    uint16_t yaw_angle_deg;

    if (get_reading(dist_cm, yaw_angle_deg, status)) {
        WITH_SEMAPHORE(_sem);
        // if (status == 1) {
            // healthy data
            distance_cm = dist_cm;
            yaw_deg = yaw_angle_deg;
            new_distance = true;
            _last_distance_received_ms = AP_HAL::millis();
        // }
    }
}


// update the state of the sensor
void AP_Proximity_LightWareSF45B_I2C::update(void)
{
    WITH_SEMAPHORE(_sem);
    {
    if (new_distance) {
        float distance_m = distance_cm * 0.01f;
        process_message(distance_m, yaw_deg);
        new_distance = false;
    }
    }

    // // initialise sensor if necessary
    // initialise();

    // // process incoming messages
    // process_replies();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_SF45B_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// // initialise sensor
// void AP_Proximity_LightWareSF45B::initialise()
// {
//     // check sensor is configured correctly
//     _init_complete = (_sensor_state.stream_data_type == PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM) &&
//                      (_sensor_state.update_rate == PROXIMITY_SF45B_DESIRED_UPDATE_RATE) &&
//                      (_sensor_state.streaming_fields == PROXIMITY_SF45B_DESIRED_FIELDS);

//     // exit if initialisation requests have been sent within the last few seconds
//     uint32_t now_ms = AP_HAL::millis();
//     if ((now_ms - _last_init_ms) < PROXIMITY_SF45B_REINIT_INTERVAL_MS) {
//         return;
//     }
//     _last_init_ms = now_ms;

//     // request stream rate and contents
//     request_stream_start();
// }

// // request start of streaming of distances
// void AP_Proximity_LightWareSF45B::request_stream_start()
// {
//     // request output rate
//     send_message((uint8_t)MessageID::UPDATE_RATE, true, &PROXIMITY_SF45B_DESIRED_UPDATE_RATE, sizeof(PROXIMITY_SF45B_DESIRED_UPDATE_RATE));

//     // request first return (unfiltered), and yaw angle
//     send_message((uint8_t)MessageID::DISTANCE_OUTPUT, true, (const uint8_t*)&PROXIMITY_SF45B_DESIRED_FIELDS, sizeof(PROXIMITY_SF45B_DESIRED_FIELDS));

//     // request start streaming of DISTANCE_DATA_CM messages
//     send_message((uint8_t)MessageID::STREAM, true, (const uint8_t*)&PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM, sizeof(PROXIMITY_SF45B_STREAM_DISTANCE_DATA_CM));
// }

// // check for replies from sensor
// void AP_Proximity_LightWareSF45B::process_replies()
// {
//     if (_uart == nullptr) {
//         return;
//     }

//     // process up to 1K of characters per iteration
//     uint32_t nbytes = MIN(_uart->available(), 1024U);
//     while (nbytes-- > 0) {
//         const int16_t r = _uart->read();
//         if ((r < 0) || (r > 0xFF)) {
//             continue;
//         }
//         if (parse_byte((uint8_t)r)) {
//             process_message();
//         }
//     }
// }

// process the latest message held in the _msg structure
void AP_Proximity_LightWareSF45B_I2C::process_message(float distance_m, float angle)
{
    _last_distance_received_ms = AP_HAL::millis();
    float angle_deg = correct_angle_for_orientation(angle);

    // if distance is from a new face then update distance, angle and boundary for previous face
    // get face from 3D boundary based on yaw angle to the object
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);
    if (face != _face) {
        if (_face_distance_valid) {
            frontend.boundary.set_face_attributes(_face, _face_yaw_deg, _face_distance, state.instance);
        } else {
            // mark previous face invalid
            frontend.boundary.reset_face(_face, state.instance);
        }
        // record updated face
        _face = face;
        _face_yaw_deg = 0;
        _face_distance = INT16_MAX;
        _face_distance_valid = false;
    }

    // if distance is from a new minisector then update obstacle database using angle and distance from previous minisector
    const uint8_t minisector = convert_angle_to_minisector(angle_deg);
    if (minisector != _minisector) {
        if ((_minisector != UINT8_MAX) && _minisector_distance_valid) {
            database_push(_minisector_angle, _minisector_distance);
        }
        // init mini sector
        _minisector = minisector;
        _minisector_angle = 0;
        _minisector_distance = INT16_MAX;
        _minisector_distance_valid = false;
    }

    // check reading is valid
    if (!ignore_reading(angle_deg, distance_m) && (distance_m >= distance_min()) && (distance_m <= distance_max())) {
        // update shortest distance for this face
        if (!_face_distance_valid || (distance_m < _face_distance)) {
            _face_yaw_deg = angle_deg;
            _face_distance = distance_m;
            _face_distance_valid = true;
        }

        // update shortest distance for this mini sector
        if (distance_m < _minisector_distance) {
            _minisector_angle = angle_deg;
            _minisector_distance = distance_m;
            _minisector_distance_valid = true;
        }
    }

}

// convert an angle (in degrees) to a mini sector number
uint8_t AP_Proximity_LightWareSF45B_I2C::convert_angle_to_minisector(float angle_deg) const
{
    return wrap_360(angle_deg + (PROXIMITY_SF45B_COMBINE_READINGS_DEG * 0.5f)) / PROXIMITY_SF45B_COMBINE_READINGS_DEG;
}

#endif // AP_PROXIMITY_LIGHTWARE_SF45B_ENABLED
