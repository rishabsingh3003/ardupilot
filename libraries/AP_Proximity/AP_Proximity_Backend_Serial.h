#pragma once

#include "AP_Proximity_Backend.h"

#if HAL_PROXIMITY_ENABLED

class AP_Proximity_Backend_Serial : public AP_Proximity_Backend
{
public:
    AP_Proximity_Backend_Serial(AP_Proximity &_frontend,
                                AP_Proximity::Proximity_State &_state, uint8_t instance = 0);
    // static detection function
    static bool detect(uint8_t instance = 0);

protected:
    virtual uint16_t rxspace() const { return 0; };

    uint8_t _this_instance;

    AP_HAL::UARTDriver *_uart;              // uart for communicating with sensor
};

#endif // HAL_PROXIMITY_ENABLED
