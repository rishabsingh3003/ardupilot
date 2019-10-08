#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AR_IR.h>

class AR_IR
{
public:

   float readIR() override;

protected:
    AP_HAL::AnalogSource *_IR_pin_analog_source;
}