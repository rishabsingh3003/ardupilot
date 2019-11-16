#pragma once


#include <AP_HAL/AP_HAL.h>


class AR_IR
{
public:
	AR_IR();
  float readIR(int sensor_no);

protected:
  AP_HAL::AnalogSource *_IR_pin_analog_source;
};
