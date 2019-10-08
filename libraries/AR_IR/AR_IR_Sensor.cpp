#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"

extern const AP_HAL::HAL& hal = AP_HAL::get_HAL();




float AR_IR::readIR()
{
int pin = 14;
_IR_pin_analog_source->set_pin(pin);
float voltage  = _IR_pin_analog_source->voltage_average();
return voltage;
}

