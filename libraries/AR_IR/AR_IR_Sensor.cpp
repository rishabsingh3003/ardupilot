#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal ;


AR_IR::AR_IR()
{
	_IR_pin_analog_source = hal.analogin->channel(14);}

float AR_IR::readIR()
{
// _IR_pin_analog_source = hal.analogin->channel(0);
int pin = 14;
int i=0;
float voltage=0,final_return=0;
gcs().send_text(MAV_SEVERITY_INFO, "FLAGB");
_IR_pin_analog_source->set_pin(pin);
for(i=0;i<200;i++)
{
 voltage  = voltage + _IR_pin_analog_source->voltage_average();

}
final_return=voltage/200;
gcs().send_text(MAV_SEVERITY_INFO, "FLAGA");
return final_return;
}