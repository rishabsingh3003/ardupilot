#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal ;


AR_IR::AR_IR()
{
	_IR_pin_analog_source = hal.analogin->channel(14);}

AR_IR::~AR_IR()
{free (_IR_pin_analog_source);}

float AR_IR::readIR(int sensor_pin)
{
// _IR_pin_analog_source = hal.analogin->channel(0);

float voltage=0;
int input_start = -500;
int input_end = 500;
int output_start = -4500;
int output_end = 4500;
int input_range = input_end - input_start;
int output_range = output_end - output_start;

_IR_pin_analog_source->set_pin(sensor_pin);
    

voltage = (_IR_pin_analog_source->read_latest() - input_start)*output_range / input_range + output_start;
//gcs().send_text(MAV_SEVERITY_INFO, "FLAGB");


//gcs().send_text(MAV_SEVERITY_INFO, "FLAGA");
return voltage;
}