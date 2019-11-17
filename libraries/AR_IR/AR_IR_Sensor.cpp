#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"

extern const AP_HAL::HAL& hal ;

AR_IR::AR_IR(){
	_IR_pin_analog_source = hal.analogin->channel(14);
}


float AR_IR::readIR(int sensor_pin) {
	int output_start = -1;
	int output_end = 1;
	int output_range = output_end - output_start; 
	
	_IR_pin_analog_source->set_pin(sensor_pin);

	float mapped_output = (_IR_pin_analog_source->read_latest())*output_range / 5 + output_start; //map the output from anlog input to -1 to 1  
	return mapped_output;
}