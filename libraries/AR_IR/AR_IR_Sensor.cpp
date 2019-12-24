#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"
#include "AR_IR_Sensor_Analog.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>


const AP_Param::GroupInfo AR_IR::var_info[] = {

// @Param: IR_PIN_1
// @DisplayName: Infrared Sensor 1 pin number
// @Description: Analog Pin number to where the first infrared sensor is attached to the left of the rover
// @Range: 0 20
AP_GROUPINFO("PIN_1", 1, AR_IR, param._ir_1_pin_num, IR_1_PIN_NUM),

// @Param: IR_PIN_2
// @DisplayName: Infrared Sensor 2 pin number
// @Description: Analog Pin number to where the second infrared sensor is attached to the right of the rover 
// @Range: 0 20
AP_GROUPINFO("PIN_2", 2, AR_IR, param._ir_2_pin_num, IR_2_PIN_NUM),

// @Param: IR_ADC_VOLTAGE
// @DisplayName: Infrared Sensor ADC voltage value
// @Description: ADV voltage value number to where the second infrared sensor is attached
// @Values: 3.3,5,6.6
AP_GROUPINFO("ADC_VOLTAGE", 3, AR_IR, param.adc_voltage, 5),



AP_GROUPEND
}; 

AR_IR::AR_IR(){
	
}

void AR_IR::init(){
	drivers = new AR_IR_Analog(*this, state);
}


void AR_IR::read() {
	drivers->read();
}

void AR_IR:: calc_line_error(float &line_error){
	float sensor_error = state.voltage_ir_1 - state.voltage_ir_2; //0v from sensor corresponds to the sensor on black line
	gcs().send_text(MAV_SEVERITY_CRITICAL, "voltage 1 %10.10f", (double) state.voltage_ir_1);
	gcs().send_text(MAV_SEVERITY_CRITICAL, "voltage 2 %10.10f", (double) state.voltage_ir_2);
	line_error = linear_interpolate(-1,1,sensor_error,-param.adc_voltage,param.adc_voltage);
}


	