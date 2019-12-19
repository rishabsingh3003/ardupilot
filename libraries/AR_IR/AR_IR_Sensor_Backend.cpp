#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_IR_Sensor.h"
#include "AR_IR_Sensor_Backend.h"

AR_IR_Backend:: AR_IR_Backend(AR_IR &val, AR_IR::AR_IR_State &val_state) :

        _val(val),
        _state(val_state)
{
}
