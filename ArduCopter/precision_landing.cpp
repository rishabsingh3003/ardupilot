//
// functions to support precision landing
//

#include "Copter.h"
#include "precision_landing.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init(400);
}

void Copter::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;

    // use range finder altitude if it is valid, otherwise use home alt
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.alt_cm_glitch_protected;
    }

    precland.update(height_above_ground_cm, rangefinder_alt_ok());
}
#endif

AC_PrecLand_State_Machine_Copter::AC_PrecLand_State_Machine_Copter(AC_PrecLand &precland):
AC_PrecLand_State_Machine(precland)
{
    
}

bool AC_PrecLand_State_Machine_Copter::update( Vector3f &loc)
{
    plnd_status();
    landing_retry_status();

    if (current_landing_retry_state() == LANDING_RETRY_STATE::LANDING_RETRY_VERTICAL) {
        get_retry_fly_location(loc);
        return true;
    } else if (current_landing_retry_state() == LANDING_RETRY_STATE::LANDING_RETRY_FAILED) {
        return false;
    }
    return false;
}