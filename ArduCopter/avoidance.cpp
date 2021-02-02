#include "Copter.h"

void Copter::low_alt_avoidance()
{   
    int32_t alt_cm;
    if(!get_rangefinder_height_interpolated_cm(alt_cm)) {
        return;
    }

    bool enable_avoidance = true;
    if (alt_cm < avoid.get_min_alt()*100) {
        enable_avoidance = false;
    }
    avoid.proximity_alt_avoidance_enable(enable_avoidance);
}