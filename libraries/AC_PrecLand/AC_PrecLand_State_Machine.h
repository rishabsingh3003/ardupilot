#include <AC_PrecLand/AC_PrecLand.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>

class AC_PrecLand_State_Machine {
public:
    AC_PrecLand_State_Machine(AC_PrecLand &precland);
    void init();
    void plnd_status();
    void landing_retry_status();
    bool run_vertical_retry();
    void get_retry_fly_location(Vector3f &loc) { loc = retry_location; }

    enum class PLD_STATE: uint8_t {
            PRECISION_LAND_STATE_NOT_ACTIVE = 0,
            PRECISION_LAND_STATE_INIT,
            PRECISION_LAND_STATE_ACTIVE,
            PRECISION_LAND_STATE_TARGET_LOST,
            PRECISION_LAND_STATE_SEARCH,
            PRECISION_LAND_STATE_DESCEND,
            PRECISION_LAND_STATE_DESCEND_FINAL,
            PRECISION_LAND_STATE_RETRY,
            PRECISION_LAND_STATE_SUCCESS,
            PRECISION_LAND_STATE_FAILED,
        };

    PLD_STATE current_plnd_status() { return _plnd_state; }

    enum class VERTICAL_RETRY_STATE: uint8_t {
            START = 0,
            CLIMB,
            REACHED_TOP,
            DESCEND,
            REACHED_BOTTOM
        };

    enum class LANDING_RETRY_STATE: uint8_t {
            LANDING_RETRY_NOT_REQUIRED = 0,
            LANDING_RETRY_VERTICAL,
            LANDING_RETRY_FAILED
        };

    LANDING_RETRY_STATE current_landing_retry_state() { return landing_retry_state; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    AC_PrecLand &_precland;

    bool last_status; // last known status of the landing target, true if it was in sight
    PLD_STATE _plnd_state;
    LANDING_RETRY_STATE landing_retry_state;
    Vector3f last_known_pos;

    // vertical run stuff

    VERTICAL_RETRY_STATE vertical_run_counter;
    Vector3f retry_location;

    AP_Int8 _enabled;
    AP_Int8 _type;
    AP_Int8 _fs_action;
};