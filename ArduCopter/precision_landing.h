#pragma once

#include <AC_PrecLand/AC_PrecLand_State_Machine.h>

class AC_PrecLand_State_Machine_Copter : public AC_PrecLand_State_Machine
{
public:
    AC_PrecLand_State_Machine_Copter(AC_PrecLand &precland);
    bool update(Vector3f &loc);

private:

};