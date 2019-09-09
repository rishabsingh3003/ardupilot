#include "mode.h"
#include "Rover.h"

void ModeLineFollower::update()
{
    g2.motors.set_throttle(100.0f);
}