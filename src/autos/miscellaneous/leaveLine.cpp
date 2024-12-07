#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void leaveLine() {
    autoStarted = true;
    armTarget = restingPos;
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 5, 0, 1000);
}