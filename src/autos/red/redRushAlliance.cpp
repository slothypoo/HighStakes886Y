#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"


void redRushAlliance() {
  autoStarted = true;
  armTarget = 12000;
  chassis.setPose(0, 0, 195);

  chassis.moveToPoint(8, 30.3, 1000,
                      {.forwards = false, .minSpeed = 72, .earlyExitRange = 7});
  // chassis.swingToHeading(205, DriveSide::LEFT, 1000, {.minSpeed = 30,
  // .earlyExitRange = 10});
  chassis.turnToHeading(205, 600, {.minSpeed = 40, .earlyExitRange = 10});
  chassis.moveToPose(15, 44, 205, 1000, {.forwards = false, .minSpeed = 40});
  chassis.waitUntilDone();
  pros::delay(200);
  goalClamp.set_value(true);

  // chassis.moveToPose(17,46, 195, 1950, {.forwards = false});

  // chassis.waitUntilDone();
  // pros::delay(200);
  // goalClamp.set_value(true);
  // chassis.turnToHeading(240, 1000);
  // chassis.turnToPoint(-6.5, -51.5, 400, {.forwards = false});

  // intake.move_velocity(-12000);
  // chassis.moveToPoint(-66, -63, 1500, {.maxSpeed = 70});
  //  chassis.waitUntilDone();
  //
  // conveyor.move_velocity(-12000);
}

