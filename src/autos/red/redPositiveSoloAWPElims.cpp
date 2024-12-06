#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void redPositiveSoloAWPElims() {
  autoStarted = true;
  chassis.setPose(-57.5, -14, 315);
  armTarget = 16200;
  pros::delay(500);
  conveyor.move_velocity(-12000);
  chassis.moveToPose(-61.5, -10, 315, 700);
  chassis.waitUntilDone();
  conveyor.move_velocity(0);
  pros::delay(100);
  armTarget = 32000;
  pros::delay(300);
  armTarget = 35000;
  pros::delay(400);
  chassis.moveToPoint(-57.5, -14, 500, {.forwards = false});
  chassis.turnToPoint(-50, 0, 500, {.maxSpeed = 60});
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  armTarget = 20000;
  chassis.moveToPoint(-50, 0, 500, {.maxSpeed = 60});
  chassis.waitUntilDone();
  // pros::delay(300);
  intakeRaise.set_value(true);
  // armTarget = 12000;
  pros::delay(500);
  chassis.moveToPoint(-52, -7, 700, {.forwards = false, .maxSpeed = 60});
  chassis.turnToPoint(-27, -21.5, 700, {.forwards = false});
  chassis.moveToPoint(-38, -15, 400, {.forwards = false});
  chassis.moveToPoint(-27, -21.5, 700, {.forwards = false, .maxSpeed = 50});
  chassis.waitUntilDone();
  pros::delay(200);
  goalClamp.set_value(true);
  pros::delay(200);
  chassis.turnToPoint(-24, -41, 1000, {.maxSpeed = 60});
  chassis.waitUntilDone();
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(-24, -41, 1000);
  chassis.moveToPoint(-24, -35, 500, {.forwards = false});
  chassis.turnToPoint(-58, -35, 500);
  chassis.moveToPoint(-58, -35, 700);
  chassis.waitUntil(10);
  conveyor.move_velocity(0);
  intake.move_velocity(0);
  chassis.turnToPoint(-70, -65, 400);
  chassis.moveToPoint(-70, -65, 1250, {.maxSpeed = 60});
  chassis.waitUntil(10);
  intake.move_velocity(-12000);
  chassis.waitUntilDone();
  conveyor.move_velocity(-12000);
}

