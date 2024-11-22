#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void blueRushAlliance() {
  autoStarted = true;
  armTarget = 12000;
  chassis.setPose(50, -62.25, 90);
  chassis.moveToPose(27, -62.26, 90, 700, {.forwards = false, .minSpeed = 90});
  // chassis.turnToHeading(240, 1000);
  chassis.turnToPoint(5.5, -51.5, 400, {.forwards = false});
  chassis.moveToPoint(5.5, -51.5, 1200, {.forwards = false, .maxSpeed = 50});
  chassis.waitUntilDone();
  pros::delay(200);
  goalClamp.set_value(true);
  intakeRaise.set_value(true);
  pros::delay(200);
  intake.move_voltage(-12000);
  conveyor.move_voltage(-12000);
  chassis.moveToPose(23, -50, 90, 1000);
  chassis.waitUntilDone();
  pros::delay(100);
  intake.move_voltage(0);
  conveyor.move_voltage(0);
  goalClamp.set_value(false);
  chassis.moveToPose(23, -48, 90, 600);
  chassis.waitUntilDone();
  // chassis.turnToHeading(180, 1000);
  // chassis.turnToPoint(-18, -32, 1000, {.forwards = false});
  chassis.turnToPoint(23, -25, 750, {.forwards = false});
  chassis.moveToPoint(23, -25, 1000, {.forwards = false, .minSpeed = 50});
  chassis.waitUntilDone();
  pros::delay(200);
  goalClamp.set_value(true);
  pros::delay(200);
  conveyor.move_velocity(-12000);
  intake.move_velocity(-12000);
  chassis.turnToPoint(41, -3, 700);
  chassis.waitUntilDone();
  intakeRaise.set_value(false);
  armTarget = 16200;
  chassis.moveToPoint(41, -3, 900, {.maxSpeed = 70});
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  intakeRaise.set_value(true);
  pros::delay(200);
  chassis.moveToPoint(36, -10, 600, {.forwards = false});
  chassis.waitUntilDone();
  intake.move_velocity(0);
  chassis.turnToPoint(53.5, -3, 500);
  chassis.moveToPoint(53.5, -3, 900, {.maxSpeed = 60});
  // chassis.moveToPoint(-56, -10, 1000, {.maxSpeed = 50});
  // chassis.turnToPoint(-70, 3, 700, {.maxSpeed = 40});
  chassis.waitUntilDone();
  conveyor.move_velocity(0);
  pros::delay(200);
  armTarget = 30000;
  pros::delay(300);
  armTarget = 35000;
  pros::delay(500);
  chassis.moveToPoint(44, -7.5, 600, {.forwards = false});
  chassis.waitUntilDone();
  armTarget = 20000;
  chassis.turnToPoint(66, -63, 600);
  chassis.moveToPoint(59.5, -47.5, 600, {.minSpeed = 100});
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  chassis.moveToPoint(66, -63, 1500, {.maxSpeed = 70});
  // chassis.waitUntilDone();
  conveyor.move_velocity(-12000);
}

