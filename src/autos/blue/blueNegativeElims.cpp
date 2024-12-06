#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void blueNegativeElims() {
  autoStarted = true;
  armTarget = restingPos;
  chassis.setPose(55, 23.5, 90);
  chassis.moveToPose(27, 23.5, 90, 1000, {.forwards = false});
  chassis.waitUntilDone();
  intakeRaise.set_value(true);
  goalClamp.set_value(true);
  pros::delay(500);
  chassis.turnToPoint(9, 39, 1000);
  pros::delay(300);
  intake.move_voltage(-12000);
  conveyor.move_voltage(-12000);
  chassis.moveToPoint(9, 39, 1000);
  chassis.turnToPoint(6, 47, 1000);
  chassis.moveToPoint(6, 47, 1000);
  pros::delay(500);
  chassis.moveToPoint(12, 33, 1000, {.forwards = false});
  chassis.turnToPoint(20, 42.5, 1000);
  chassis.moveToPoint(20, 42.5, 1000);
  chassis.turnToHeading(90, 1000);
  chassis.moveToPose(65, 65, 45, 2000, {.minSpeed = 60});
  chassis.waitUntilDone();
  conveyor.move_velocity(6000);
  pros::delay(200);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(65, 65, 200, {.maxSpeed = 30});
  chassis.turnToHeading(180, 500);
//   // QUALS - 5 RING + BAR TOUCH//
//   chassis.turnToPoint(23.5, 0, 1000);
//   chassis.moveToPoint(33.5, 15.6, 1000);
//   chassis.waitUntilDone();
//   intake.move_voltage(0);
//   conveyor.move_voltage(0);
//   chassis.moveToPoint(23.5, 0, 3000, {.maxSpeed = 40});

  // ELIMS ONLY BELOW - 6 RING//
  chassis.turnToPoint(49, 10, 500);
  chassis.waitUntilDone();
  intakeRaise.set_value(false);
  chassis.moveToPoint(49, 10, 1000);
  chassis.waitUntilDone();
  pros::delay(200);
  intakeRaise.set_value(true);
  pros::delay(700);
  chassis.moveToPoint(52, 19, 1000, {.forwards = false});
  chassis.waitUntilDone();
  pros::delay(500);
  conveyor.move_velocity(0);
}

