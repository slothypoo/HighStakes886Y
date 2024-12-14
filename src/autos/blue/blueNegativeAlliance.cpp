#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void blueNegativeAlliance() {
  autoStarted = true;
  armTarget = restingPos;
  chassis.setPose(55, 23.5, 90);
  chassis.moveToPose(27, 23.5, 90, 1000, {.forwards = false, .minSpeed = 50});
  chassis.waitUntilDone();
  pros::delay(200);
  intakeRaise.set_value(true);
  goalClamp.set_value(true);
//   pros::delay(100);
  chassis.turnToPoint(9, 39, 700);
//   chassis.waitUntilDone();
//   pros::delay(300);
  chassis.moveToPoint(9, 39, 800);
  chassis.waitUntil(4);
  intake.move_voltage(-12000);
  conveyor.move_voltage(-12000);
  chassis.turnToPoint(6, 47, 500);
  chassis.moveToPoint(6, 47, 700);
//   pros::delay(500);
  chassis.moveToPoint(12, 33, 800, {.forwards = false});
  chassis.turnToPoint(20, 42.5, 700);
  chassis.moveToPoint(20, 42.5, 800);
  chassis.turnToHeading(90, 700);
  chassis.moveToPose(65, 65, 45, 2200, {.minSpeed = 75});
  chassis.waitUntilDone();
//   conveyor.move_velocity(6000);
  pros::delay(100);
  conveyor.move_velocity(-12000);
//   chassis.moveToPoint(-65, 65, 600, {.maxSpeed = 30});
  chassis.turnToHeading(180, 500);
  chassis.waitUntilDone();
  conveyor.move_velocity(-12000);
//   //QUALS - 5 RING + BAR TOUCH//
//   chassis.turnToPoint(-23.5, 0, 1000);
//   chassis.moveToPoint(-33.5, 15.6, 1000);
//   chassis.waitUntilDone();
//   intake.move_voltage(0);
//   conveyor.move_voltage(0);
//   chassis.moveToPoint(-23.5, 0, 3000, {.maxSpeed = 40});

  // ELIMS ONLY BELOW - 6 RING//
  chassis.turnToPoint(49, 7, 500);
  chassis.waitUntilDone();
  intakeRaise.set_value(false);
  chassis.moveToPoint(49, 7, 1500, {.maxSpeed = 90});
  chassis.waitUntil(10);
  armTarget = loadingPos;
//   pros::delay(200);
  chassis.waitUntilDone();
  intakeRaise.set_value(true);
  pros::delay(200);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(49, 19, 1000, {.forwards = false});
//   chassis.turnToHeading(225, 800);
  chassis.turnToPoint(62.5, 8.75, 800);
  //SCORE
  chassis.moveToPoint(62.5, 8.75, 1000, {.maxSpeed = 60});
  chassis.waitUntil(3);
  conveyor.move_velocity(0);
  intake.move_velocity(0);
  armTarget = allianceStakePos;
  pros::delay(750);
  chassis.moveToPoint(51, 19, 800, {.forwards = false});
  chassis.turnToPoint(23, 0, 800);
  chassis.waitUntilDone();
  armTarget = armRaisedPos;
  chassis.moveToPoint(31.5, 5, 1000, {.maxSpeed = 70});
  chassis.waitUntil(8);
  armTarget = wallStakePos;
}