#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"


void redRushAlliance() {
  autoStarted = true;
  chassis.setPose(-62.5, -12.5, 325);
  // armTarget = loadingPos;
  // pros::delay(500);
  // conveyor.move_velocity(-12000);
  // chassis.moveToPose(61.5, -10, 45, 700);
  // chassis.waitUntilDone();
  // conveyor.move_velocity(0);
  // pros::delay(100);
  // armTarget = allianceStakePos;
  armTarget = allianceStakePos;
  pros::delay(800);
  chassis.moveToPoint(-57.5, -14, 700, {.forwards = false});
  chassis.turnToPoint(-50, 0, 500, {.maxSpeed = 60});
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  armTarget = armRaisedPos;
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
  chassis.turnToPoint(-25, -41, 1000, {.maxSpeed = 80});
  chassis.waitUntilDone();
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(-25, -41, 1000);
  chassis.moveToPoint(-25, -35, 500, {.forwards = false});
  chassis.waitUntilDone();
  pros::delay(700);
  conveyor.move_velocity(0);
  intake.move_velocity(0);
  goalClamp.set_value(false);
  chassis.moveToPoint(-25, -41, 700);
  chassis.turnToPoint(-8, -46, 800, {.forwards = false});
  chassis.moveToPoint(-8, -46, 800, {.forwards = false, .maxSpeed = 60});
  chassis.waitUntilDone();
  goalClamp.set_value(true);
  pros::delay(100);
  chassis.moveToPoint(-55, -33, 1000);
  chassis.turnToPoint(-65, -65, 700);
  chassis.moveToPoint(-65, -65, 1200, {.maxSpeed = 60});
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  pros::delay(1000);
  chassis.turnToHeading(90 ,1000);
  intake.move_velocity(0);
}

