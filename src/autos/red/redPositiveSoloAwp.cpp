#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void redPositiveSoloAWP() {
  autoStarted = true;
  chassis.setPose(-62.5, -12.5, 325);
  armTarget = allianceStakePos;
  pros::delay(800);
  // chassis.setPose(-57.5, -14, 315);
  // armTarget = loadingPos;
  // pros::delay(500);
  // conveyor.move_velocity(-12000);
  // chassis.moveToPose(-61.5, -10, 315, 700);
  // chassis.waitUntilDone();
  // conveyor.move_velocity(0);
  // pros::delay(100);
  // armTarget = allianceStakePos;
  // pros::delay(400);
  chassis.moveToPoint(-57.5, -14, 500, {.forwards = false});
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
  pros::delay(200);
  chassis.moveToPoint(-58, -35, 1000, {.forwards = false});
  chassis.turnToPoint(-28.5, 1, 1000);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(-39, -15, 1000, {.minSpeed = 40});
  chassis.waitUntilDone();
  armTarget = wallStakePos;
  // chassis.moveToPoint(-28.5, 1, 2000, {.maxSpeed = 40});

  // chassis.waitUntilDone();
  // pros::delay(1000);
  // goalClamp.set_value(false);
  // pros::delay(200);
  // conveyor.move_velocity(0);
  // intake.move_velocity(0);
  // chassis.moveToPoint(-24, -48, 500);
  // chassis.turnToPoint(-10, -49, 500, {.forwards = false});
  // chassis.moveToPoint(-10, -49, 1000, {.forwards = false, .maxSpeed = 50});
  // chassis.waitUntilDone();
  // pros::delay(200);
  // goalClamp.set_value(true);
  // pros::delay(200);
  // chassis.turnToPoint(-63, -63, 200);
  // chassis.moveToPoint(-63, -63, 2500, {.maxSpeed = 80});
  // chassis.waitUntil(10);
  // intake.move_velocity(-12000);
  // chassis.waitUntilDone();
  // // chassis.moveToPoint(-65, -65, 1000, {.maxSpeed = 40});
  // conveyor.move_velocity(-12000);
  // chassis.turnToHeading(180, 1000, {.maxSpeed = 40});
}

