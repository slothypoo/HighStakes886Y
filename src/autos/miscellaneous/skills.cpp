#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void skills() {
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  autoStarted = true;
  chassis.setPose(0, 0, 0);

  armTarget = restingPos;
  intakeRaise.set_value(true);
  conveyor.move_velocity(-12000);
  pros::delay(700); // Ring on alliance stake
  conveyor.move_velocity(0);

  chassis.moveToPoint(0, 14, 1000);
  chassis.turnToHeading(-90, 700);
  chassis.moveToPoint(
      16.5, 14, 1200,
      {.forwards = false, .maxSpeed = 55}); // back into first goal
  chassis.waitUntilDone();
  pros::delay(80);
  goalClamp.set_value(true); // clamps on first goal
  chassis.turnToHeading(12, 800);
  chassis.moveToPoint(24.5, 31.5, 1000);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.turnToHeading(71, 800);
  chassis.moveToPoint(46, 39.2, 1200); // gets 2 rings

  chassis.turnToHeading(35, 800); // aim for neutral ring
  chassis.moveToPoint(58.5, 57.5, 1200);
  chassis.turnToHeading(-11, 800); // the ring for neutral stake

  chassis.moveToPoint(54, 85, 1300, {.maxSpeed = 70});
  chassis.waitUntil(15);
  armTarget = loadingPos;
  chassis.waitUntilDone();
  pros::delay(350);
  chassis.moveToPoint(57, 66, 1200,{.forwards = false}); // moving back to neutral stake
  chassis.turnToHeading(90, 800);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(80, 66, 1200);
  chassis.waitUntil(3);
  armTarget = wallStakePos;
  // scoring neutral stake
  chassis.moveToPoint(49, 66, 1200, {.forwards = false});
  chassis.waitUntilDone();
  armTarget = restingPos;
  chassis.turnToHeading(180, 800); // aim for bottom 2 rings
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(50.2, 6, 2000, {.maxSpeed = 90});  // intakes 2 rings
  chassis.turnToHeading(60, 800);                      // aim for last ring
  chassis.moveToPoint(60, 15, 2000); // intakes last ring
  chassis.turnToHeading(-39, 800);
  chassis.waitUntil(1);
  intake.move_velocity(5000);
  conveyor.move_velocity(5000);
  chassis.waitUntil(2);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(63, 5, 2000, {.forwards = false});
  chassis.waitUntilDone();
  intake.move_velocity(1000);
  conveyor.move_velocity(1000);
  pros::delay(300);
  goalClamp.set_value(false); // put goal down
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(61, 15, 1000);
  chassis.turnToHeading(90, 750);
  chassis.moveToPose(-22, 15, 90, 2000, {.forwards = false, .maxSpeed = 120}); // back at to 2nd goal
  chassis.waitUntilDone();
  goalClamp.set_value(true); // clamps 2nd goal
  pros::delay(100);
  // armTarget = 12000;
  chassis.turnToHeading(-2, 800);
  chassis.moveToPoint(-23, 36, 1000);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.waitUntilDone();
  pros::delay(300);
  chassis.turnToHeading(-90, 800);
  chassis.moveToPoint(-45, 39, 1000);
  chassis.turnToHeading(-30, 800);
  chassis.moveToPoint(-57, 60, 1100);
  chassis.turnToHeading(13, 800);
  chassis.moveToPoint(-47, 88, 1200);
  chassis.waitUntil(12);
  armTarget = loadingPos;
  chassis.waitUntilDone();
  armTarget = loadingPos;
  pros::delay(200);
  chassis.moveToPoint(-57, 71, 1200, {.forwards = false});
  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(-80, 71, 1200);
  chassis.waitUntil(3);
  armTarget = wallStakePos;
  chassis.moveToPoint(-49, 68, 900, {.forwards = false});
  chassis.waitUntilDone();
  armTarget = restingPos;
  chassis.turnToHeading(180, 800); // aim for bottom 2 rings
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(-50.5, 6, 2000, {.maxSpeed = 90}); // intakes 2 rings
  chassis.turnToHeading(-60, 800);                       // aim for last ring
  chassis.moveToPoint(-60, 15, 2000);  // intakes last ring
  chassis.turnToHeading(39, 800);
  chassis.waitUntil(1);
  intake.move_velocity(5000);
  conveyor.move_velocity(5000);
  chassis.waitUntil(2);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(-63, 5, 1700, {.forwards = false});
  chassis.waitUntil(1);
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.waitUntilDone();
  intake.move_velocity(1000);
  conveyor.move_velocity(1000);
  pros::delay(200);
  goalClamp.set_value(false);
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPose(-20.7, 82.6, 51, 2000, {.minSpeed = 95});
  chassis.waitUntil(1);
  intake.move_velocity(-12000);
  pros::delay(300);
  chassis.turnToHeading(-145, 1000);
  chassis.moveToPoint(6, 107, 1200, {.forwards = false, .maxSpeed = 60});
  chassis.waitUntilDone();
  goalClamp.set_value(true);
  /*
  chassis.turnToHeading(0, 900);
  chassis.waitUntilDone();
  pros::delay(200);
   intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(6.5, 130, 800, {.maxSpeed = 60});
  chassis.moveToPoint(6.5, 117.3, 800, {.forwards = false, .maxSpeed = 60});
  chassis.waitUntilDone();
  armTarget = 34000;
  pros::delay(500);
  chassis.moveToPoint(7, 110, 800, {.forwards = false, .maxSpeed = 60});
  chassis.waitUntilDone();
  armTarget = 39000;
  chassis.turnToHeading(140, 1000);
  chassis.moveToPoint(31, 87, 1200, {.maxSpeed = 60});
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.turnToHeading(225, 1000);
  chassis.waitUntilDone();
  pros::delay(200);
  conveyor.move_velocity(0);
  chassis.moveToPoint(9, 60, 1200);
  chassis.moveToPoint(31, 87, 1200, {.forwards = false});
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000); 
  chassis.turnToHeading(45, 800);
  chassis.moveToPoint(55, 115, 1000);
  chassis.turnToHeading(250, 800);
  chassis.waitUntilDone();
  intake.move_velocity(1000);
  conveyor.move_velocity(1000);
  pros::delay(300);
  goalClamp.set_value(false);
  chassis.moveToPoint(80, 135, 1200, {.forwards = false});
  chassis.moveToPoint(60, 110, 1000, {.minSpeed = 100});
  chassis.moveToPose(-100, 150, -90, 2500, {.minSpeed = 100});

  */

  
  chassis.moveToPoint(-47, 106.5, 1400, {.maxSpeed = 90});
   intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.turnToHeading(135, 1000);
  chassis.waitUntilDone();
  intake.move_velocity(1000);
  conveyor.move_velocity(1000);
  pros::delay(300);
  goalClamp.set_value(false);
  chassis.moveToPoint(-68, 130, 700, {.forwards = false});
  chassis.moveToPose(29, 84.5, 116, 2500, {.maxSpeed = 95});
  chassis.waitUntil(1);
  armTarget = loadingPos;
  chassis.waitUntilDone();
  pros::delay(800);
  chassis.moveToPoint(7.5, 117, 1500, {.forwards = false, .maxSpeed = 90});

  chassis.turnToHeading(0, 1400, {.maxSpeed = 90});
  chassis.moveToPoint(6.5, 130, 800, {.maxSpeed = 60});
  chassis.moveToPoint(6.5, 114.1, 800, {.forwards = false, .maxSpeed = 60});
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.waitUntilDone();
  armTarget = allianceStakePos;
  pros::delay(600);
  chassis.moveToPoint(7.5, 110, 1500, {.forwards = false, .minSpeed = 110});
  chassis.waitUntilDone();
  pros::delay(500);
  armTarget = armRaisedPos;
  chassis.turnToHeading(50, 1000, {.maxSpeed = 110});
  chassis.moveToPoint(70, 140, 1500, {.minSpeed = 110});
  

  pros::delay(3000);
  intake.move_velocity(0);
  conveyor.move_velocity(0);

}