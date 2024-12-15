#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"
#include "pros/colors.hpp"

void blueRushFast () {
    chassis.setPose(52, -32, 245.8);
    autoStarted = true;
    armTarget = restingPos;
    intakeRaise.set_value(true);
    doinker.set_value(true);
    intake.move_velocity(-12000);
    chassis.moveToPose(17, -47.8, 245.8, 1200, {.lead=0.2, .minSpeed = 84});
    chassis.waitUntilDone();
    pros::delay(210);
    rushClamp.set_value(true);
    //pullback
    chassis.moveToPose(28, -42.5, 245.8, 5000, {.forwards = false, .minSpeed = 60});
    // chassis.moveToPose(43, -58, 270, 5000, {.forwards = false, .minSpeed = 60});
    // chassis.waitUntil(25);
    // rushClamp.set_value(false);
    // chassis.waitUntilDone();
    // doinker.set_value(false);
    pros::delay(400);
    chassis.turnToPoint(25, -29, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    rushClamp.set_value(false);
    chassis.moveToPoint(25, -29, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(150);
    goalClamp.set_value(true);
    pros::delay(200);
    // conveyor.move_velocity(-12000);
    // chassis.moveToPoint(38, -47, 1000);
    chassis.turnToPoint(40, -35, 700, {.forwards = false});
    chassis.waitUntilDone();
    conveyor.move_velocity(-12000);
    chassis.moveToPoint(40, -35, 700, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(200);
    goalClamp.set_value(false);
    pros::delay(200);
    intake.move_velocity(0);
    conveyor.move_velocity(0);
    //CLAMP SECOND GOAl
    chassis.turnToPoint(20, -55, 800, {.forwards = false});
    chassis.moveToPoint(20, -55, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    goalClamp.set_value(true);
    chassis.turnToPoint(65, -65, 700);
    chassis.moveToPoint(56, -63, 700);
    // chassis.waitUntilDone();
    intake.move_velocity(-12000);
    chassis.moveToPoint(65, -65, 1350, {.maxSpeed = 35});
    chassis.waitUntilDone();
    conveyor.move_velocity(-12000);
    pros::delay(500);
    chassis.moveToPoint(44.8, -60.5, 800, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    doinker.set_value(true);
    rushClamp.set_value(true);
    intake.move_velocity(0);
    conveyor.move_velocity(0);
    chassis.moveToPoint(55, -63, 700, {.maxSpeed = 40});
    chassis.turnToHeading(0, 2000, {.minSpeed = 80});
    // chassis.moveToPoint(18, -48, 1500);
    chassis.waitUntilDone();
    doinker.set_value(false);
    chassis.turnToHeading(245, 1000);
    chassis.moveToPose(15, -57, 290, 1500, {.minSpeed = 70});
    chassis.waitUntil(10);
    doinker.set_value(true);
    goalClamp.set_value(false);
    rushClamp.set_value(false);
    // pros::delay(400);
    // //chassis.turnToPoint(-59, -60, 400);
    // //chassis.moveToPoint(-59, -60, 700, {.maxSpeed = 40});
    // chassis.turnToHeading(270, 2000, {.minSpeed = 80});
    // /*
    // chassis.turnToHeading(200, 1600);
    // conveyor.move_velocity(0);
    // intake.move_velocity(0);
    // */
    // // chassis.turnToPoint(-65, -65, 700);
    // // chassis.moveToPoint(-44.5, -57.5, 800, {.forwards =false});
    // // chassis.waitUntilDone();
    // // goalClamp.set_value(false);
    // // chassis.turnToHeading(351.5, 800);
    // // 351.5


}