#include "autons.hpp"
#include "lemlibglobals.hpp"
#include "globals.hpp"

void redRushFast () {
    chassis.setPose(-52, -61.5, 65.8);
    autoStarted = true;
    armTarget = restingPos;
    intakeRaise.set_value(true);
    doinker.set_value(true);
    intake.move_velocity(-12000);
    chassis.moveToPose(-17, -47.8, 65.8, 1200, {.lead=0.2, .minSpeed = 84});
    chassis.waitUntilDone();
    pros::delay(150);
    rushClamp.set_value(true);
    chassis.moveToPose(-43, -58, 65.8, 5000, {.forwards = false, .minSpeed = 60});
    chassis.waitUntil(25);
    rushClamp.set_value(false);
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(400);
    chassis.turnToPoint(-28, -27.5, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.moveToPoint(-28, -27.5, 1000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(150);
    goalClamp.set_value(true);
    pros::delay(200);
    conveyor.move_velocity(-12000);
    chassis.moveToPoint(-38, -47, 1000);
    chassis.waitUntilDone();
    pros::delay(200);
    goalClamp.set_value(false);
    pros::delay(200);
    intake.move_velocity(0);
    conveyor.move_velocity(0);
    chassis.turnToPoint(-25, -52.5, 800, {.forwards = false});
    chassis.moveToPoint(-25, -52.5, 800, {.forwards = false});
    chassis.waitUntilDone();
    goalClamp.set_value(true);
    chassis.turnToPoint(-65, -65, 700);
    chassis.moveToPoint(-54.5, -61, 700);
    chassis.waitUntilDone();
    intake.move_velocity(-12000);
    chassis.moveToPoint(-65, -65, 700, {.maxSpeed = 35});
    chassis.waitUntilDone();
    // pros::delay(100);
    conveyor.move_velocity(-12000);
    pros::delay(100);
    chassis.moveToPoint(-57, -57, 800, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    doinker.set_value(true);
    rushClamp.set_value(true);
    intake.move_velocity(0);
    conveyor.move_velocity(0);
    pros::delay(400);
    //chassis.turnToPoint(-59, -60, 400);
    //chassis.moveToPoint(-59, -60, 700, {.maxSpeed = 40});
    chassis.turnToHeading(90, 2000, {.minSpeed = 80});
    chassis.moveToPoint(-18, -48, 1500);
    chassis.waitUntil(10);
    goalClamp.set_value(false);
    rushClamp.set_value(false);
    /*
    chassis.turnToHeading(200, 1600);
    conveyor.move_velocity(0);
    intake.move_velocity(0);
    */
    // chassis.turnToPoint(-65, -65, 700);
    // chassis.moveToPoint(-44.5, -57.5, 800, {.forwards =false});
    // chassis.waitUntilDone();
    // goalClamp.set_value(false);
    // chassis.turnToHeading(351.5, 800);
    // 351.5


}