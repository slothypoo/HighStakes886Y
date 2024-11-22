#include "main.h"
#include "autons.hpp"
#include "globals.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlibglobals.hpp"
#include "mathfunc.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

//////////////////////////////////////////////////////////
// ARM PID
//////////////////////////////////////////////////////////

void on_left_button() {
  color = !color;
  if (color) {
    pros::lcd::set_text(4, "Color: Blue");
  } else {
    pros::lcd::set_text(4, "Color: Red");
  }
}

void on_center_button() {
  current = (current + 1) % 3;
  if (current == 0) {
    pros::lcd::set_text(5, "Autonomous Running: Negative Corner");
  } else if (current == 1) {
    pros::lcd::set_text(5, "Autonomous Running: Positive Corner");
  } else {
    pros::lcd::set_text(5, "Autonomous Running: Rush Alliance ");
  }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  // intakeRaise.set_value(1);
  armRotation.reset();
  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      pros::lcd::print(3, "Angle: %ld", armRotation.get_angle());
      // delay to save resources
      pros::delay(40);
    }
  });
  pros::Task Macro([&]() {
    while (autoStarted == true) {
      error = angleWrap(armTarget - armRotation.get_angle()); // 7250 = target
      derivative = (error - previous_error);
      if (fabs(error) < 0.5 || fabs(error + derivative) < 0.5) {
        arm.move_voltage(0);
        // break;
      }
      if (sign(error) != sign(previous_error)) {
        integral += error;
      } else {
        integral = 0;
      }
      arm.move_voltage(error * armkP + integral * armkI + derivative * armkD);
      previous_error = error;
      pros::delay(25);
    }
  });
  pros::lcd::set_text(4, "Color: Red");
  pros::lcd::set_text(5, "Autonomous Running: Negative Corner");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void no_auto() {}

void red_no_auto_AWP() {
  chassis.setPose(-62, -24, 90);
  chassis.moveToPose(-55, -24, 90, 1000);
}

void blue_no_auto_AWP() {
  chassis.setPose(62, -24, 270);
  chassis.moveToPose(55, -25, 270, 1000);
}

// void redPositiveAWP () {
// 	chassis.setPose(-57, -11.75, 180);
// 	chassis.moveToPose(-57, 3, 180, 1000, {.forwards = false});
// 	chassis.turnToHeading(90, 1000);
// 	chassis.moveToPoint(-60, 3, 1000, {.forwards = false, .maxSpeed = 30});
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(-12000);
// 	pros::delay(1000);
// 	chassis.turnToPoint(-53, 3, 1000);
// 	chassis.moveToPoint(-53, 3, 1000, {.maxSpeed = 30});
// 	chassis.waitUntilDone();
// 	intakeRaise.set_value(true);
// 	conveyor.move_voltage(0);
// 	intake.move_voltage(-12000);
// 	pros::delay(500);
// 	chassis.turnToPoint(-25, -23, 1000, {.forwards = false, .maxSpeed =
// 90}); 	chassis.moveToPoint(-25, -23, 1000, {.forwards = false,
// .maxSpeed = 90}); 	chassis.waitUntilDone(); 	pros::delay(700);
// goalClamp.set_value(true); 	conveyor.move_velocity(6000);
// 	chassis.turnToPoint(-23, -45, 1000);
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(-23, -45, 1000);
// 	chassis.waitUntilDone();
// 	chassis.turnToPoint(0, -25, 1000);
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(6000);
// 	pros::delay(500);
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(-10.5, -33.5, 1000);
// 	chassis.waitUntilDone();
// 	intake.move_voltage(0);
// 	chassis.moveToPoint(0, -25, 3000, {.maxSpeed = 40});
// }

// void skills() {
// 	chassis.setPose(-60.75, 0, 90);
// 	autoStarted = true;
// 	armTarget = 12000;
// 	conveyor.move_velocity(-12000);
// 	pros::delay(700);
// 	conveyor.move_velocity(0);
// 	chassis.moveToPose(-50, 0, 90, 700);
// 	chassis.turnToHeading(0, 800);
// 	chassis.moveToPoint(-47, -18, 1000, {.forwards = false, .maxSpeed =
// 60}); 	chassis.waitUntilDone(); 	pros::delay(200);
// goalClamp.set_value(true); 	intakeRaise.set_value(true); pros::delay(200);
// 	chassis.turnToPoint(-23.8, -23.8, 500);
// 	chassis.waitUntilDone();
// 	intake.move_velocity(-12000);
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(-23.8, -23.8, 1000, {.maxSpeed = 70});
// 	chassis.turnToPoint(-24, -47, 800);
// 	chassis.moveToPoint(-24, -47, 1000);
// 	chassis.turnToPoint(24, -50, 800);
// 	chassis.moveToPoint(24, -50, 1500);
// 	chassis.turnToPoint(2, -61, 1000);
// 	chassis.moveToPoint(2, -61, 1000, {.maxSpeed = 60});
// 	chassis.waitUntil(7);
// 	armTarget = 16200;
// 	chassis.waitUntilDone();
// 	pros::delay(500);
// 	chassis.moveToPoint(3, -59, 400, {.forwards = false});
// 	chassis.turnToHeading(180, 800);
// 	// chassis.moveToPose(2, -70, 180, 1000, {.minSpeed = 20});
// 	chassis.moveToPoint(2, -70, 800, {.maxSpeed = 40});
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(0);
// 	pros::delay(200);
// 	armTarget = 32000;
// 	chassis.moveToPoint(2, -70, 1000, {.maxSpeed = 40});
// 	chassis.moveToPoint(0, -55, 1200, {.forwards = false});
// 	chassis.waitUntilDone();
// 	armTarget = 20000;
// 	pros::delay(700);
// 	chassis.turnToPoint(-47, -65, 600);
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(-47, -65, 1500);
// 	chassis.turnToPoint(-47, -47, 800);
// 	chassis.moveToPoint(-47, -47, 800);
// 	chassis.turnToPoint(-59, -50, 700);
// 	chassis.moveToPoint(-59, -50, 1000);
// 	pros::delay(500);
// 	chassis.turnToPoint(-65, -65, 700, {.forwards = false});
// 	chassis.moveToPoint(-65, -65, 700, {.forwards = false});
// 	chassis.waitUntilDone();
// 	pros::delay(700);
// 	conveyor.move_velocity(2000);
// 	pros::delay(500);
// 	goalClamp.set_value(false);
// 	pros::delay(500);
// 	intake.move_velocity(0);
// 	conveyor.move_velocity(0);
// 	chassis.moveToPoint(-51, -37, 1200, {.maxSpeed = 60});
// 	chassis.turnToPoint(-51, 18, 1000, {.forwards = false, .maxSpeed = 80});
// 	// chassis.moveToPoint(-49, 0, 1000, {.forwards = false, .maxSpeed =
// 100}); 	chassis.moveToPoint(-51, 19, 2000, {.forwards = false, .maxSpeed
// = 60}); 	chassis.waitUntilDone();
// 	// pros::delay(200);
// 	goalClamp.set_value(true);
// 	pros::delay(200);
// 	chassis.turnToPoint(-23.8, 23.8, 1000);
// 	chassis.waitUntilDone();
// 	intake.move_velocity(-12000);
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(-23.7, 23.8, 1000);
// 	chassis.turnToPoint(-23.8, 50, 1000);
// 	chassis.moveToPoint(-23.8, 50, 1000);
// 	chassis.turnToPoint(23.8, 50, 1000);
// 	chassis.moveToPoint(23.8, 50, 1500);
// 	chassis.turnToPoint(0, 59, 1000);
// 	chassis.moveToPoint(0, 59, 1000);
// 	chassis.waitUntil(7);
// 	armTarget = 16200;
// 	chassis.turnToHeading(0, 1000);
// 	chassis.moveToPoint(0, 70, 1000);
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(0);
// 	pros::delay(200);
// 	armTarget = 32000;
// 	pros::delay(1000);
// 	chassis.moveToPoint(0, 55, 1000, {.forwards = false});
// 	chassis.waitUntilDone();
// 	armTarget = 20000;
// 	pros::delay(500);
// 	chassis.turnToPoint(-47, 50, 1000);
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(-47, 50, 1500);
// 	chassis.turnToPoint(-58.5, 50, 700);
// 	chassis.moveToPoint(-58.5, 50, 1000);
// 	chassis.turnToPoint(-47, 58.5, 1000);
// 	chassis.moveToPoint(-47, 58.5, 1000);
// 	chassis.turnToPoint(-65, 65, 1000, {.forwards = false});
// 	chassis.moveToPoint(-65, 65, 1000, {.forwards = false});
// 	chassis.waitUntilDone();
// 	pros::delay(500);
// 	conveyor.move_velocity(2000);
// 	goalClamp.set_value(false);
// 	pros::delay(500);
// 	conveyor.move_velocity(0);
// 	intake.move_velocity(0);
// 	chassis.turnToPoint(0, 47, 1000);
// 	chassis.moveToPoint(0, 47, 2000);
// 	chassis.turnToPoint(23.65, 23.65, 1000);
// 	chassis.waitUntilDone();
// 	intake.move_velocity(-12000);
// 	chassis.moveToPoint(25, 27, 1000);
// 	chassis.turnToPoint(48, 11, 700, {.forwards = false});
// 	chassis.moveToPoint(48, 11, 1000, {.forwards = false, .maxSpeed = 70});
// 	chassis.waitUntilDone();
// 	pros::delay(200);
// 	goalClamp.set_value(true);
// 	pros::delay(200);
// 	conveyor.move_velocity(-12000);
// 	chassis.turnToPoint(47, 47, 700);
// 	chassis.moveToPoint(47, 47, 1000);
// 	chassis.turnToPoint(65, 65, 1000, {.forwards = false});
// 	chassis.moveToPoint(65, 65, 1000, {.forwards = false});
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(12000);
// 	intake.move_velocity(0);
// 	goalClamp.set_value(false);
// 	chassis.moveToPoint(25, 25, 1000);
// 	chassis.turnToPoint(0, 0, 700, {.forwards = false});
// 	chassis.waitUntilDone();
// 	hang.set_value(true);
// 	chassis.moveToPoint(0, 0, 700, {.forwards = false, .minSpeed = 100});
// 	// chassis.turnToPoint(56, 45, 700);
// 	// chassis.moveToPoint(56, 45, 1000);
// 	// chassis.turnToPoint(51, 55, 700);
// 	// chassis.moveToPoint(51, 55, 1000);
// 	// chassis.turnToPoint(65, 65, 700, {.forwards = false});
// 	// chassis.moveToPoint(65, 65, 1000, {.forwards = false});
// 	// chassis.waitUntilDone();
// 	// conveyor.move_velocity(0);
// 	// intake.move_velocity(0);
// 	// goalClamp.set_value(false);

// }

void skills() {

  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  autoStarted = true;
  chassis.setPose(0, 0, 0);

  /* arm test

  armTarget = 16250;
  pros::delay(700);
  chassis.moveToPoint(0, 10, 1000);
  chassis.waitUntilDone();
  armTarget = 30000;
  pros::delay(1250);	//scoring neutral stake
  armTarget = 21000;
  pros::delay(700);
  chassis.moveToPoint(0, 20, 1000);
  chassis.waitUntil(2);
  armTarget = 22000;
  pros::delay(700);
  armTarget = 12000;

  */

  armTarget = 12000;
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
  pros::delay(300);
  goalClamp.set_value(true); // clamps on first goal
  chassis.turnToHeading(12, 800);
  chassis.moveToPoint(24.5, 31.5, 1000);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.turnToHeading(71, 800);
  chassis.moveToPoint(46, 39.2, 1500); // gets 2 rings

  chassis.turnToHeading(35, 1000); // aim for neutral ring
  chassis.moveToPoint(58.5, 57.5, 1200);
  chassis.turnToHeading(-11, 1000); // the ring for neutral stake

  chassis.moveToPoint(54, 85, 1300, {.maxSpeed = 70});
  chassis.waitUntil(4);
  armTarget = 16250;
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.moveToPoint(57, 66, 1200,
                      {.forwards = false}); // moving back to neutral stake
  chassis.turnToHeading(90, 900);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(69, 66, 700);
  chassis.waitUntilDone();
  armTarget = 30000;
  pros::delay(1250); // scoring neutral stake
  armTarget = 21000;

  chassis.moveToPoint(49, 66, 1200, {.forwards = false});
  chassis.turnToHeading(180, 800); // aim for bottom 2 rings
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(49, 7, 2000, {.maxSpeed = 90});  // intakes 2 rings
  chassis.turnToHeading(60, 800);                      // aim for last ring
  chassis.moveToPoint(60, 15, 2000, {.maxSpeed = 50}); // intakes last ring
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
  goalClamp.set_value(false); // put goal down
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(61, 15, 2000);
  chassis.turnToHeading(90, 1200);
  chassis.moveToPose(
      -22, 15, 90, 2000,
      {.forwards = false, .maxSpeed = 90}); // back at to 2nd goal
  chassis.waitUntilDone();
  goalClamp.set_value(true); // clamps 2nd goal
  pros::delay(100);
  // armTarget = 12000;
  chassis.turnToHeading(-2, 1000);
  chassis.moveToPoint(-23, 35, 1200);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.turnToHeading(-90, 1000);
  chassis.moveToPoint(-45, 37.2, 1200);
  chassis.turnToHeading(-30, 1000);
  chassis.moveToPoint(-57, 60, 1200);
  chassis.turnToHeading(13, 1000);
  chassis.moveToPoint(-48, 87, 1200);
  chassis.waitUntil(9.5);
  armTarget = 16250;
  chassis.waitUntilDone();
  armTarget = 16250;
  pros::delay(200);
  chassis.moveToPoint(-57, 68, 1200, {.forwards = false});
  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(-69, 68, 700);
  chassis.waitUntilDone();
  armTarget = 30000;
  pros::delay(1250); // scoring neutral stake
  armTarget = 21000;

  chassis.moveToPoint(-49, 68, 900, {.forwards = false});
  chassis.turnToHeading(180, 800); // aim for bottom 2 rings
  chassis.waitUntilDone();
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(-49.5, 6, 2000, {.maxSpeed = 90}); // intakes 2 rings
  chassis.turnToHeading(-60, 800);                       // aim for last ring
  chassis.moveToPoint(-60, 15, 2000, {.maxSpeed = 50});  // intakes last ring
  chassis.turnToHeading(39, 800);
  chassis.waitUntil(1);
  intake.move_velocity(5000);
  conveyor.move_velocity(5000);
  chassis.waitUntil(2);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.moveToPoint(-63, 5, 2000, {.forwards = false});
  chassis.waitUntil(1);
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.waitUntilDone();
  intake.move_velocity(1000);
  conveyor.move_velocity(1000);
  goalClamp.set_value(false);
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPose(-20.7, 82.6, 51, 2000, {.minSpeed = 95});
  chassis.waitUntil(1);
  intake.move_velocity(-12000);
  chassis.waitUntilDone();
  pros::delay(300);
  chassis.turnToHeading(-145, 1000);
  chassis.moveToPoint(6, 106, 1200, {.forwards = false, .maxSpeed = 60});
  chassis.waitUntilDone();
  goalClamp.set_value(true);
  pros::delay(100);
  intake.move_velocity(-12000);
  conveyor.move_velocity(-12000);
  chassis.turnToHeading(-90, 1000);
  chassis.moveToPoint(-40, 106.5, 1400, {.maxSpeed = 90});
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.turnToHeading(135, 1000);
  chassis.waitUntilDone();
  goalClamp.set_value(false);
  chassis.moveToPoint(-68, 130, 700, {.forwards = false});
  chassis.moveToPose(29, 86, 115, 2500, {.maxSpeed = 95});
  chassis.waitUntil(1);
  armTarget = 16250;
  chassis.waitUntilDone();
  pros::delay(800);
  chassis.moveToPoint(7.5, 117, 1500, {.forwards = false, .maxSpeed = 90});
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.turnToHeading(0, 1400, {.maxSpeed = 90});
  chassis.waitUntilDone();
  armTarget = 24000;
  pros::delay(500);
  armTarget = 34000;
  pros::delay(500);
  chassis.moveToPoint(7.5, 108, 1500, {.forwards = false, .minSpeed = 110});
  chassis.waitUntilDone();
  pros::delay(500);
  armTarget = 20000;
  chassis.turnToHeading(50, 1000, {.maxSpeed = 110});
  chassis.moveToPoint(70, 135, 1500, {.minSpeed = 110});

  pros::delay(3000);
  intake.move_velocity(0);
  conveyor.move_velocity(0);
}

// void bluePositiveAWP () {
// 	chassis.setPose(57, -11.75, 180);
// 	chassis.moveToPose(57, 2, 180, 1000, {.forwards = false});
// 	chassis.turnToHeading(270, 1000);
// 	chassis.moveToPoint(60.5, 2, 1000, {.forwards = false, .maxSpeed = 30});
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(-12000);
// 	pros::delay(1000);
// 	chassis.turnToPoint(53, 2, 1000);
// 	chassis.moveToPoint(53, 2, 1000, {.maxSpeed = 30});
// 	chassis.waitUntilDone();
// 	intakeRaise.set_value(true);
// 	conveyor.move_voltage(0);
// 	intake.move_voltage(-12000);
// 	pros::delay(500);
// 	chassis.turnToPoint(25, -23, 1000, {.forwards = false, .maxSpeed = 90});
// 	chassis.moveToPoint(25, -23, 1000, {.forwards = false, .maxSpeed = 90});
// 	chassis.waitUntilDone();
// 	pros::delay(700);
// 	goalClamp.set_value(true);
// 	conveyor.move_velocity(6000);
// 	chassis.turnToPoint(23, -45, 1000);
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(23, -45, 1000);
// 	chassis.waitUntilDone();
// 	chassis.turnToPoint(0, -25, 1000);
// 	chassis.waitUntilDone();
// 	conveyor.move_velocity(6000);
// 	pros::delay(500);
// 	conveyor.move_velocity(-12000);
// 	chassis.moveToPoint(10.5, -33.5, 1000);
// 	chassis.waitUntilDone();
// 	intake.move_voltage(0);
// 	chassis.moveToPoint(0, -25, 3000, {.maxSpeed = 40});
// }

// void redPositiveGoalRushQuals() {
// 	chassis.setPose(-50, -62.25, 270);
// 	chassis.moveToPose(-27, -62.26, 270, 1000, {.forwards = false, .minSpeed
// = 90});
// 	// chassis.turnToHeading(240, 1000);
// 	chassis.turnToPoint(-5.5, -53, 500, {.forwards = false});
// 	chassis.moveToPose(-5.5, -53, 240, 1000, {.forwards = false, .minSpeed =
// 50}); 	chassis.waitUntilDone(); 	pros::delay(500);
// goalClamp.set_value(true); 	intakeRaise.set_value(true); pros::delay(200);
// 	chassis.moveToPose(-18, -48, 285, 1000);
// 	intake.move_voltage(-12000);
// 	conveyor.move_voltage(-12000);
// 	chassis.waitUntilDone();
// 	pros::delay(250);
// 	intake.move_voltage(0);
// 	conveyor.move_voltage(0);
// 	goalClamp.set_value(false);
// 	chassis.moveToPose(-23, -48, 270, 1000);
// 	// chassis.turnToHeading(180, 1000);
// 	// chassis.turnToPoint(-18, -32, 1000, {.forwards = false});
// 	chassis.turnToPoint(-23, -30, 1000, {.forwards = false});
// 	chassis.moveToPoint(-23, -30, 1000, {.forwards = false, .minSpeed =
// 50}); 	chassis.waitUntilDone(); 	pros::delay(500);
// goalClamp.set_value(true); 	pros::delay(200);
// intake.move_voltage(-12000); 	conveyor.move_voltage(-12000);
// 	chassis.moveToPose(-27, -62.25, 270, 1000);
// 	chassis.moveToPoint(-60, -63, 2500);
// 	chassis.turnToHeading(180, 1500);
// 	chassis.moveToPoint(-49, -32, 1000, {.forwards = false});
// 	chassis.turnToHeading(270, 1000);
// 	chassis.moveToPoint(-25, -32, 1000, {.forwards = false});
// 	// chassis.turnToHeading(88, 1000);
// 	// chassis.waitUntilDone();
// 	// intake.move_voltage(0);
// 	// chassis.moveToPoint(0, -20, 5000);
// 	// chassis.waitUntilDone();
// 	// intake.move_voltage(0);
// 	// chassis.turnToHeading(60, 1000);
// 	// chassis.moveToPoint(-4.5, -25, 1000);

// }

// void bluePositiveGoalRushQuals() {
// 	chassis.setPose(50, -62.25, 90);
// 	chassis.moveToPose(27, -62.26, 90, 1000, {.forwards = false, .minSpeed =
// 90});
// 	// chassis.turnToHeading(240, 1000);
// 	chassis.turnToPoint(8, -53, 500, {.forwards = false});
// 	chassis.moveToPose(8, -53, 120, 1000, {.forwards = false, .minSpeed =
// 50}); 	chassis.waitUntilDone(); 	pros::delay(500);
// goalClamp.set_value(true); 	intakeRaise.set_value(true); pros::delay(200);
// 	chassis.moveToPose(18, -48, 75, 1000);
// 	intake.move_voltage(-12000);
// 	conveyor.move_voltage(-12000);
// 	chassis.waitUntilDone();
// 	pros::delay(500);
// 	intake.move_voltage(0);
// 	goalClamp.set_value(false);
// 	chassis.moveToPose(23.5, -48, 90, 1000);
// 	// chassis.turnToHeading(180, 1000);
// 	// chassis.turnToPoint(-18, -32, 1000, {.forwards = false});
// 	// chassis.turnToHeading(180, 1000);
// 	chassis.turnToPoint(23.5, -30, 1000, {.forwards = false});
// 	chassis.moveToPose(23.5, -30, 180, 1000, {.forwards = false, .minSpeed =
// 50}); 	chassis.waitUntilDone(); 	pros::delay(500);
// goalClamp.set_value(true); 	pros::delay(200);
// intake.move_voltage(12000); 	chassis.moveToPose(27, -62.25, 90, 1000);
// 	chassis.moveToPoint(63, -63, 1500);
// 	chassis.turnToHeading(180, 1500);
// 	chassis.moveToPoint(49, -32, 1000, {.forwards = false});
// 	chassis.turnToHeading(272, 1000);
// 	chassis.waitUntilDone();
// 	intake.move_voltage(0);
// 	conveyor.move_velocity(0);
// 	chassis.turnToHeading(90, 1000);
// 	chassis.moveToPoint(15, -32, 1000, {.forwards = false});
// 	// intakeRaise.set_value(false);
// 	// chassis.moveToPoint(0, -20, 5000, {.maxSpeed = 100});
// 	// chassis.waitUntilDone();
// 	// intakeRaise.set_value(true);
// 	// chassis.waitUntilDone();
// 	// intake.move_voltage(0);
// 	// chassis.turnToHeading(60, 1000);
// 	// chassis.moveToPoint(-4.5, -25, 1000);

// }

void autonomous() {
  if (color) {
    if (current == 0) {
      blueNegativeQuals();
    } else if (current == 1) {
      bluePositiveSoloAWP();
    } else {
      blueRushAlliance();
    }
  } else {
    if (current == 0) {
      redNegativeQuals();
    } else if (current == 1) {
      redPositiveSoloAWP();
    } else {
      redRushAlliance();
    }
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
  autoStarted = false;
  bool isExtended1 = true; // remove for DRIVER SKILLS
  bool isExtended2 = false;
  bool isExtended3 = false;
  bool isExtended4 = false;
  bool taskDisable1 = false;
  bool taskDisable2 = false;
  bool taskDisable3 = false;
  arm.set_brake_mode(pros::MotorBrake::hold);
  // //BELOW FOR DRIVER SKILLS//
  // intakeRaise.set_value(true);
  // bool isExtended1 = false;
  while (true) {
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    /////////////////////////////////////////////////////
    // ARM CONTROL//
    /////////////////////////////////////////////////////

    // if(rightY > 10){
    //     armMacro = false;
    //     if((armRotation.get_angle() < 1000) || (armRotation.get_angle() >
    //     3000)){
    //         arm.move(rightY * 120);
    // 	} else {
    // 		arm.move(0);
    // 	}
    // }

    // if(rightY < -10){
    //     armMacro = false;
    //     arm.move(rightY * 120);

    // }

    // if (!armMacro && std::abs(rightY) < 10){
    //  	arm.move(0);

    // }

    if (std::abs(rightY) > 10) {
      armMacro = false;
    }
    if (armMacro == false) {
      // if ((armRotation.get_angle() < 27500) || (armRotation.get_angle() >
      // 10000)) {
      if (rightY > 10) {
        armMacro = false;
        arm.move(rightY);
      } else if (rightY < 10) {
        armMacro = false;
        arm.move(rightY);
      }
      // }
      // if (((armRotation.get_angle() > 27500))) {
      // 	if (rightY > 10) {
      // 		arm.move(0);
      // 	}
      // 	else if (rightY < 10) {
      // 		armMacro = false;
      // 		arm.move(rightY*120);
      // 	}
      // }
    }

    if (!armMacro && std::abs(rightY) < 10) {
      arm.move(0);
    }
    if (rightX > 70) {
      armMacro = true;
      armTarget = 16200;
    }

    // if (rightX < -70) {
    // 	armMacro = true;
    // 	armTarget = 28000;
    // }

    // if (digitalA) {
    // 	armMacro = true;
    // 	armTarget = 32000;
    // }

    // if (rightX < -70) {
    // 	armMacro = true;
    // 	armTarget = 30000;
    // }

    if (armMacro) {
      error = angleWrap(armTarget - armRotation.get_angle()); // 7250 = target
      derivative = (error - previous_error);
      if (fabs(error) < 2 || fabs(error + derivative) < 2) {
        arm.move_voltage(0);
        break;
      }
      if (sign(error) != sign(previous_error)) {
        integral += error;
      } else {
        integral = 0;
      }
      arm.move_voltage(error * armkP + integral * armkI + derivative * armkD);
      previous_error = error;
    }

    // if (digitalY) {
    // 	taskDisable2 = true;
    // 	taskDisable3 = true;
    // 	taskDisable1 = false;
    //     double integral = 0;
    //     double error = 100;
    //     double previous_error = 0;
    //     pros::Task task{[&] {
    // 		while (true && taskDisable1 == false) {
    // 			error = angleWrap(7250 - armRotation.get_angle());
    // //7250 = target 			derivative = (error - previous_error);
    // if(fabs(error) < 2
    // || fabs(error+derivative) < 2) {
    // arm.move_voltage(0); 			break;
    // 			}
    // 			if (sign(error) != sign(previous_error)) {
    // 			integral += error;
    // 			} else {
    // 			integral = 0;
    // 			}
    // 			arm.move_voltage(error*armkP +integral * armkI +
    // derivative * armkD); 			previous_error = error;
    // pros::delay(10);
    // 			}
    // 		}
    // 	};
    // }

    // if (digitalRight) {
    // 	taskDisable1 = true;
    // 	taskDisable3 = true;
    // 	taskDisable2 = false;
    // 	pros::Task task2{[&] {
    //         double integral = 0;
    //         double error = 100;
    //         double previous_error = 0;
    // 		while (true && taskDisable2 == false) {
    // 			error = angleWrap(11000 - armRotation.get_angle());
    // //7700 = target 			derivative = (error - previous_error);
    // if(fabs(error) < 2
    // || fabs(error+derivative) < 2) {
    // arm.move_voltage(0); 			break;
    // 			}
    // 			if (sign(error) != sign(previous_error)) {
    // 			integral += error;
    // 			} else {
    // 			integral = 0;
    // 			}
    // 			arm.move_voltage(error*armkP +integral * armkI +
    // derivative * armkD); 			previous_error = error;
    // pros::delay(10);
    // 			}
    // 		}
    // 	};
    // }

    // if (digitalB) {
    // 	taskDisable1 = true;
    // 	taskDisable3 = false;
    // 	taskDisable2 = true;
    // 	pros::Task task3{[&] {
    //         double integral = 0;
    //         double error = 100;
    //         double previous_error = 0;
    // 		while (true && taskDisable3 == false) {
    // 			error = angleWrap(33700 - armRotation.get_angle());
    // //7700 = target 			derivative = (error - previous_error);
    // if(fabs(error) < 2
    // || fabs(error+derivative) < 2) {
    // arm.move_voltage(0); 			break;
    // 			}
    // 			if (sign(error) != sign(previous_error)) {
    // 			integral += error;
    // 			} else {
    // 			integral = 0;
    // 			}
    // 			arm.move_voltage(error*armkP +integral * armkI +
    // derivative * armkD); 			previous_error = error;
    // pros::delay(10);
    // 			}
    // 		}
    // 	};
    // }
    // error = angleWrap(target - armRotation.get_angle());
    // if(sign(error) != sign(previous_error)) {
    // 	integral = 0;
    // } else {
    // 	integral += error;
    // }

    // armVoltage = (error * armkP + integral * armkI + (error - previous_error)
    // * armkD); arm.move_voltage(armVoltage); previous_error = error;

    chassis.arcade(leftY, leftX);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_voltage(12000);
      conveyor.move_voltage(12000);

    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      // ejectorIntake(forward, no_colour);
      intake.move_voltage(12000);
      conveyor.move_voltage(-12000);
      // ejectorIntake();
    } else {
      intake.move_voltage(0);
      conveyor.move_voltage(0);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      isExtended2 = !isExtended2;
      doinker.set_value(isExtended2);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      isExtended1 = !isExtended1;
      goalClamp.set_value(isExtended1);
    }

    if (rightY != 0) {
      taskDisable1 = true;
      taskDisable2 = true;
      taskDisable3 = true;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      isExtended3 = !isExtended3;
      intakeRaise.set_value(isExtended3);
    }

    // if (digitalA) {
    // 	isExtended4 = !isExtended4;
    // 	hang.set_value(isExtended4);
    // }
    // arm.move_voltage(rightY * 120);
    pros::delay(25); // Run for 20 ms then update
  }
}