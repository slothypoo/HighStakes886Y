#include "main.h"
#include "autons.hpp"
#include "globals.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlibglobals.hpp"
#include "liblvgl/llemu.hpp"
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
  pros::lcd::register_btn0_cb(on_left_button);
  pros::lcd::register_btn1_cb(on_center_button);
  chassis.calibrate(); // calibrate sensors
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
      error = angleWrapOneDirection(armTarget, armRotation.get_angle(), -1); // 7250 = target
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
  armTarget = 15800;
  chassis.waitUntilDone();
  pros::delay(350);
  chassis.moveToPoint(57, 66, 1200,{.forwards = false}); // moving back to neutral stake
  chassis.turnToHeading(90, 800);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(80, 66, 1200);
  chassis.waitUntil(3);
  armTarget = 30000;
  // scoring neutral stake
  chassis.moveToPoint(49, 66, 1200, {.forwards = false});
  chassis.waitUntilDone();
  armTarget = 12000;
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
  armTarget = 15800;
  chassis.waitUntilDone();
  armTarget = 15800;
  pros::delay(200);
  chassis.moveToPoint(-57, 71, 1200, {.forwards = false});
  chassis.turnToHeading(-90, 1000);
  chassis.waitUntilDone();
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.moveToPoint(-80, 71, 1200);
  chassis.waitUntil(3);
  armTarget = 30000;
  chassis.moveToPoint(-49, 68, 900, {.forwards = false});
  chassis.waitUntilDone();
  armTarget = 12000;
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
  pros::delay(100);
  goalClamp.set_value(false);
  chassis.moveToPoint(-68, 130, 700, {.forwards = false});
  chassis.moveToPose(29, 84.5, 116, 2500, {.maxSpeed = 95});
  chassis.waitUntil(1);
  armTarget = 16250;
  chassis.waitUntilDone();
  pros::delay(800);
  chassis.moveToPoint(7.5, 117, 1500, {.forwards = false, .maxSpeed = 90});

  chassis.turnToHeading(0, 1400, {.maxSpeed = 90});
  chassis.moveToPoint(6.5, 130, 800, {.maxSpeed = 60});
  chassis.moveToPoint(6.5, 114.4, 800, {.forwards = false, .maxSpeed = 60});
  intake.move_velocity(0);
  conveyor.move_velocity(0);
  chassis.waitUntilDone();
  armTarget = 34000;
  pros::delay(600);
  chassis.moveToPoint(7.5, 110, 1500, {.forwards = false, .minSpeed = 110});
  chassis.waitUntilDone();
  pros::delay(500);
  armTarget = 20000;
  chassis.turnToHeading(50, 1000, {.maxSpeed = 110});
  chassis.moveToPoint(70, 140, 1500, {.minSpeed = 110});
  

  pros::delay(3000);
  intake.move_velocity(0);
  conveyor.move_velocity(0);
}

void autonomous() {

  skills();
  /*
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
  */
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
  // bool isExtended1 = true; // remove for DRIVER SKILLS
  bool isExtended2 = false;
  bool isExtended3 = false;
  bool isExtended4 = false;
  bool taskDisable1 = false;
  bool taskDisable2 = false;
  bool taskDisable3 = false;
  arm.set_brake_mode(pros::MotorBrake::hold);
  // //BELOW FOR DRIVER SKILLS//
  intakeRaise.set_value(true);
  bool isExtended1 = false;
  while (true) {
    int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    /////////////////////////////////////////////////////
    // ARM CONTROL//
    /////////////////////////////////////////////////////
    if (rightY > 30) {
      armMacro = false;
      arm.move(rightY);
      std::cout << "rightY > 30" << std::endl;
    } else if (rightY < -30) {
      armMacro = false;
      arm.move(rightY);
      std::cout << "rightY < -30" << std::endl;
    }

    if (!armMacro && std::abs(rightY) < 30) {
      arm.move(0);
      std::cout<<"!armMAcro"<< std::endl;
    }
    if (rightX > 70) {
      //loading
      armMacro = true;
      armTarget = 15800;
      std::cout <<"rightX > 70" << std::endl;
    }

    if (rightX < -70) {
      //alliance stake lineup
      armMacro = true;
      armTarget = 30000;
      std::cout << "rightX < -70" << std::endl;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      //goal flip
      armMacro = true;
      armTarget = 36000;
      std::cout << "digitalA" << std::endl;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      //descore
      armMacro = true;
      armTarget = 28000;
      std::cout << "digitalY" << std::endl;
    }
    if (armMacro) {
      std::cout << "armMacro running" << std::endl;
      error = angleWrapOneDirection(armTarget, armRotation.get_angle(),
                                    -1); // 7250 = target
      derivative = (error - previous_error);
      if (fabs(error) < 2 || fabs(error + derivative) < 2) {
        arm.move_voltage(0);
        break;
        std::cout << "fabs error < 2" << std::endl;
      }
      if (sign(error) != sign(previous_error)) {
        integral += error;
        std::cout << "integral += error" << std::endl;
      } else {
        integral = 0;
        std::cout << "integral = 0" << std::endl;
      }
      arm.move_voltage(error * armkP + integral * armkI + derivative * armkD);
      previous_error = error;
    }

    chassis.arcade(leftY, leftX);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_voltage(12000);
      conveyor.move_voltage(12000);

    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      // ejectorIntake(forward, no_colour);
      intake.move_voltage(-12000);
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