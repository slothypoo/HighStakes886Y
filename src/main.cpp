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
  skills();
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
    } else if (rightY < -30) {
      armMacro = false;
      arm.move(rightY);
    }

    if (!armMacro && std::abs(rightY) < 30) {
      arm.move(0);
    }
    if (rightX > 70) {
      //loading
      armMacro = true;
      armTarget = 15800;
    }

    if (rightX < -70) {
      //alliance stake lineup
      armMacro = true;
      armTarget = 30000;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      //goal flip
      armMacro = true;
      armTarget = 36000;
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      //descore
      armMacro = true;
      armTarget = 28000;
    }
    if (armMacro) {
      error = angleWrapOneDirection(armTarget, armRotation.get_angle(),
                                    -1); // 7250 = target
      derivative = (error - previous_error);
      if (fabs(error) < 0.5 || fabs(error + derivative) < 0.5) {
        arm.move_voltage(0);
      }
      if (sign(error) != sign(previous_error)) {
        integral += error;
      } else {
        integral = 0;
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