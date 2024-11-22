#ifndef LEMLIBGLOBALS_HPP
#define LEMLIBGLOBALS_HPP


#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"


// MOTORS //
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

extern pros::Motor conveyor;
extern pros::Motor intake;
extern pros::Motor arm;
extern lemlib::Drivetrain drivetrain;

// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
extern lemlib::ControllerSettings angular_controller;

// create the chassis

// imu
extern pros::Imu imu;
// horizontal tracking wheel encoder
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;
// vertical tracking wheel encoder
// pros::adi::Encoder vertical_encoder('E', 'F', true);
// horizontal tracking wheel
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;
// // vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
extern lemlib::OdomSensors sensors;

extern lemlib::Chassis chassis;

#endif