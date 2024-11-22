#include "lemlibglobals.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"


// MOTORS //
pros::MotorGroup left_motors({-17, 18, -8}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({19, -10, 7 }, pros::MotorGearset::blue);

pros::Motor conveyor(6, pros::MotorGearset::blue);
pros::Motor intake(9, pros::MotorGearset::blue);
pros::Motor arm (20, pros::MotorGearset::red);
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 10.2, lemlib::Omniwheel::NEW_275, 450, 2);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              9.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis

// imu
pros::Imu imu(11);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(14);
pros::Rotation vertical_encoder(-12);
// vertical tracking wheel encoder
// pros::adi::Encoder vertical_encoder('E', 'F', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2,1.25);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2,0);
// // vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

