#include "globals.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"

// MOTORS //
pros::Rotation armRotation(21);
pros::adi::DigitalOut hang(3);
pros::adi::DigitalOut intakeRaise(4);
pros::adi::DigitalOut doinker(2);
pros::adi::DigitalOut goalClamp(1);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ARM PID //
bool armMacro = false;
double armPosition;
double armkP = 3;
double armkI = 0.000000000000001;
double armkD = 1;
double previous_error = 0;
double target = 12000;
double error;
double integral;
double armVoltage;
int armTarget;
double derivative;

// OTHER AUTO INFO //
bool autoStarted = true;


// AUTO SELECTOR // 
// red = false, blue = true
bool color = false;
// current = 0 = negative, current = 1 = positive current = 2 = rush 
int current =0;
// random other stuff like # of rings 
int misc = 0;
