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

// ARM TARGETS //
const int loadingPos = 15800;
const int restingPos = 12000;
const int wallStakePos = 30000;
const int allianceStakePos = 35000;
const int armRaisedPos = 20000;

// ARM PID //
bool armMacro = false;
double armPosition;
double armkP = 2.45;
double armkI = 0;
double armkD = 2.1;
double previous_error = 0;
double target = restingPos;
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

//MO AUTO SELECTOR
//color 
int autonColor = 0;
//path
int path = 0;
//starting pos
int startingPos = 0; 



