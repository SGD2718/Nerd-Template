#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;


// VEXcode device constructors
controller Controller1 = controller(primary);

motor DriveL1 = motor(PORT4, gearSetting::ratio6_1, true);
motor DriveL2 = motor(PORT6, gearSetting::ratio6_1, true);
motor DriveL3 = motor(PORT9, gearSetting::ratio6_1, true);

motor DriveR1 = motor(PORT1, gearSetting::ratio6_1, false);
motor DriveR3 = motor(PORT3, gearSetting::ratio6_1, false);
motor DriveR2 = motor(PORT5, gearSetting::ratio6_1, false);

motor_group DriveMotorsL(DriveL1, DriveL2, DriveL3);
motor_group DriveMotorsR(DriveR1, DriveR2, DriveR3);

motor IntakeMotor = motor(PORT14, gearSetting::ratio6_1, true);
motor LadyBrownMotor = motor(PORT17, gearSetting::ratio18_1, true);

digital_out MogoPiston(Brain.ThreeWirePort.A);
digital_out IntakePiston(Brain.ThreeWirePort.H);
digital_out HangPiston(Brain.ThreeWirePort.B);
digital_out DoinkerPiston(Brain.ThreeWirePort.G);

rotation LBRotation(PORT20);
optical ColorSensor(PORT8);
distance MogoSensor(PORT16);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}