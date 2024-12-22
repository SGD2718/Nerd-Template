using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;

extern motor DriveL1;
extern motor DriveL2;
extern motor DriveL3;

extern motor DriveR1;
extern motor DriveR2;
extern motor DriveR3;

extern motor_group DriveMotorsL;
extern motor_group DriveMotorsR;

extern motor IntakeMotor;
extern motor LadyBrownMotor;

extern digital_out MogoPiston;
extern digital_out IntakePiston;
extern digital_out HangPiston;
extern digital_out DoinkerPiston;

extern rotation LBRotation;
extern optical ColorSensor;
extern distance MogoSensor;



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );