#include "vex.h"
#include <iostream>
#include <sstream>

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]         
// Controller1          controller                      
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors using the V5 port icon in the top right of the screen. Doing     */
/*  so will update robot-config.cpp and robot-config.h automatically, so     */
/*  you don't have to.                                                       */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             Nerd-Template Config                          */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your robot manually with the sidebar configurer. */
/*---------------------------------------------------------------------------*/


Drive chassis(

//Specify your drive setup below. There are seven options:
//ZERO_TRACKER, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
//TANK_ONE_ENCODER and TANK_ONE_ROTATION assumes that you have a sideways tracking wheel and will use right drive base motor encoders for forward tracking.
//For example, if you are not using odometry, put ZERO_TRACKER below:
TANK_ONE_ENCODER,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(DriveL1, DriveL2, DriveL3),

//Right Motors:
motor_group(DriveR1, DriveR2, DriveR3),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT11,

// Input your wheel diameter. (4" omnis are actually closer to 4.125"):
2.75,

// Input the maximum rpm of your drive motors. Blue = 600, green = 200, red = 100.
600.,

// Input the maximum rpm of your drive train wheels. If you looked up your gearing from a catelogue, it should've told you what rpm you chose.
450.,

// Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
// NOTE: If, for some reason, a full clockwise rotation is not -360°, then all of 
// Nerd Template's auton functions will absolutely break. Modify at your own risk.
-360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT1,

//LB:      //RB: 
PORT1,     -PORT1,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, leave it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1"
PORT13,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//This distance is in inches:
0,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT7,

//Sideways tracker diameter (reverse to make the direction switch):
-2.125,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
0.112844587982,

// drive width (distance between centers of opposite wheels)
10.988
);

LadyBrown lady_brown(&LadyBrownMotor, &LBRotation);
MogoClamp mogo_clamp(&MogoPiston, &MogoSensor);
Intake intake(&IntakeMotor, &ColorSensor, &IntakePiston);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

// fill this list with your auton functions in the format {"auton name", function_name, display_color [OPTIONAL] }
// you declare these in "autons.h" and implement them in "autons.cpp"
std::vector<Auton> autons = {
  {"red-positive", red_positive, vex::red},
  {"red-negative", red_negative, vex::purple},
  {"blue-positive", blue_positive, vex::blue},
  {"blue-negative", blue_negative, vex::green}
}; 

int current_auton_selection = 1;
int prev_auton_selection = -1;
bool auto_started = false;

int auton_selector_task() {
  bool was_pressing = false;
  while(true) {
    Brain.Screen.clearScreen();
    if (current_auton_selection != prev_auton_selection) {
      Brain.Screen.setFillColor(autons[current_auton_selection].display_color);
      Brain.Screen.setPenColor(white);
      Brain.Screen.printAt(50, 50, autons[current_auton_selection].name.c_str());
    }

    if(Brain.Screen.pressing() && !was_pressing) {
      if (++current_auton_selection == autons.size()) {
        current_auton_selection = 0;
      }
    }
    was_pressing = Brain.Screen.pressing();
    task::sleep(10);
  }
  return 0;
}

int subsystem_task() {
  while (true) {
    lady_brown.update();
    intake.update();
    task::sleep(10);
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  std::cout << "main boomerang kp = " << DRIVE_TO_POSE_BOOMERANG_DEFAULT.drive_pid.kp << std::endl;
  std::cout << "main drive kp = " << DRIVE_PID_DEFAULT.kp << std::endl;

  if (chassis.Gyro.installed()) {
    chassis.Gyro.calibrate();
    wait(2000, msec);
    while(chassis.Gyro.isCalibrating()) {
     task::sleep(100);
    }
    std::cout << "Gyro Calibrated" <<std::endl;
  }
  auto t1 = task(subsystem_task);
  auto t = task(auton_selector_task);
}

bool auton_running = false;

void autonomous(void) {
  while(chassis.Gyro.isCalibrating())
    task::sleep(100);
  auton_running = true;
  auto_started = true;
  std::cout << "running auton" << std::endl;
  //chassis.set_coordinates(0, 0, 0);
  //chassis.drive_to_point({36, 24}, DriveToPointConfig().set_is_rigid(true));
  //chassis.drive_to_point({0, 0}, DriveToPointConfig().set_direction(REVERSE));
  chassis.drive_distance(24, DriveDistanceConfig().set_settle_error(0.02));
  task::sleep(500);
  chassis.drive_distance(-24, 20, DriveDistanceConfig().set_settle_error(0.02));
  std::cout << "auton done" << std::endl;
  auton_running = false;
  // Path
  // Path
  return;
  chassis.follow({
    Waypoint(0, 0, 10),
    Waypoint(-0.597, 0.344, 10),
    Waypoint(40.353, 22.083, 10),
    Waypoint(30.242, 44.832, 10),
  }, RAMSETEConfig());


  //find_tracking_center(3, 5000);
  //blue_positive();
  //red_negative();
  //blue_negative();
  //autons[current_auton_selection].auton_function();
  //chassis.drive_to_point(0, 36);
  //chassis.drive_to_point(0, 0);
  // /chassis.turn_to_angle(45);
  //chassis.boomerang(36,36, 45);
  auton_running = false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  std::cout << "usercontrol started" << std::endl;
  auton_running = false;
  chassis.set_coordinates(0,0,0);
  

  while(chassis.Gyro.isCalibrating()) {
     wait(100, msec);
  }

  
  intake.calibrate();

  PID alliance_stake_PID(0, 1.5, 0, 0.02, 1);

  Controller1.ButtonA.pressed([]() -> void {intake.set_filter_mode(RED_RINGS);});
  Controller1.ButtonX.pressed([]() -> void {intake.set_filter_mode(BLUE_RINGS);});
  Controller1.ButtonLeft.pressed([]() -> void {intake.set_filter_mode(NO_RINGS);});
  Controller1.ButtonUp.pressed([]() -> void {
    autonomous();
  });

  Controller1.ButtonL2.pressed([]() {
    if (lady_brown.get_state() == LB_IDLE)
      lady_brown.rest();
    else if (lady_brown.get_state() == LB_REST || lady_brown.get_state() == LB_SCORE)
      lady_brown.idle();
  });

  Controller1.ButtonB.pressed([]() -> void {DoinkerPiston.set(!DoinkerPiston.value());});

  //int length = 0;
  while (true) {
    //std::cout << "running some shit " << std::endl;
    if (!auton_running) {
      chassis.set_brake_type(coast);
      // drivebase control
      chassis.control_tank(8); // comment for arcadde control

      // add the rest of the controls here
      //IntakeMotor.spin(fwd, 12 * (Controller1.ButtonR2.pressing() - Controller1.ButtonR1.pressing()), volt);
      intake.set_velocity(12 * (Controller1.ButtonR2.pressing() - Controller1.ButtonR1.pressing()));

      mogo_clamp.update(!Controller1.ButtonDown.pressing(), intake.is_spinning());
      
      intake.set_raised(Controller1.ButtonY.pressing());
      if (Controller1.ButtonL1.pressing()) {
        if (lady_brown.get_state() == LB_REST) {
          if (MogoSensor.isObjectDetected()) {
            auto speed = alliance_stake_PID.compute(3.4 - MogoSensor.objectDistance(inches));
            chassis.drive_with_voltage(speed, speed);
          }
        } else
          lady_brown.score();
      }
      lady_brown.update();
    }

    if(is_print_frame())
      if (chassis.get_current() > 3)
        std::cout << "current = " << chassis.get_current() << std::endl;
  
    task::sleep(10); // Sleep the task for a short amount of time to prevent wasted resources.
  }
} 

//
// Main will set up the competition functions and callbacks.
//
int main() {
  pre_auton();

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  //int length = 0;
  while (true) {
    wait(100, msec);
  }
}
