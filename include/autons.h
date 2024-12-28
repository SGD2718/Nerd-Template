#pragma once
#include "Nerd-Template/drive.h"
#include "lady-brown.h"
#include "intake.h"
#include "mogo-mech.h"
#include <functional>

class Drive;
class LadyBrown;
class Intake;
class MogoClamp;

extern Drive chassis;
extern LadyBrown lady_brown;
extern MogoClamp mogo_clamp;
extern Intake intake;

void default_constants();

void find_tracking_center(float turnVoltage = 6, uint32_t time = 5000);
void blue_positive();
void red_positive();
void red_negative();
void blue_negative();
/*void positive_blue();
void positive_red();
void negative_blue();
void negative_red();*/

void pure_pursuit_test();
void goal_side_alliance_triball();
void near_side();
void far_side();
void skills();
void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();





struct Auton {
  Auton(std::string name, std::function<void(void)> auton_function, vex::color display_color);
  Auton(std::string name, std::function<void(void)> auton_function);

  std::string name;
  std::function<void(void)> auton_function;
  vex::color display_color;
};