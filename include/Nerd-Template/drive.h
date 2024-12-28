#pragma once
#include "vex.h"
//#include "config-structs.h"
#include "odom.h"

class Odom;

enum drive_setup: int {
  ZERO_TRACKER = 0, 
  TANK_IME = 1, 
  TANK_ONE_ENCODER = 2, 
  TANK_ONE_ROTATION = 4, 
  TANK_TWO_ENCODER = 8, 
  TANK_TWO_ROTATION = 16, 
  HOLONOMIC_TWO_ENCODER = 32,
  HOLONOMIC_TWO_ROTATION = 64
};

class Drive
{
private:
  float wheel_diameter;
  float wheel_ratio;
  float gyro_scale;
  float drive_in_to_deg_ratio;
  float max_velocity_in_per_sec;
  float ForwardTracker_center_distance;
  float ForwardTracker_diameter;
  float ForwardTracker_in_to_deg_ratio;
  float SidewaysTracker_center_distance;
  float SidewaysTracker_diameter;
  float SidewaysTracker_in_to_deg_ratio;
  vex::triport ThreeWire = vex::triport(vex::PORT22);

public: 
  drive_setup drive_setup = ZERO_TRACKER;
  motor_group DriveL;
  motor_group DriveR;
  inertial Gyro;
  motor DriveLF;
  motor DriveRF;
  motor DriveLB;
  motor DriveRB;
  rotation R_ForwardTracker;
  rotation R_SidewaysTracker;
  encoder E_ForwardTracker;
  encoder E_SidewaysTracker;

  float motor_rpm;
  float drive_rpm;

  FollowConfig follow_config;
  DriveToPointConfig drive_to_point_config;
  VelocityControllerConfig velocity_controller_config;
  
  float follow_mu;
  float follow_min_ld;
  float follow_max_ld;
  float follow_ld;

  float follow_max_acceleration;
  float follow_max_voltage;

  float follow_heading_error_turn_threshold;
  float follow_settle_error;
  float follow_settle_time;
  float follow_timeout;
  
  float desired_heading;
  float width;

  bool stop_auton = false;

  Drive(
    enum::drive_setup drive_setup, 
    motor_group DriveL, 
    motor_group DriveR, 
    int gyro_port, 
    float wheel_diameter, 
    float motor_rpm, 
    float drive_rpm, 
    float gyro_scale, 
    int DriveLF_port, int DriveRF_port, 
    int DriveLB_port, int DriveRB_port, 
    int ForwardTracker_port, 
    float ForwardTracker_diameter, 
    float ForwardTracker_center_distance, 
    int SidewaysTracker_port, 
    float SidewaysTracker_diameter, 
    float SidewaysTracker_center_distance,
    float width);

  void drive_with_voltage(float leftVoltage, float rightVoltage);
  void drive_keep_turn_rate(float leftVoltage, float rightVoltage);

  float get_velocity_volts();
  float get_left_velocity_volts();
  float get_right_velocity_volts();

  float get_absolute_heading();

  float get_left_position_in();

  float get_right_position_in();

  float get_current();
  float get_holonomic_current();

  void set_follow_constants(float follow_max_voltage, float follow_max_acceleration, float follow_max_applicable_curvature_radius, float follow_min_ld, float follow_max_ld, float follow_k_ld);
  void set_follow_exit_conditions(float follow_settle_error, float follow_settle_time, float follow_timeout);

  void set_brake_type(vex::brakeType mode);

  void turn_to_angle(float angle, const TurnConfig& config = {});
  void drive_time(float time, float left_voltage, float right_voltage, bool stop = true);

  void drive_distance(float distance, const DriveDistanceConfig& config = {});
  void drive_distance(float distance, float heading, const DriveDistanceConfig& config = {});

  void stop(brakeType mode);
  void left_swing_to_angle(float angle, const SwingConfig& config = {});
  void right_swing_to_angle(float angle, const SwingConfig& config = {});
  
  Odom odom;
  float get_ForwardTracker_position();
  float get_SidewaysTracker_position();
  void set_coordinates(float X_position, float Y_position, float orientation_deg);
  void position_track();
  static int position_track_task();
  vex::task async_task;
  float get_X_position() const;
  float get_Y_position() const;
  Vector2 get_position() const;
  Pose get_pose();

  float get_max_velocity() const;

  void drive_to_point(const Vector2& pos, const DriveToPointConfig& config = {});
  
  /**
   * @brief Drive to pose (x, y, θ) using boomerang.
   * @param pose: target pose
   * @param config: motion configuration:
   *    - direction: forward, reverse, or flexible
   *    - lead_distance: Interpolation parameter to determine how far between the end point and the robot's position to put the carrot point/look ahead point. 1 means basically at the robot, 0 means at the end point.
   *    - drive_pid: drive PID configuration
   *    - drive_max_voltage: maximum voltage use for driving
   *    - heading_pid: heading PID configuration
   *    - heading_max_voltage: maximum voltage used to control heading
   *    - settle_conditions: settle error, settle time, timeout, and max_current.
  */
  void drive_to_pose(Pose pose, const DriveToPoseConfig& config = ::DRIVE_TO_POSE_BOOMERANG_DEFAULT);

  void follow(const std::vector<Vector2>& path);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth = true);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time);
  void follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time, float follow_mu, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  void follow(const std::vector<Waypoint>& path, RAMSETEConfig config = {}, const VelocityControllerConfig& velocity_controller_config = DRIVE_VELOCITY_DEFAULT); // follow with RAMSETE

  //void follow(const std::vector<Vector2>& path);
  void follow(const std::vector<Vector2>& path, const FollowConfig& config);
  
  void turn_to_point(const Vector2& pos, const TurnConfig& config = {});
  void turn_to_point(const Vector2& pos, float extra_angle_deg, const TurnConfig& config = {});
  
  void holonomic_drive_to_point(float X_position, float Y_position);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time);
  void holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);

  void holonomic_drive_to_point(const Vector2& pos, const HolonomicDriveToPointConfig& config);
  void holonomic_drive_to_point(const Pose& pose, const HolonomicDriveToPointConfig& config);

  void control_arcade(float tFwd = 5, float tTurn = 10);
  void control_tank(float t = 10);
};