#pragma once

enum Direction {
    FLEXIBLE = 0,
    FORWARD = 1,
    REVERSE = -1,
};

struct SettleConfig;
struct PIDConfig;
struct PIDMotionConfig;
struct VelocityControllerConfig;
struct DriveToPointConfig;
struct DriveToPoseConfig;
struct FollowConfig;
struct RAMSETEConfig;

extern SettleConfig DRIVE_SETTLE_DEFAULT;
extern SettleConfig MOVE_ODOM_SETTLE_DEFAULT;
extern SettleConfig RAMSETE_SETTLE_DEFAULT; // RAMSETE is a bit special since we can calculate exactly how long the movement should take.
extern SettleConfig TURN_SETTLE_DEFAULT;
extern SettleConfig SWING_SETTLE_DEFAULT;

extern PIDMotionConfig DRIVE_PID_DEFAULT;
extern PIDMotionConfig HEADING_PID_DEFAULT;
extern PIDMotionConfig TURN_PID_DEFAULT;
extern PIDMotionConfig SWING_PID_DEFAULT;

extern DriveToPoseConfig DRIVE_TO_POSE_BOOMERANG_DEFAULT;
extern FollowConfig FOLLOW_PP_DEFAULT;
extern RAMSETEConfig FOLLOW_RAMSETE_DEFAULT;
extern VelocityControllerConfig DRIVE_VELOCITY_DEFAULT;

struct SettleConfig {
    float settle_error;
    float settle_time;
    float timeout;
    float max_current;

    SettleConfig& set_settle_error(float e);
    SettleConfig& set_settle_time(float t);
    SettleConfig& set_timeout(float t);
    SettleConfig& set_max_current(float I);
};

struct PIDConfig {
    float kp;
    float ki;
    float kd;
    float integral_range;

    PIDConfig() = default;
    PIDConfig(float kp, float ki, float kd, float integral_range);

    PIDConfig& set_kp(float val);
    PIDConfig& set_ki(float val);
    PIDConfig& set_kd(float val);
    PIDConfig& set_integral_range(float range);
};

struct PIDMotionConfig: public PIDConfig {
    float max_output;

    PIDMotionConfig() = default;
    PIDMotionConfig(float max_output, float kp, float ki, float kd, float integral_range);

    PIDMotionConfig& set_max_output(float val);
    PIDMotionConfig& set_kp(float val);
    PIDMotionConfig& set_ki(float val);
    PIDMotionConfig& set_kd(float val);
    PIDMotionConfig& set_integral_range(float range);
    PIDMotionConfig& operator=(const PIDConfig& pid);
};

struct VelocityControllerConfig {
    float kv;
    float ka;
    float kf;
    float kp;
    float ki;
    float kd;
    float integral_range;

    VelocityControllerConfig& set_kv(float val);
    VelocityControllerConfig& set_ka(float val);
    VelocityControllerConfig& set_kf(float val);
    VelocityControllerConfig& set_kp(float val);
    VelocityControllerConfig& set_ki(float val);
    VelocityControllerConfig& set_kd(float val);
    VelocityControllerConfig& set_integral_range(float range);
};

struct TurnConfig {
    PIDMotionConfig turn_pid = TURN_PID_DEFAULT;
    SettleConfig settle_conditions = TURN_SETTLE_DEFAULT;

    TurnConfig& set_turn_max_voltage(float voltage);
    TurnConfig& set_heading_pid(const PIDConfig& pid);
    TurnConfig& set_settle_conditions(const SettleConfig& settle);
    TurnConfig& set_settle_error(float error);
    TurnConfig& set_settle_time(float time);
    TurnConfig& set_timeout(float time);
    TurnConfig& set_max_current(float I);
};

struct SwingConfig {
    PIDMotionConfig swing_pid = SWING_PID_DEFAULT;
    SettleConfig settle_conditions = SWING_SETTLE_DEFAULT;

    SwingConfig& set_turn_max_voltage(float voltage);
    SwingConfig& set_heading_pid(const PIDConfig& pid);
    SwingConfig& set_settle_conditions(const SettleConfig& settle);
    SwingConfig& set_settle_error(float error);
    SwingConfig& set_settle_time(float time);
    SwingConfig& set_timeout(float time);
    SwingConfig& set_max_current(float I);
};

struct DriveDistanceConfig {
    PIDMotionConfig drive_pid = DRIVE_PID_DEFAULT;
    PIDMotionConfig heading_pid = HEADING_PID_DEFAULT;
    SettleConfig settle_conditions = DRIVE_SETTLE_DEFAULT;

    DriveDistanceConfig& set_drive_max_voltage(float voltage);
    DriveDistanceConfig& set_heading_max_voltage(float voltage);
    DriveDistanceConfig& set_drive_pid(const PIDConfig& pid);
    DriveDistanceConfig& set_heading_pid(const PIDConfig& pid);
    DriveDistanceConfig& set_settle_conditions(const SettleConfig& settle);
    DriveDistanceConfig& set_settle_error(float error);
    DriveDistanceConfig& set_settle_time(float time);
    DriveDistanceConfig& set_timeout(float time);
    DriveDistanceConfig& set_max_current(float I);
};

struct DriveToPointConfig {
    Direction direction = FLEXIBLE;
    bool is_rigid = false;
    PIDMotionConfig drive_pid = DRIVE_PID_DEFAULT;
    PIDMotionConfig heading_pid = HEADING_PID_DEFAULT;
    SettleConfig settle_conditions = MOVE_ODOM_SETTLE_DEFAULT;

    DriveToPointConfig& set_direction(Direction dir);
    DriveToPointConfig& set_drive_max_voltage(float voltage);
    DriveToPointConfig& set_heading_max_voltage(float voltage);
    DriveToPointConfig& set_is_rigid(bool rigid);
    DriveToPointConfig& set_drive_pid(const PIDConfig& pid);
    DriveToPointConfig& set_heading_pid(const PIDConfig& pid);
    DriveToPointConfig& set_settle_conditions(const SettleConfig& settle);
    DriveToPointConfig& set_settle_error(float error);
    DriveToPointConfig& set_settle_time(float time);
    DriveToPointConfig& set_timeout(float time);
    DriveToPointConfig& set_max_current(float I);
};

struct DriveToPoseConfig {
    float lead_distance = 0.6;
    Direction direction = FLEXIBLE;
    PIDMotionConfig drive_pid = DRIVE_PID_DEFAULT;
    PIDMotionConfig heading_pid = HEADING_PID_DEFAULT;
    SettleConfig settle_conditions = MOVE_ODOM_SETTLE_DEFAULT;
    
    static bool default_initialized;

    DriveToPoseConfig();
    DriveToPoseConfig(float lead_distance);

    DriveToPoseConfig& set_direction(Direction dir);
    DriveToPoseConfig& set_lead_distance(float d_lead);
    DriveToPoseConfig& set_drive_max_voltage(float voltage);
    DriveToPoseConfig& set_heading_max_voltage(float voltage);
    DriveToPoseConfig& set_drive_pid(const PIDConfig& pid);
    DriveToPoseConfig& set_heading_pid(const PIDConfig& pid);
    DriveToPoseConfig& set_settle_conditions(const SettleConfig& settle);
    DriveToPoseConfig& set_settle_error(float error);
    DriveToPoseConfig& set_settle_time(float time);
    DriveToPoseConfig& set_timeout(float time);
    DriveToPoseConfig& set_max_current(float I);
};

struct HolonomicDriveToPointConfig {
    PIDMotionConfig drive_pid = DRIVE_PID_DEFAULT;
    PIDMotionConfig turn_pid = TURN_PID_DEFAULT;
    SettleConfig drive_settle_conditions = MOVE_ODOM_SETTLE_DEFAULT;
    SettleConfig turn_settle_conditions = TURN_SETTLE_DEFAULT;

    HolonomicDriveToPointConfig& set_drive_max_voltage(float voltage);
    HolonomicDriveToPointConfig& set_turn_max_voltage(float voltage);
    HolonomicDriveToPointConfig& set_drive_pid(const PIDConfig& pid);
    HolonomicDriveToPointConfig& set_turn_pid(const PIDConfig& pid);
    HolonomicDriveToPointConfig& set_drive_settle_conditions(const SettleConfig& settle);
    HolonomicDriveToPointConfig& set_drive_settle_error(float error);
    HolonomicDriveToPointConfig& set_drive_settle_time(float time);
    HolonomicDriveToPointConfig& set_timeout(float time);
    HolonomicDriveToPointConfig& set_max_current(float I);
    HolonomicDriveToPointConfig& set_turn_settle_conditions(const SettleConfig& settle);
    HolonomicDriveToPointConfig& set_turn_settle_error(float e);
    HolonomicDriveToPointConfig& set_turn_settle_time(float t);
};

struct TrajectoryConfig {
    float max_full_speed_turn_radius = 36.0f;
    float acceleration_distance = 8.0f;
    float max_speed_in_per_sec = 20.625 * M_PI;
    float min_target_velocity = 3.0f;
    float max_target_velocity = 12.0f;
    float dt = 10.0f;

    TrajectoryConfig& set_full_speed_turn_radius(float r);
    TrajectoryConfig& set_deceleration_distance(float d);
    TrajectoryConfig& set_max_speed_in_per_sec(float v);
    TrajectoryConfig& set_max_target_velocity(float v);
    TrajectoryConfig& set_min_target_velocity(float v);
    TrajectoryConfig& set_dt(float dt);
};

struct RAMSETEConfig {
    float b = 2.0f;
    float zeta = 0.7f;

    Direction direction = FLEXIBLE;
    TrajectoryConfig trajectory_config{};
    SettleConfig settle_conditions = RAMSETE_SETTLE_DEFAULT;

    static bool default_initialized;
    
    RAMSETEConfig();
    RAMSETEConfig(float b, float zeta);

    RAMSETEConfig& set_b(float convergence_rate);
    RAMSETEConfig& set_zeta(float damping);
    RAMSETEConfig& set_convergence_rate(float convergence_rate); // set_b but English
    RAMSETEConfig& set_damping(float damping);                   // set_zeta but English
    RAMSETEConfig& set_settle_conditions(const SettleConfig& settle);
    RAMSETEConfig& set_settle_error(float e);
    RAMSETEConfig& set_settle_time(float t);
    RAMSETEConfig& set_timeout(float t);
    RAMSETEConfig& set_max_current(float I);
    RAMSETEConfig& set_trajectory_config(const TrajectoryConfig& config);
    RAMSETEConfig& set_full_speed_turn_radius(float r);
    RAMSETEConfig& set_deceleration_distance(float d);
    RAMSETEConfig& set_max_speed_in_per_sec(float v);
    RAMSETEConfig& set_max_target_velocity(float v);
    RAMSETEConfig& set_min_target_velocity(float v);
    RAMSETEConfig& set_dt(float dt);
};

struct FollowConfig {
    Direction direction = FLEXIBLE;
    float point_spacing = 0.25f;
    float max_start_deviation = 6.0f;
    float max_change_per_update = 2.0f;
    float max_ld = 24;
    float drive_min_voltage = 3;
    float max_full_speed_turn_radius = 24;
    float acceleration_distance = 6;
    PIDMotionConfig drive_pid = DRIVE_PID_DEFAULT;
    PIDMotionConfig heading_pid = HEADING_PID_DEFAULT;
    SettleConfig settle_conditions = MOVE_ODOM_SETTLE_DEFAULT;

    static bool default_initialized;

    // constructor 
    FollowConfig();
    FollowConfig(float max_ld);

    // Setters
    FollowConfig& set_direction(Direction dir);
    FollowConfig& set_point_spacing(float spacing);
    FollowConfig& set_acceleration_distance(float d);
    FollowConfig& set_max_start_deviation(float deviation);
    FollowConfig& set_max_change_per_update(float change);
    FollowConfig& set_max_ld(float ld);
    FollowConfig& set_drive_max_voltage(float voltage);
    FollowConfig& set_drive_min_voltage(float voltage);
    FollowConfig& set_heading_max_voltage(float voltage);
    FollowConfig& set_max_full_speed_turn_radius(float radius);
    FollowConfig& set_drive_pid(const PIDMotionConfig& pid);
    FollowConfig& set_heading_pid(const PIDMotionConfig& pid);
    FollowConfig& set_settle_conditions(const SettleConfig& settle);
    FollowConfig& set_settle_error(float e);
    FollowConfig& set_settle_time(float t);
    FollowConfig& set_timeout(float t);
    FollowConfig& set_max_current(float I);
};