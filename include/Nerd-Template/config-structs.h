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
struct FollowConfig;
struct RAMSETEConfig;

extern SettleConfig DRIVE_SETTLE_DEFAULT;
extern SettleConfig MOVE_ODOM_SETTLE_DEFAULT;
extern SettleConfig TURN_SETTLE_DEFAULT;
extern SettleConfig SWING_SETTLE_DEFAULT;

extern PIDMotionConfig DRIVE_PID_DEFAULT;
extern PIDMotionConfig HEADING_PID_DEFAULT;
extern PIDMotionConfig TURN_PID_DEFAULT;
extern PIDMotionConfig SWING_PID_DEFAULT;

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
};

struct SwingConfig {
    PIDMotionConfig swing_pid = SWING_PID_DEFAULT;
    SettleConfig settle_conditions = SWING_SETTLE_DEFAULT;

    SwingConfig& set_turn_max_voltage(float voltage);
    SwingConfig& set_heading_pid(const PIDConfig& pid);
    SwingConfig& set_settle_conditions(const SettleConfig& settle);
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
    HolonomicDriveToPointConfig& set_turn_settle_conditions(const SettleConfig& settle);
};

struct TrajectoryConfig {
    float max_full_speed_turn_radius = 36.0f;
    float deceleration_distance = 8.0f;
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

struct RAMSETEConfig : public TrajectoryConfig, SettleConfig {
    float b = 2.0f;
    float zeta = 0.7f;
    float k2_v_weight = 0.f;
    float k2_b_weight = 1.f;
    float k3_omega_weight = 0.f;
    float k3_b_weight = 1.f;
    
    RAMSETEConfig();
    RAMSETEConfig(float b, float zeta, float k2_v_weight = 0.f, float k2_b_weight = 1.f, float k3_omega_weight = 0.f, float k3_b_weight = 1.f);

    RAMSETEConfig& set_settle_error(float e);
    RAMSETEConfig& set_settle_time(float t);
    RAMSETEConfig& set_timeout(float t);
    RAMSETEConfig& set_max_current(float I);
    RAMSETEConfig& set_b(float convergence_rate);
    RAMSETEConfig& set_zeta(float damping);
    RAMSETEConfig& set_k2_v_weight(float w);
    RAMSETEConfig& set_k2_b_weight(float w);
    RAMSETEConfig& set_k3_omega_weight(float w);
    RAMSETEConfig& set_k3_b_weight(float w);
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
    PIDMotionConfig drive_pid;
    PIDMotionConfig heading_pid;
    SettleConfig settle_conditions;

    // constructor
    FollowConfig(float drive_min_voltage = 3, float max_full_speed_turn_radius = 24, float max_ld = 24, float acceleration_distance = 6);

    // Setters
    FollowConfig& set_direction(Direction dir);
    FollowConfig& set_point_spacing(float spacing);
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
};