#include "vex.h"

SettleConfig DRIVE_SETTLE_DEFAULT;
SettleConfig MOVE_ODOM_SETTLE_DEFAULT;
SettleConfig RAMSETE_SETTLE_DEFAULT; // RAMSETE is a bit special since we can calculate exactly how long the movement should take.
SettleConfig TURN_SETTLE_DEFAULT;
SettleConfig SWING_SETTLE_DEFAULT;

PIDMotionConfig DRIVE_PID_DEFAULT;
PIDMotionConfig HEADING_PID_DEFAULT;
PIDMotionConfig TURN_PID_DEFAULT;
PIDMotionConfig SWING_PID_DEFAULT;

DriveToPoseConfig DRIVE_TO_POSE_BOOMERANG_DEFAULT;
FollowConfig FOLLOW_PP_DEFAULT;
RAMSETEConfig FOLLOW_RAMSETE_DEFAULT;
VelocityControllerConfig DRIVE_VELOCITY_DEFAULT;

SettleConfig& SettleConfig::set_settle_error(float e) {
    this->settle_error = e;
    return *this;
}

SettleConfig& SettleConfig::set_settle_time(float t) {
    this->settle_time = t;
    return *this;
}

SettleConfig& SettleConfig::set_timeout(float t) {
    this->timeout = t;
    return *this;
}

SettleConfig& SettleConfig::set_max_current(float I) {
    this->max_current = I;
    return *this;
}


PIDConfig::PIDConfig(float kp, float ki, float kd, float integral_range): kp(kp), ki(ki), kd(kd), integral_range(integral_range) {}

PIDConfig& PIDConfig::set_kp(float val) {
    this->kp = val;
    return *this;
}

PIDConfig& PIDConfig::set_ki(float val) {
    this->ki = val;
    return *this;
}

PIDConfig& PIDConfig::set_kd(float val) {
    this->kd = val;
    return *this;
}

PIDConfig& PIDConfig::set_integral_range(float range) {
    this->integral_range = range;
    return *this;
}


PIDMotionConfig::PIDMotionConfig(float max_output, float kp, float ki, float kd, float integral_range): PIDConfig(kp, ki, kd, integral_range), max_output(max_output) {}

PIDMotionConfig& PIDMotionConfig::set_max_output(float val) {
    this->max_output = val;
    return *this;
}

PIDMotionConfig& PIDMotionConfig::set_kp(float val) {
    this->kp = val;
    return *this;
}

PIDMotionConfig& PIDMotionConfig::set_ki(float val) {
    this->ki = val;
    return *this;
}

PIDMotionConfig& PIDMotionConfig::set_kd(float val) {
    this->kd = val;
    return *this;
}

PIDMotionConfig& PIDMotionConfig::set_integral_range(float range) {
    this->integral_range = range;
    return *this;
}

PIDMotionConfig& PIDMotionConfig::operator=(const PIDConfig& pid) {
    this->kp = pid.kp;
    this->ki = pid.ki;
    this->kd = pid.kd;
    this->integral_range = pid.integral_range;
    return *this;
}


VelocityControllerConfig& VelocityControllerConfig::set_kv(float val) {
    this->kv = val;
    return *this;
}

VelocityControllerConfig& VelocityControllerConfig::set_ka(float val) {
    this->ka = val;
    return *this;
}

VelocityControllerConfig& VelocityControllerConfig::set_kf(float val) {
    this->kf = val;
    return *this;
}

VelocityControllerConfig& VelocityControllerConfig::set_kp(float val) {
    this->kp = val;
    return *this;
}

VelocityControllerConfig& VelocityControllerConfig::set_ki(float val) {
    this->ki = val;
    return *this;
}

VelocityControllerConfig& VelocityControllerConfig::set_kd(float val) {
    this->kd = val;
    return *this;
}

VelocityControllerConfig& VelocityControllerConfig::set_integral_range(float range) {
    this->integral_range = range;
    return *this;
}


TurnConfig& TurnConfig::set_turn_max_voltage(float voltage) {
    this->turn_pid.max_output = voltage;
    return *this;
}

TurnConfig& TurnConfig::set_heading_pid(const PIDConfig& pid) {
    this->turn_pid = pid;
    return *this;
}

TurnConfig& TurnConfig::set_settle_conditions(const SettleConfig& settle) {
    this->settle_conditions = settle;
    return *this;
}

TurnConfig& TurnConfig::set_settle_error(float e) {
    this->settle_conditions.settle_error = e;
    return *this;
}

TurnConfig& TurnConfig::set_settle_time(float t) {
    this->settle_conditions.settle_time = t;
    return *this;
}

TurnConfig& TurnConfig::set_timeout(float t) {
    this->settle_conditions.timeout = t;
    return *this;
}

TurnConfig& TurnConfig::set_max_current(float I) {
    this->settle_conditions.max_current = I;
    return *this;
}


SwingConfig& SwingConfig::set_turn_max_voltage(float voltage) {
    this->swing_pid.max_output = voltage;
    return *this;
}

SwingConfig& SwingConfig::set_heading_pid(const PIDConfig& pid) {
    this->swing_pid = pid;
    return *this;
}

SwingConfig& SwingConfig::set_settle_conditions(const SettleConfig& settle) {
    this->settle_conditions = settle;
    return *this;
}

SwingConfig& SwingConfig::set_settle_error(float e) {
    this->settle_conditions.settle_error = e;
    return *this;
}

SwingConfig& SwingConfig::set_settle_time(float t) {
    this->settle_conditions.settle_time = t;
    return *this;
}

SwingConfig& SwingConfig::set_timeout(float t) {
    this->settle_conditions.timeout = t;
    return *this;
}

SwingConfig& SwingConfig::set_max_current(float I) {
    this->settle_conditions.max_current = I;
    return *this;
}


DriveDistanceConfig& DriveDistanceConfig::set_drive_max_voltage(float voltage) {
    this->drive_pid.max_output = voltage;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_heading_max_voltage(float voltage) {
    this->heading_pid.max_output = voltage;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_drive_pid(const PIDConfig& pid) {
    this->drive_pid = pid;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_heading_pid(const PIDConfig& pid) {
    this->heading_pid = pid;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_settle_conditions(const SettleConfig& settle) {
    this->settle_conditions = settle;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_settle_error(float e) {
    this->settle_conditions.settle_error = e;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_settle_time(float t) {
    this->settle_conditions.settle_time = t;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_timeout(float t) {
    this->settle_conditions.timeout = t;
    return *this;
}

DriveDistanceConfig& DriveDistanceConfig::set_max_current(float I) {
    this->settle_conditions.max_current = I;
    return *this;
}


DriveToPointConfig& DriveToPointConfig::set_direction(Direction dir) {
    this->direction = dir;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_drive_max_voltage(float voltage) {
    this->drive_pid.max_output = voltage;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_heading_max_voltage(float voltage) {
    this->heading_pid.max_output = voltage;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_is_rigid(bool rigid) {
    this->is_rigid = rigid;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_drive_pid(const PIDConfig& pid) {
    this->drive_pid = pid;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_heading_pid(const PIDConfig& pid) {
    this->heading_pid = pid;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_settle_conditions(const SettleConfig& settle) {
    this->settle_conditions = settle;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_settle_error(float e) {
    this->settle_conditions.settle_error = e;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_settle_time(float t) {
    this->settle_conditions.settle_time = t;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_timeout(float t) {
    this->settle_conditions.timeout = t;
    return *this;
}

DriveToPointConfig& DriveToPointConfig::set_max_current(float I) {
    this->settle_conditions.max_current = I;
    return *this;
}


bool DriveToPoseConfig::default_initialized = false;

DriveToPoseConfig::DriveToPoseConfig() {
    if (DriveToPoseConfig::default_initialized)
        *this = DRIVE_TO_POSE_BOOMERANG_DEFAULT;
}

DriveToPoseConfig::DriveToPoseConfig(float lead_distance): 
        lead_distance(lead_distance) {
    if (DriveToPoseConfig::default_initialized) {
        *this = DRIVE_TO_POSE_BOOMERANG_DEFAULT;
        this->lead_distance = lead_distance;
    }
}

DriveToPoseConfig& DriveToPoseConfig::set_direction(Direction dir) {
    this->direction = dir;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_lead_distance(float d_lead) {
    this->lead_distance = d_lead;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_drive_max_voltage(float voltage) {
    this->drive_pid.max_output = voltage;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_heading_max_voltage(float voltage) {
    this->heading_pid.max_output = voltage;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_drive_pid(const PIDConfig& pid) {
    this->drive_pid = pid;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_heading_pid(const PIDConfig& pid) {
    this->heading_pid = pid;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_settle_conditions(const SettleConfig& settle) {
    this->settle_conditions = settle;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_settle_error(float error) {
    this->settle_conditions.settle_error = error;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_settle_time(float time) {
    this->settle_conditions.settle_time = time;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_timeout(float time) {
    this->settle_conditions.timeout = time;
    return *this;
}

DriveToPoseConfig& DriveToPoseConfig::set_max_current(float I) {
    this->settle_conditions.max_current = I;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_drive_max_voltage(float voltage) {
    this->drive_pid.max_output = voltage;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_turn_max_voltage(float voltage) {
    this->turn_pid.max_output = voltage;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_drive_pid(const PIDConfig& pid) {
    this->drive_pid = pid;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_turn_pid(const PIDConfig& pid) {
    this->turn_pid = pid;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_drive_settle_conditions(const SettleConfig& settle) {
    this->drive_settle_conditions = settle;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_turn_settle_conditions(const SettleConfig& settle) {
    this->turn_settle_conditions = settle;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_drive_settle_error(float e) {
    this->drive_settle_conditions.settle_error = e;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_drive_settle_time(float t) {
    this->drive_settle_conditions.settle_time = t;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_turn_settle_error(float e) {
    this->turn_settle_conditions.settle_error = e;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_turn_settle_time(float t) {
    this->turn_settle_conditions.settle_time = t;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_timeout(float t) {
    this->drive_settle_conditions.timeout = t;
    this->turn_settle_conditions.timeout = t;
    return *this;
}

HolonomicDriveToPointConfig& HolonomicDriveToPointConfig::set_max_current(float I) {
    this->drive_settle_conditions.max_current = I;
    this->turn_settle_conditions.max_current = I;
    return *this;
}


TrajectoryConfig& TrajectoryConfig::set_full_speed_turn_radius(float r) {
    this->max_full_speed_turn_radius = r;
    return *this;
}

TrajectoryConfig& TrajectoryConfig::set_deceleration_distance(float d) {
    this->acceleration_distance = d;
    return *this;
}

TrajectoryConfig& TrajectoryConfig::set_max_speed_in_per_sec(float v) {
    this->max_speed_in_per_sec = v;
    return *this;
}

TrajectoryConfig& TrajectoryConfig::set_max_target_velocity(float v) {
    this->max_target_velocity = v;
    return *this;
}

TrajectoryConfig& TrajectoryConfig::set_min_target_velocity(float v) {
    this->min_target_velocity = v;
    return *this;
}

TrajectoryConfig& TrajectoryConfig::set_dt(float dt) {
    this->dt = dt;
    return *this;
}


bool RAMSETEConfig::default_initialized = false;

RAMSETEConfig::RAMSETEConfig() {
    if (RAMSETEConfig::default_initialized)
        *this = FOLLOW_RAMSETE_DEFAULT;
}

RAMSETEConfig::RAMSETEConfig(float b, float zeta):
        b(b), zeta(zeta) {
    if (RAMSETEConfig::default_initialized) {
        *this = FOLLOW_RAMSETE_DEFAULT;
        this->b = b;
        this->zeta = zeta;
    }
}

RAMSETEConfig& RAMSETEConfig::set_settle_conditions(const SettleConfig& settle) {
    this->settle_conditions = settle;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_settle_error(float e) {
    this->settle_conditions.settle_error = e;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_settle_time(float t) {
    this->settle_conditions.settle_time = t;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_timeout(float t) {
    this->settle_conditions.timeout = t;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_max_current(float I) {
    this->settle_conditions.max_current = I;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_b(float convergence_rate) {
    this->b = convergence_rate;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_zeta(float damping) {
    this->zeta = damping;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_convergence_rate(float convergence_rate) {
    return this->set_b(convergence_rate);
}

RAMSETEConfig& RAMSETEConfig::set_damping(float damping) {
    return this->set_zeta(damping);
}

RAMSETEConfig& RAMSETEConfig::set_trajectory_config(const TrajectoryConfig& config) {
    this->trajectory_config = config;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_full_speed_turn_radius(float r) {
    this->trajectory_config.max_full_speed_turn_radius = r;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_deceleration_distance(float d) {
    this->trajectory_config.acceleration_distance = d;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_max_speed_in_per_sec(float v) {
    this->trajectory_config.max_speed_in_per_sec = v;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_max_target_velocity(float v) {
    this->trajectory_config.max_target_velocity = v;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_min_target_velocity(float v) {
    this->trajectory_config.min_target_velocity = v;
    return *this;
}

RAMSETEConfig& RAMSETEConfig::set_dt(float dt) {
    this->trajectory_config.dt = dt;
    return *this;
}


bool FollowConfig::default_initialized = false;

FollowConfig::FollowConfig() {
    if (FollowConfig::default_initialized)
        *this = FOLLOW_PP_DEFAULT;
}

FollowConfig::FollowConfig(float max_ld):
        max_ld(max_ld) {
    if (FollowConfig::default_initialized) {
        *this = FOLLOW_PP_DEFAULT;
        this->max_ld = max_ld;
    }
}

FollowConfig& FollowConfig::set_direction(Direction dir) {
    direction = dir;
    return *this;
}

FollowConfig& FollowConfig::set_point_spacing(float spacing) {
    point_spacing = spacing;
    return *this;
}

FollowConfig& FollowConfig::set_max_start_deviation(float deviation) {
    max_start_deviation = deviation;
    return *this;
}

FollowConfig& FollowConfig::set_max_change_per_update(float change) {
    max_change_per_update = change;
    return *this;
}

FollowConfig& FollowConfig::set_max_ld(float ld) {
    max_ld = ld;
    return *this;
}

FollowConfig& FollowConfig::set_drive_max_voltage(float voltage) {
    drive_pid.max_output = voltage;
    return *this;
}

FollowConfig& FollowConfig::set_drive_min_voltage(float voltage) {
    drive_min_voltage = voltage;
    return *this;
}

FollowConfig& FollowConfig::set_heading_max_voltage(float voltage) {
    heading_pid.max_output = voltage;
    return *this;
}

FollowConfig& FollowConfig::set_max_full_speed_turn_radius(float radius) {
    max_full_speed_turn_radius = radius;
    return *this;
}

FollowConfig& FollowConfig::set_acceleration_distance(float d) {
    this->acceleration_distance = d;
    return *this;
}

FollowConfig& FollowConfig::set_drive_pid(const PIDMotionConfig& pid) {
    drive_pid = pid;
    return *this;
}

FollowConfig& FollowConfig::set_heading_pid(const PIDMotionConfig& pid) {
    heading_pid = pid;
    return *this;
}

FollowConfig& FollowConfig::set_settle_conditions(const SettleConfig& settle) {
    settle_conditions = settle;
    return *this;
}

FollowConfig& FollowConfig::set_settle_error(float e) {
    this->settle_conditions.settle_error = e;
    return *this;
}

FollowConfig& FollowConfig::set_settle_time(float t) {
    this->settle_conditions.settle_time = t;
    return *this;
}

FollowConfig& FollowConfig::set_timeout(float t) {
    this->settle_conditions.timeout = t;
    return *this;
}

FollowConfig& FollowConfig::set_max_current(float I) {
    this->settle_conditions.max_current = I;
    return *this;
}