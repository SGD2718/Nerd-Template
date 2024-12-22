#include "vex.h"
#include <iostream>
#include <vector>

//Odom Drive::odom;

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float motor_rpm, float drive_rpm, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance, float width) :
    wheel_diameter(wheel_diameter),
    drive_rpm(drive_rpm),
    motor_rpm(motor_rpm),
    wheel_ratio(drive_rpm / motor_rpm),
    max_velocity_in_per_sec((M_PI * this->wheel_diameter) * (this->drive_rpm / 60.0f)),
    gyro_scale(gyro_scale),
    drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
    ForwardTracker_center_distance(ForwardTracker_center_distance),
    ForwardTracker_diameter(ForwardTracker_diameter),
    ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
    SidewaysTracker_center_distance(SidewaysTracker_center_distance),
    SidewaysTracker_diameter(SidewaysTracker_diameter),
    SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
    drive_setup(drive_setup),
    DriveL(DriveL),
    DriveR(DriveR),
    Gyro(inertial(gyro_port)),
    DriveLF(DriveLF_port, is_reversed(DriveLF_port)),
    DriveRF(DriveRF_port, is_reversed(DriveRF_port)),
    DriveLB(DriveLB_port, is_reversed(DriveLB_port)),
    DriveRB(DriveRB_port, is_reversed(DriveRB_port)),
    R_ForwardTracker(ForwardTracker_port),
    R_SidewaysTracker(SidewaysTracker_port),
    E_ForwardTracker(ThreeWire.D),
    E_SidewaysTracker(ThreeWire.F),
    width(width)
{
    if (this->drive_setup != ZERO_TRACKER) {
        if (this->drive_setup & TANK_IME)
            odom.set_physical_distances(0, 0);
        else if (this->drive_setup & (TANK_ONE_ENCODER | TANK_ONE_ROTATION))
            odom.set_physical_distances(0, SidewaysTracker_center_distance);
        else {
            std::cout << "two rotation" << std::endl;
            std::cout << "Forward Ratio: " << this->ForwardTracker_in_to_deg_ratio << std::endl;
            std::cout << "Forward Diameter: " << this->ForwardTracker_diameter << std::endl;
            std::cout << "Sideways Ratio: " << this->SidewaysTracker_in_to_deg_ratio << std::endl;
            std::cout << "Sideways Diameter: " << this->SidewaysTracker_diameter << std::endl;
            odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
        }

        this->R_ForwardTracker.setPosition(0, deg);
        this->R_SidewaysTracker.setPosition(0, deg);
        
        this->async_task = vex::task(position_track_task);
        set_coordinates(0,0,90);
        std::cout << "Chassis Initialized2" << std::endl;
    }
}

float Drive::get_max_velocity() const {
    return this->max_velocity_in_per_sec;
}

float Drive::get_current() {
    return fmaxf(this->DriveL.current(), this->DriveR.current());
}

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
    DriveL.spin(fwd, leftVoltage, volt);
    DriveR.spin(fwd, rightVoltage, volt);
}

void Drive::drive_keep_turn_rate(float leftVoltage, float rightVoltage) {
    auto high = fmaxf(fabsf(leftVoltage), fabsf(rightVoltage));
    if (high > 12.0f) {
        high = 1.0f / high;
        leftVoltage *= high;
        rightVoltage *= high;
    }
    DriveL.spin(fwd, leftVoltage, volt);
    DriveR.spin(fwd, rightVoltage, volt);
}

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
    this->turn_max_voltage = turn_max_voltage;
    this->turn_kp = turn_kp;
    this->turn_ki = turn_ki;
    this->turn_kd = turn_kd;
    this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
    this->drive_max_voltage = drive_max_voltage;
    this->drive_kp = drive_kp;
    this->drive_ki = drive_ki;
    this->drive_kd = drive_kd;
    this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
    this->heading_max_voltage = heading_max_voltage;
    this->heading_kp = heading_kp;
    this->heading_ki = heading_ki;
    this->heading_kd = heading_kd;
    this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
    this->swing_max_voltage = swing_max_voltage;
    this->swing_kp = swing_kp;
    this->swing_ki = swing_ki;
    this->swing_kd = swing_kd;
    this->swing_starti = swing_starti;
} 

void Drive::set_follow_constants(float follow_max_voltage, float follow_max_acceleration, float follow_max_applicable_curvature_radius, float follow_min_ld, float follow_max_ld, float follow_ld) {
    this->follow_max_voltage = follow_max_voltage;
    this->follow_max_acceleration = follow_max_acceleration;
    this->follow_min_ld = follow_min_ld;
    this->follow_max_ld = follow_max_ld;
    this->follow_ld = follow_ld;
    this->follow_mu = 12.0f / sqrtf(follow_max_applicable_curvature_radius);
}

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
    this->turn_settle_error = turn_settle_error;
    this->turn_settle_time = turn_settle_time;
    this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_continue_error, float drive_continue_time){
    this->drive_settle_error = drive_settle_error;
    this->drive_settle_time = drive_settle_time;
    this->drive_timeout = drive_timeout;
    this->drive_continue_error = drive_continue_error;
    this->drive_continue_time = drive_continue_time;
}

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
    this->swing_settle_error = swing_settle_error;
    this->swing_settle_time = swing_settle_time;
    this->swing_timeout = swing_timeout;
}

void Drive::set_follow_exit_conditions(float follow_settle_error, float follow_settle_time, float follow_timeout) {
    this->follow_settle_error = follow_settle_error;
    this->follow_timeout = follow_timeout;
    this->follow_settle_time = follow_settle_time;
    //this->follow_heading_error_turn_threshold = follow_heading_error_turn_threshold;
}

void Drive::set_brake_type(vex::brakeType mode) {
    this->DriveL.setStopping(mode);
    this->DriveR.setStopping(mode);
}

float Drive::get_absolute_heading(){ 
    return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) ); 
}

float Drive::get_left_position_in(){
    return( DriveL.position(deg) * drive_in_to_deg_ratio );
}

float Drive::get_right_position_in(){
    return( DriveR.position(deg) * drive_in_to_deg_ratio );
}

/*void Drive::turn_to_angle(float angle){
    turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}*/

void Drive::turn_to_angle(float angle, float turn_timeout){
    turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_timeout, float turn_max_voltage){
    turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time){
    turn_to_angle(angle, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
    if (this->stop_auton) return;
    desired_heading = angle;
    PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
    while (!this->stop_auton && turnPID.is_settled() == false){
        float error = reduce_negative_180_to_180(angle - get_absolute_heading());
        float output = turnPID.compute(error);
        output = clamp(output, -turn_max_voltage, turn_max_voltage);
        drive_with_voltage(-output, output);

        std::cout << "error = " << error << ", accumulated = " << turnPID.accumulated_error << ", max i rate = " << turnPID.integral_range << std::endl;
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}

void Drive::turn_to_angle(float angle, const TurnConfig& config) {
    if (this->stop_auton) return;
    desired_heading = angle;
    PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), config.turn_pid, config.settle_conditions);
    while (!this->stop_auton && !turnPID.is_settled(this->get_current())){
        float error = reduce_negative_180_to_180(angle - get_absolute_heading());
        float output = turnPID.compute(error);
        output = clamp(output, -config.turn_pid.max_output, config.turn_pid.max_output);
        drive_with_voltage(-output, output);
        task::sleep(10);
    }
    if (this->stop_auton)
        this->set_brake_type(coast);
    else
        this->stop(brake);
}

void Drive::drive_time(float time, float voltage, bool stop) {
    this->DriveL.spin(fwd, voltage, volt);
    this->DriveR.spin(fwd, voltage, volt);
    task::sleep(time);
    if (stop) {
        this->stop(brake);
    }
}

void Drive::stop(brakeType mode) {
    this->DriveL.stop(mode);
    this->DriveR.stop(mode);
}

/*void Drive::drive_distance(float distance){
    drive_distance(distance, desired_heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
    drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}*/

void Drive::drive_distance(float distance, float heading, float drive_timeout){
    drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage) {
    drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time){
    drive_distance(distance, heading, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
    desired_heading = heading;
    PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
    PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
    float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float average_position = start_average_position;
    while(!this->stop_auton && drivePID.is_settled() == false){
        average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float drive_error = distance+start_average_position-average_position;
        float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
        float drive_output = drivePID.compute(drive_error);
        float heading_output = headingPID.compute(heading_error);

        drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
        heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

        drive_with_voltage(drive_output-heading_output, drive_output+heading_output);
        //std::cout << "error = " << drive_error << ", accumulated = " << drivePID.accumulated_error << ", maxirate = " << drivePID.integral_max_rate << std::endl;
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}

void Drive::drive_distance(float distance, const DriveDistanceConfig& config) {
    this->drive_distance(distance, this->desired_heading, config);
}

void Drive::drive_distance(float distance, float heading, const DriveDistanceConfig& config) {
    this->desired_heading = heading;
    PID drivePID(distance, config.drive_pid, config.settle_conditions);
    PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), config.heading_pid);
    float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float average_position = start_average_position;
    while(!this->stop_auton && drivePID.is_settled(this->get_current()) == false){
        average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float drive_error = distance+start_average_position-average_position;
        float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
        float drive_output = drivePID.compute(drive_error);
        float heading_output = headingPID.compute(heading_error);

        drive_output = clamp(drive_output, -config.drive_pid.max_output, config.drive_pid.max_output);
        heading_output = clamp(heading_output, -config.heading_pid.max_output, config.heading_pid.max_output);

        drive_with_voltage(drive_output-heading_output, drive_output+heading_output);
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}

void Drive::left_swing_to_angle(float angle, const SwingConfig& config) {
    if (this->stop_auton) return;
    desired_heading = angle;
    PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), config.swing_pid, config.settle_conditions);
    while(!this->stop_auton && swingPID.is_settled(this->get_current()) == false){
        float error = reduce_negative_180_to_180(angle - get_absolute_heading());
        float output = swingPID.compute(error);
        output = clamp(output, -config.swing_pid.max_output, config.swing_pid.max_output);
        DriveL.spin(fwd, -output, volt);
        DriveR.stop(hold);
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}

void Drive::right_swing_to_angle(float angle, const SwingConfig& config) {
    if (this->stop_auton) return;
    desired_heading = angle;
    PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), config.swing_pid, config.settle_conditions);
    while(!this->stop_auton && swingPID.is_settled(this->get_current()) == false){
        float error = reduce_negative_180_to_180(angle - get_absolute_heading());
        float output = swingPID.compute(error);
        output = clamp(output, -config.swing_pid.max_output, config.swing_pid.max_output);
        DriveR.spin(fwd, output, volt);
        DriveL.stop(hold);
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}

float Drive::get_ForwardTracker_position(){
    if (drive_setup & (TANK_TWO_ENCODER | HOLONOMIC_TWO_ENCODER))
        return(E_ForwardTracker.position(deg) * ForwardTracker_in_to_deg_ratio);
    if (drive_setup & (TANK_ONE_ENCODER | TANK_ONE_ROTATION | TANK_IME))
        return ( get_right_position_in() + get_left_position_in() ) * 0.5;
    return(R_ForwardTracker.position(deg) * ForwardTracker_in_to_deg_ratio);
}

float Drive::get_SidewaysTracker_position(){
    if (TANK_IME)
        return 0;
    if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER)
        return(E_SidewaysTracker.position(deg) * SidewaysTracker_in_to_deg_ratio);
    return(R_SidewaysTracker.position(deg) * SidewaysTracker_in_to_deg_ratio);
}

void Drive::position_track() {
    while(true) {
        odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
        task::sleep(5);
    }
}

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
    this->Gyro.setRotation(orientation_deg / gyro_scale * 360, deg);
    this->desired_heading = orientation_deg;
    odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
}

int Drive::position_track_task(){
    chassis.position_track();
    std::cout << "ODOM DONE! SOMETHING WENT VERY WRONG!!!" << std::endl;
    return(0);
}

float Drive::get_X_position() const {
    return(odom.X_position);
}

float Drive::get_Y_position() const {
    return(odom.Y_position);
}

Vector2 Drive::get_position() const {
    return Vector2(odom.X_position, odom.Y_position);
}

Pose Drive::get_pose() {
    return Pose(odom.X_position, odom.Y_position, this->get_absolute_heading());
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse){
    drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout){
    drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage){
    drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time){
    drive_to_point(X_position, Y_position, is_rigid, is_reverse, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(const Vector2& pos, const DriveToPointConfig& config) {
    if (this->stop_auton) return;

    if (config.is_rigid) {
        this->turn_to_point(pos, config.direction == REVERSE ? 180.0f : 0.0f, TurnConfig().set_turn_max_voltage(12).set_settle_conditions(SettleConfig().set_settle_error(5).set_settle_time(50).set_timeout(500)));
    }

    auto vector_to_robot = this->get_position() - pos;
    PID drivePID(vector_to_robot.norm(), config.drive_pid, config.settle_conditions);
    PID headingPID(reduce_negative_180_to_180(to_deg(vector_to_robot.heading())-get_absolute_heading()), config.heading_pid);

    while(!this->stop_auton && drivePID.is_settled(this->get_current()) == false){
        auto vector_to_robot = this->get_position() - pos;
        float drive_error = vector_to_robot.norm();
        float heading_error = reduce_negative_180_to_180(to_deg(vector_to_robot.heading()) - get_absolute_heading());
        float drive_output = drivePID.compute(drive_error);

        float heading_scale_factor = cos(to_rad(heading_error));

        switch (config.direction) {
            case FORWARD:
                heading_scale_factor = fmaxf(0, heading_scale_factor);
                break;
            case REVERSE:
                heading_scale_factor = fminf(0, -heading_scale_factor);
                break;
            case FLEXIBLE:
                heading_error = reduce_negative_90_to_90(heading_error);
                break;
        }

        drive_output *= heading_scale_factor;
        float heading_output = headingPID.compute(heading_error);
        
        if (drive_error < config.settle_conditions.settle_error) { heading_output = 0; }

        float scaled_drive_max_voltage = fabs(heading_scale_factor) * config.drive_pid.max_output;
        drive_output = clamp(drive_output, -scaled_drive_max_voltage, scaled_drive_max_voltage);
        heading_output = clamp(heading_output, -config.heading_pid.max_output, config.heading_pid.max_output);

        drive_with_voltage(drive_output-heading_output, drive_output+heading_output);

        std::cout << drive_error << "," << std::endl;
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}

void Drive::drive_to_point(float X_position, float Y_position, bool is_rigid, bool is_reverse, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
    if (this->stop_auton) return;

    if (is_rigid) {
        this->turn_to_point(X_position, Y_position, is_reverse ? 180.0f : 0.0f, 500, 12, 5, 50);
    }

    PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
    PID headingPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position - get_X_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);

    while(!this->stop_auton && drivePID.is_settled() == false){
        float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
        float heading_error = reduce_negative_180_to_180(to_deg(atan2(Y_position - get_Y_position(), X_position - get_X_position())) - get_absolute_heading());
        float drive_output = drivePID.compute(drive_error);

        float heading_scale_factor = cos(to_rad(heading_error));
        drive_output *= heading_scale_factor;
        heading_error = reduce_negative_90_to_90(heading_error);
        float heading_output = headingPID.compute(heading_error);
        
        if (drive_error<drive_settle_error) { heading_output = 0; }

        float scaled_drive_max_voltage = fabs(heading_scale_factor)*drive_max_voltage;
        drive_output = clamp(drive_output, -scaled_drive_max_voltage, scaled_drive_max_voltage);
        heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

        drive_with_voltage(drive_output-heading_output, drive_output+heading_output);

        std::cout << "drive_error = " << drive_error << ", drive output = " << drive_output << ", heading output = " << heading_output << std::endl;
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}

/*void Drive::boomerang(float X_position, float Y_position, float heading, bool reverse, float lead_distance) {
    boomerang(X_position, Y_position, heading, reverse, lead_distance, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::boomerang(float X_position, float Y_position, float heading, bool reverse, float lead_distance, float drive_timeout, float drive_max_voltage, float heading_max_voltage){
    boomerang(X_position, Y_position, heading, reverse, lead_distance, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::boomerang(float X_position, float Y_position, float heading, bool reverse, float lead_distance, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time){
    boomerang(X_position, Y_position, heading, reverse, lead_distance, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::boomerang(float X_position, float Y_position, float heading, bool reverse, float lead_distance, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
    if (this->stop_auton) return;

    auto target = Vector2(X_position, Y_position);
    auto displacement = target - this->get_position();

    PID drivePID(displacement.norm(), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
    PID headingPID(reduce_negative_180_to_180(to_deg(displacement.angle()) - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti, turn_settle_error, turn_settle_time, drive_timeout);

    auto target_heading_rad = to_rad(heading);
    auto carrot_offset = Vector2(cosf(target_heading_rad) * lead_distance, sinf(target_heading_rad) * lead_distance);

    while(!this->stop_auton && (!drivePID.is_settled() || !headingPID.is_settled())){
        displacement = target - this->get_position();
        auto carrot = target - displacement.norm() * carrot_offset;
        auto to_carrot = carrot - this->get_position();

        float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
        float drive_error = (carrot - this->get_position()).norm();
        
        float drive_output = 5 * drivePID.compute(drive_error);

        //float heading_scale_factor = cos(to_rad(heading_error));
        //drive_output *= heading_scale_factor;
        //heading_error = reduce_negative_90_to_90(heading_error);
        float heading_output = headingPID.compute(heading_error);
        
        if (drive_error<drive_settle_error) { heading_output = 0; }

        float scaled_drive_max_voltage = drive_max_voltage;//fabs(heading_scale_factor)*drive_max_voltage;
        drive_output = clamp(drive_output, -scaled_drive_max_voltage, scaled_drive_max_voltage);
        heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

        drive_with_voltage(drive_output-heading_output, drive_output+heading_output);

        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}*/

void Drive::follow(const std::vector<Vector2>& path, const FollowConfig& config) {
    if (stop_auton) return;
    std::cout << "\n\n\n----------------------------\n\nrunning follow" << std::endl;
    auto pure_puresuit = PurePursuit(this->get_position(), this->get_absolute_heading(), path, config);

    if (pure_puresuit.is_settled()) {
        std::cout << "Pure Pursuit started settled... wtf" << std::endl;
    }
    while (!this->stop_auton && !pure_puresuit.is_settled()) {
        float drive_output, heading_output;
        pure_puresuit.compute(this->get_position(), this->get_absolute_heading(), drive_output, heading_output);
        std::cout << "Follow: drive output = " << drive_output << ", heading output = " << heading_output << std::endl;
        this->drive_with_voltage(drive_output - heading_output, drive_output + heading_output);
        task::sleep(10);
    }
    if (!this->stop_auton) {
        this->stop(brake);
        pure_puresuit.print_logs();
    } else 
        this->set_brake_type(coast);
}

void Drive::follow(const std::vector<Vector2>& path) {
    follow(path, follow_timeout, true, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth) {
    follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage) {
    follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}
void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld) {
    follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}
void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time) {
    follow(path, follow_timeout, make_smooth, follow_max_voltage, heading_max_voltage, follow_ld, follow_settle_error, follow_settle_time, follow_mu, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time, float follow_mu, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti) {
    if (path.empty() || this->stop_auton) return;
    if (path.size() == 1) {
        drive_to_point(path[0].x, path[0].y, false, false, follow_timeout, follow_max_voltage, heading_max_voltage, follow_settle_error, follow_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
        return;
    }

    std::vector<Vector2> waypoints;
    int max_waypoint_index;
    
    if (make_smooth) {
        // fill waypoints
        for (int i = 0; i < path.size() - 1; ++i) {
            auto direction_vector = (path[i+1] - path[i]).unit_vector();
            auto point = path[i];
            int length = (int)(path[i+1] - path[i]).norm();
            for (int j = 0; j <= length; ++j) {
                waypoints.emplace_back(point);
                point += direction_vector;
                if ((point - path[i]).norm_squared() < 0.25) break;
            }
        }

        max_waypoint_index = waypoints.size() - 1;

        // smoothing
        const float min_squared_deviation_to_keep_smoothing = 0.001;
        if (waypoints.size() >= 3) {
            const float t = 0.9;    // smoothing weight

            float max_squared_deviation;
            do {
                max_squared_deviation = 0;
                auto reference = waypoints;

                for (int i = 1; i < max_waypoint_index; ++i) {
                    waypoints[i] = lerp<Vector2>(reference[i], (reference[i-1] + reference[i+1]) * 0.5, t);
                    max_squared_deviation = std::fmaxf((waypoints[i] - reference[i]).norm_squared(), max_squared_deviation);
                }
            } while (max_squared_deviation >= min_squared_deviation_to_keep_smoothing);
        }
    } else {
        waypoints = path;
        max_waypoint_index = waypoints.size() - 1;
    }

    // compute distances
    float distance_remaining = 0;
    std::vector<float> distances;
    for (int i = 0; i < max_waypoint_index; ++i) {
        distances.emplace_back((waypoints[i+1] - waypoints[i]).norm());
        distance_remaining += distances.back();
    }
    distances.emplace_back(0);

    // get initial look ahead point
    int look_ahead_index = 0;
    int nearest_index = 0;
    Vector2 look_ahead_point = waypoints.front();
    
    while (look_ahead_index < max_waypoint_index && (look_ahead_point - get_position()).norm() < ld) {
        distance_remaining -= distances[look_ahead_index++];
        look_ahead_point = waypoints[look_ahead_index];
    }

    // get initial nearest point
    float squared_distance_to_path;
    float next_squared_distance;
    
    while (nearest_index < max_waypoint_index) {
        squared_distance_to_path = (path[nearest_index] - get_position()).norm_squared();
        next_squared_distance = (path[nearest_index + 1] - get_position()).norm_squared();
        if (next_squared_distance < squared_distance_to_path) ++nearest_index;
        else break;
    }

    // follow path
    PID drivePID(distance_remaining + (look_ahead_point - get_position()).norm(), drive_kp, drive_ki, drive_kd, drive_starti, follow_settle_error, follow_settle_time, follow_timeout);
    PID headingPID(reduce_negative_180_to_180(to_deg((look_ahead_point - get_position()).angle()) - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
    
    while (!this->stop_auton && !drivePID.is_settled()) {
        // update look ahead point
        while (look_ahead_index < max_waypoint_index && (look_ahead_point - get_position()).norm() < follow_ld) {
            distance_remaining -= distances[look_ahead_index++];
            look_ahead_point = waypoints[look_ahead_index];
        }
    
        // update nearest point
        while (nearest_index < max_waypoint_index) {
            squared_distance_to_path = (path[nearest_index] - get_position()).norm_squared();
            next_squared_distance = (path[nearest_index + 1] - get_position()).norm_squared();
            if (next_squared_distance < squared_distance_to_path) ++nearest_index;
            else break;
        }
        
        // pursue look ahead point
        float heading_error = reduce_negative_180_to_180(to_deg((look_ahead_point - get_position()).angle()) - get_absolute_heading());
        float heading_scale_factor = cos(to_rad(heading_error));
        heading_error = reduce_negative_90_to_90(heading_error);
        float heading_output = headingPID.compute(heading_error);
        heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

        float drive_error = distance_remaining + (look_ahead_point - get_position()).norm();
        float drive_output = drivePID.compute(drive_error); // update PID so that the derivative and integral is accurate.

        if (nearest_index != 0 && look_ahead_index != max_waypoint_index) {
            // Use v = sqrt(µgR), µ = coefficient of friction, g = gravity, and R = radius of curvature. Note that follow_mu = sqrt(µg).
            if (!(waypoints[nearest_index] - waypoints[nearest_index - 1]).is_parallel(waypoints[nearest_index + 1] - waypoints[nearest_index])) {
                float radius_of_curvature = three_point_circle_radius(waypoints[nearest_index - 1], waypoints[nearest_index], waypoints[nearest_index + 1]);
                drive_output = std::fminf(drive_output, follow_mu * sqrtf(radius_of_curvature));
            } else {
                drive_output = std::fminf(drive_output, follow_max_voltage);
            }
        }
        drive_output = clamp(drive_output, -follow_max_voltage, follow_max_voltage);
        drive_output *= heading_scale_factor;

        if (drive_error < follow_settle_error) { heading_output = 0; }

        drive_with_voltage(drive_output - heading_output, drive_output + heading_output);
        std::cout << "follow output: " << drive_output << std::endl;
        vex::task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(vex::hold);
        DriveR.stop(vex::hold);
    } else {
        DriveR.setStopping(vex::coast);
        DriveL.setStopping(vex::coast);
    }
}

void Drive::follow(const std::vector<Waypoint>& waypoints, const RAMSETEConfig& config) {
    this->follow(waypoints, config, this->velocity_controller_config);
}

void Drive::follow(const std::vector<Waypoint>& waypoints, const RAMSETEConfig& config, const VelocityControllerConfig& velocity_controller_config) {
    Trajectory trajectory(waypoints, config);
    VelocityController left_velocity_controller(0, velocity_controller_config);
    VelocityController right_velocity_controller(0, velocity_controller_config);
    RAMSETE ramsete(config);
    Settle settle(config);
    settle.timeout += trajectory.get_duration();

    float t = 0;

    State target = trajectory.sample(t);
    Pose pose = this->get_pose();

    while (!this->stop_auton && !settle.is_exit(this->get_current())) {
        // get controller output
        auto steer = ramsete.compute(pose, target);
        
        // convert Ackerman inputs to differential drive inputs
        auto turn = this->width * 0.5 * steer.omega;
        auto left_velocity = steer.v - turn;
        auto right_velocity = steer.v + turn;

        // get voltage from velocity controller
        auto left_voltage = left_velocity_controller.compute(rpm_to_volt(DriveL.velocity(rpm), this->motor_rpm), left_velocity);
        auto right_voltage = right_velocity_controller.compute(rpm_to_volt(DriveR.velocity(rpm), this->motor_rpm), right_velocity);

        // drive
        this->drive_with_voltage(left_voltage, right_voltage);
        
        // update state
        task::sleep(roundf(config.dt));
        t += config.dt;

        target = trajectory.sample(t);
        pose = this->get_pose();

        settle.update(t < trajectory.get_duration() ? std::numeric_limits<float>::infinity() : (target - pose).norm_squared()); // don't let it settle until it reaches the end of the trajectory.
    }

    if (this->stop_auton)
        this->set_brake_type(coast);
    else
        this->stop(brake);
}

void Drive::turn_to_point(float X_position, float Y_position){
    turn_to_point(X_position, Y_position, 0, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
    turn_to_point(X_position, Y_position, extra_angle_deg, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout){
    turn_to_point(X_position, Y_position, extra_angle_deg, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time){
    turn_to_point(X_position, Y_position, extra_angle_deg, turn_timeout, turn_max_voltage, turn_settle_error, turn_settle_time, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_timeout, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
    if (this->stop_auton) return;
    PID turnPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position - get_X_position())) - get_absolute_heading() + extra_angle_deg), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
    while(!this->stop_auton && turnPID.is_settled() == false){
        float error = reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position - get_X_position())) - get_absolute_heading() + extra_angle_deg);
        float output = turnPID.compute(error);
        output = clamp(output, -turn_max_voltage, turn_max_voltage);
        drive_with_voltage(-output, output);
        task::sleep(10);
    }
    if (!this->stop_auton) {
        DriveL.stop(hold);
        DriveR.stop(hold);
    } else {
        DriveR.setStopping(coast);
        DriveL.setStopping(coast);
    }
}


void Drive::turn_to_point(const Vector2& pos, float extra_angle_deg, const TurnConfig& config){
    if (this->stop_auton) return;
    PID turnPID(reduce_negative_180_to_180(to_deg((pos - this->get_position()).heading()) - get_absolute_heading() + extra_angle_deg), config.turn_pid, config.settle_conditions);
    while (!this->stop_auton && !turnPID.is_settled(this->get_current())){
        float error = reduce_negative_180_to_180(to_deg((pos - this->get_position()).heading()) - get_absolute_heading() + extra_angle_deg);
        float output = turnPID.compute(error);
        output = clamp(output, -config.turn_pid.max_output, config.turn_pid.max_output);
        drive_with_voltage(-output, output);
        task::sleep(10);
    }
    if (!this->stop_auton)
        this->stop(brake);
    else
        this->set_brake_type(coast);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position){
    holonomic_drive_to_point(X_position, Y_position, get_absolute_heading(), drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle){
    holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout){
    holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage){
    holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time){
    holonomic_drive_to_point(X_position, Y_position, angle, drive_timeout, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_timeout, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
    if (this->stop_auton) return;
    PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
    PID turnPID(reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position-get_X_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
    while(!this->stop_auton && !( drivePID.is_settled() && turnPID.is_settled() ) ){
        float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
        float turn_error = reduce_negative_180_to_180(to_deg(atan2(Y_position-get_Y_position(), X_position-get_X_position()))-get_absolute_heading());

        float drive_output = drivePID.compute(drive_error);
        float turn_output = turnPID.compute(turn_error);

        drive_output = clamp(drive_output, drive_max_voltage, drive_max_voltage);
        turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

        float heading_error = atan2(Y_position-get_Y_position(), X_position-get_X_position());

        DriveLF.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output, volt);
        DriveLB.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output, volt);
        DriveRB.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output, volt);
        DriveRF.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output, volt);
        task::sleep(10);
    }
    DriveLF.stop(hold);
    DriveLB.stop(hold);
    DriveRB.stop(hold);
    DriveRF.stop(hold);
}

float joystick_curve(float value, float t) {
        //return value * expf((fabsf(value)/127 - 1) * t);
        float tmp = expf(-t * 0.1);
        return (tmp + expf((fabsf(value) - 127) * 0.1) * (1-tmp)) * value;
}

void Drive::control_arcade(float tFwd, float tTurn){
    auto velCtrl = joystick_curve(vex::controller(primary).Axis3.value(), tFwd);
    auto turnCtrl = joystick_curve(vex::controller(primary).Axis1.value(), tTurn);
    DriveL.spin(fwd, to_volt(velCtrl+turnCtrl), volt);
    DriveR.spin(fwd, to_volt(velCtrl-turnCtrl), volt);
}

void Drive::control_tank(float t){
    auto left_joystick = Vector2(Controller1.Axis3.value(), Controller1.Axis4.value()).norm() * fsign(Controller1.Axis3.value());
    auto right_joystick = Vector2(Controller1.Axis1.value(), Controller1.Axis2.value()).norm() * fsign(Controller1.Axis2.value());
    this->DriveL.spin(fwd, to_volt(joystick_curve(left_joystick, t)), volt);
    this->DriveR.spin(fwd, to_volt(joystick_curve(right_joystick, t)), volt);
}