#include "vex.h"
#include <iostream>
#include <vector>

//Odom Drive::odom;

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float motor_rpm, float drive_rpm, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance, float width) :
    wheel_diameter(wheel_diameter),
    motor_rpm(motor_rpm),
    drive_rpm(drive_rpm),
    wheel_ratio(drive_rpm / motor_rpm),
    gyro_scale(gyro_scale),
    drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
    max_velocity_in_per_sec((M_PI * wheel_diameter) * (drive_rpm / 60.0f)),
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
            odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
        }

        this->R_ForwardTracker.setPosition(0, deg);
        this->R_SidewaysTracker.setPosition(0, deg);
        
        this->async_task = vex::task(position_track_task);
        set_coordinates(0,0,90);
        std::cout << "Chassis Initialized" << std::endl;
    }
}

float Drive::get_max_velocity() const {
    return this->max_velocity_in_per_sec;
}

float Drive::get_current() {
    return fmaxf(this->DriveL.current(), this->DriveR.current()) / 3.f;
}

float Drive::get_holonomic_current() {
    return fmaxf(
        fmaxf(this->DriveLF.current(), this->DriveRF.current()),
        fmaxf(this->DriveLB.current(), this->DriveRB.current())
    );
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

void Drive::set_follow_constants(float follow_max_voltage, float follow_max_acceleration, float follow_max_applicable_curvature_radius, float follow_min_ld, float follow_max_ld, float follow_ld) {
    this->follow_max_voltage = follow_max_voltage;
    this->follow_max_acceleration = follow_max_acceleration;
    this->follow_min_ld = follow_min_ld;
    this->follow_max_ld = follow_max_ld;
    this->follow_ld = follow_ld;
    this->follow_mu = 12.0f / sqrtf(follow_max_applicable_curvature_radius);
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

float Drive::get_velocity_volts() {
    return 0.5f * rpm_to_volt(this->DriveL.velocity(rpm) + this->DriveR.velocity(rpm), this->motor_rpm);
}

float Drive::get_left_velocity_volts() {
    return rpm_to_volt(this->DriveL.velocity(rpm), this->motor_rpm);
}

float Drive::get_right_velocity_volts() {
    return rpm_to_volt(this->DriveR.velocity(rpm), this->motor_rpm);
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

void Drive::drive_time(float time, float left_voltage, float right_voltage, bool stop) {
    this->DriveL.spin(fwd, left_voltage, volt);
    this->DriveR.spin(fwd, right_voltage, volt);
    task::sleep(time);
    if (stop) {
        this->stop(brake);
    }
}

void Drive::stop(brakeType mode) {
    this->DriveL.stop(mode);
    this->DriveR.stop(mode);

    this->DriveLB.stop(mode);
    this->DriveLF.stop(mode);
    this->DriveRB.stop(mode);
    this->DriveRF.stop(mode);
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
    while(!this->stop_auton && !drivePID.is_settled(this->get_current())){
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
    while(!this->stop_auton && !swingPID.is_settled(this->get_current())){
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
    while(!this->stop_auton && !swingPID.is_settled(this->get_current())){
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

void Drive::drive_to_point(const Vector2& target, const DriveToPointConfig& config) {
    if (this->stop_auton) return;

    auto error_vec = target - this->get_position();
    float heading_offset;

    switch (config.direction) {
        case FLEXIBLE:
            if (fabsf(reduce_negative_180_to_180(error_vec.heading() - this->get_absolute_heading())) > 90)
                heading_offset = 180.f;
            else 
                heading_offset = 0.f;
            break;
        case FORWARD:
            heading_offset = 0;
            break;
        case REVERSE:
            heading_offset = 180.f;
            break;
    }

    if (config.is_rigid) {
        this->turn_to_point(target, heading_offset, TurnConfig().set_turn_max_voltage(12).set_settle_conditions(SettleConfig().set_settle_error(5).set_settle_time(20).set_timeout(500)));
        error_vec = target - this->get_position();
    }

    float heading_error = to_deg(error_vec.heading())-get_absolute_heading() + heading_offset;
    if (config.direction == FLEXIBLE)
        heading_error = reduce_negative_90_to_90(heading_error);
    else
        heading_error = reduce_negative_180_to_180(heading_error);
        
    PID drivePID(error_vec.norm(), config.drive_pid, config.settle_conditions);
    PID headingPID(heading_error, config.heading_pid);

    while(!this->stop_auton && !drivePID.is_settled(this->get_current())){
        error_vec = target - this->get_position();

        float drive_error = error_vec.norm();
        float drive_output = drivePID.compute(drive_error);

        float heading_error = to_deg(error_vec.heading()) - get_absolute_heading() + heading_offset;
        float cosine_scale = cos(to_rad(heading_error));

        if (config.direction == FLEXIBLE) {
            heading_error = reduce_negative_90_to_90(heading_error);
        } else {
            heading_error = reduce_negative_180_to_180(heading_error);
            if (config.direction == REVERSE)
                cosine_scale = -cosine_scale;
        }

        float heading_output = headingPID.compute(heading_error);

        if (drive_error < config.settle_conditions.settle_error)
            heading_output = 0;

        drive_output = cosine_scale * clamp(drive_output, -config.drive_pid.max_output, config.drive_pid.max_output);
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

    this->desired_heading = this->get_absolute_heading();
}


void Drive::drive_to_pose(Pose target, const DriveToPoseConfig& config) {
    if (this->stop_auton) return;

    this->desired_heading = target.theta;
    
    auto direction = config.direction;

    const auto end_unit_tangent = Vector2(cosf(to_rad(target.theta)), sinf(to_rad(target.theta)));
    if (direction == FLEXIBLE) {
        if (end_unit_tangent.dot(this->get_position() - target) < 0)
            direction = FORWARD;
        else
            direction = REVERSE;
    }

    float heading_offset = (direction == REVERSE) ? 180.f : 0.f;
    const auto end_tangent = (config.lead_distance * (float)direction) * end_unit_tangent;
    auto carrot = target.to_vector2() - (target - this->get_position()).norm() * end_tangent;

    PID drivePID((carrot - this->get_position()).norm(), config.drive_pid, config.settle_conditions);
    PID headingPID(reduce_negative_180_to_180(to_deg((carrot - this->get_position()).heading()) - get_absolute_heading() + heading_offset), config.heading_pid);

    while (!this->stop_auton && !drivePID.is_settled(this->get_current())) {
        carrot = target.to_vector2() - (target - this->get_position()).norm() * end_tangent;
        auto to_carrot = carrot - this->get_position();

        float heading_error = reduce_negative_180_to_180(to_deg(to_carrot.heading()) - get_absolute_heading() + heading_offset);
        float drive_error = to_carrot.norm();
        float drive_output = drivePID.compute(drive_error);
        float cosine_scale = (float)direction * cos(to_rad(heading_error));

        if (drive_error < config.settle_conditions.settle_error * 4) {
            if (drive_error < config.settle_conditions.settle_error) 
                heading_error = reduce_negative_180_to_180(target.theta - this->get_absolute_heading());
            else
                heading_error = reduce_negative_90_to_90(heading_error);
        }
        
        if (is_print_frame())
            std::cout << "drive_error = " << drive_error << ", drive_output = " << drive_output << ", cosine scale = " << cosine_scale << std::endl;
        
        float heading_output = headingPID.compute(heading_error);

        drive_output = clamp(drive_output, -config.drive_pid.max_output, config.drive_pid.max_output) * cosine_scale;
        heading_output = clamp(heading_output, -config.heading_pid.max_output, config.heading_pid.max_output);

        drive_with_voltage(drive_output - heading_output, drive_output + heading_output);

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

void Drive::follow(const std::vector<Vector2>& path, const FollowConfig& config) {
    if (stop_auton) return;
    std::cout << "\n\n\n----------------------------\n\nrunning follow" << std::endl;
    auto pure_puresuit = PurePursuit(this->get_position(), this->get_absolute_heading(), path, config);

    if (pure_puresuit.is_settled()) {
        std::cout << "Pure Pursuit started settled... wtf" << std::endl;
    }
    while (!this->stop_auton && !pure_puresuit.is_settled(this->get_current())) {
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

/*void Drive::follow(const std::vector<Vector2>& path) {
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
}*/

void Drive::follow(const std::vector<Vector2>& path, float follow_timeout, bool make_smooth, float follow_max_voltage, float heading_max_voltage, float ld, float follow_settle_error, float follow_settle_time, float follow_mu, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti) {
    if (path.empty() || this->stop_auton) return;
    if (path.size() == 1) {
        drive_to_point(path[0]);
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
    
    while (!this->stop_auton && !drivePID.is_settled(this->get_current())) {
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

void Drive::follow(const std::vector<Waypoint>& waypoints, RAMSETEConfig config, const VelocityControllerConfig& velocity_controller_config) {
    Trajectory trajectory(waypoints, config.trajectory_config.set_max_speed_in_per_sec(this->max_velocity_in_per_sec));
    VelocityController left_velocity_controller(0, velocity_controller_config);
    VelocityController right_velocity_controller(0, velocity_controller_config);
    RAMSETE ramsete(config);
    Settle settle(config.settle_conditions);
    settle.timeout += trajectory.get_duration();

    float t = 0;
    unsigned int dt = roundf(config.trajectory_config.dt);

    State target = trajectory.sample(t);
    Pose pose = this->get_pose();

    while (!this->stop_auton && !settle.is_exit(this->get_current())) {
        // get controller output
        auto steer = ramsete.compute(pose, target);
        
        // convert Ackerman inputs to differential drive inputs
        auto turn = this->width * 0.5f * steer.omega;
        auto left_velocity = steer.v - turn;
        auto right_velocity = steer.v + turn;

        // get voltage from velocity controller
        auto left_voltage = left_velocity_controller.compute(rpm_to_volt(DriveL.velocity(rpm), this->motor_rpm), left_velocity);
        auto right_voltage = right_velocity_controller.compute(rpm_to_volt(DriveR.velocity(rpm), this->motor_rpm), right_velocity);

        // drive
        this->drive_with_voltage(left_voltage, right_voltage);
        
        // update state
        task::sleep(dt);
        t += dt;

        target = trajectory.sample(t);
        pose = this->get_pose();

        settle.update(t < trajectory.get_duration() ? std::numeric_limits<float>::infinity() : (target - pose).norm()); // don't let it settle until it reaches the end of the trajectory.
    }

    if (this->stop_auton)
        this->set_brake_type(coast);
    else
        this->stop(brake);
}

void Drive::turn_to_point(const Vector2& pos, const TurnConfig& config) {
    this->turn_to_point(pos, 0, config);
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

    this->desired_heading = this->get_absolute_heading();
}
/*
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
    while(!this->stop_auton && !( drivePID.is_settled(this->get_holonomic_current()) && turnPID.is_settled(this->get_holonomic_current()) ) ){
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
}*/

void Drive::holonomic_drive_to_point(const Vector2& pos, const HolonomicDriveToPointConfig& config) {
    this->holonomic_drive_to_point({pos, this->desired_heading}, config);
}

void Drive::holonomic_drive_to_point(const Pose& pose, const HolonomicDriveToPointConfig& config) {
    if (this->stop_auton) return;

    auto error = pose - this->get_pose();
    PID drivePID(error.norm(), config.drive_pid, config.drive_settle_conditions);
    PID turnPID(reduce_negative_180_to_180(error.theta), config.turn_pid, config.turn_settle_conditions);
    
    while(!this->stop_auton && !( drivePID.is_settled(this->get_holonomic_current()) && turnPID.is_settled(this->get_holonomic_current()) ) ){
        auto error = pose - this->get_pose();
        
        float drive_error = error.norm();
        float turn_error = reduce_negative_180_to_180(error.theta);

        float drive_output = drivePID.compute(drive_error);
        float turn_output = turnPID.compute(turn_error);

        drive_output = clamp(drive_output, -config.drive_pid.max_output, config.drive_pid.max_output);
        turn_output = clamp(turn_output, -config.turn_pid.max_output, config.turn_pid.max_output);

        auto cosine_scale = fabsf(cosf(2 * to_rad(turn_error)));
        auto drive_vector = error.rescale(drive_output * 0.5f * cosine_scale);

        auto cos_theta = cosf(to_rad(this->get_absolute_heading()));
        auto sin_theta = sinf(to_rad(this->get_absolute_heading()));
        
        auto NE_component = drive_vector.y + drive_vector.x;
        auto NW_component = drive_vector.y - drive_vector.x;

        auto NW_output = cos_theta * NE_component + sin_theta * NW_component;
        auto NE_output = sin_theta * NE_component - cos_theta * NW_component;

        DriveLF.spin(fwd, NE_output + turn_output, volt);
        DriveLB.spin(fwd, NW_output + turn_output, volt);
        DriveRB.spin(fwd, NE_output - turn_output, volt);
        DriveRF.spin(fwd, NW_output - turn_output, volt);
        task::sleep(10);
    }

    if (!this->stop_auton) {
        DriveLF.stop(hold);
        DriveLB.stop(hold);
        DriveRB.stop(hold);
        DriveRF.stop(hold);
    } else {
        DriveLF.setStopping(coast);
        DriveLB.setStopping(coast);
        DriveRB.setStopping(coast);
        DriveRF.setStopping(coast);
    }
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