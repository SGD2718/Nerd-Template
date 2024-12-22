#include "vex.h"
#include <algorithm>

PurePursuit::PurePursuit(Vector2 robot_position, float robot_heading, std::vector<Vector2> waypoints, FollowConfig config): 
    config(std::move(config)), 
    waypoints(waypoints),
    drivePID(0, config.drive_pid),
    headingPID(0, config.heading_pid)
{
    this->waypoints = generate_discrete_g2_path(this->waypoints, this->config.point_spacing);
    this->generate_velocity_profile();
    this->update_nearest_point(robot_position, this->config.max_start_deviation);
    auto ld = this->get_look_ahead_distance(this->get_target_velocity());
    this->update_look_ahead_point(robot_position, ld, this->config.max_start_deviation);
    drivePID.previous_error = this->get_remaining_distance();
    headingPID.previous_error = this->get_heading_error(robot_position, robot_heading);
}

void PurePursuit::compute(Vector2 robot_position, float robot_heading, float &drive_output, float &heading_output) {
    this->time_spent_running += 10.0f;
    float old_error = this->drivePID.previous_error;

    // update points
    this->update_nearest_point(robot_position, this->config.max_change_per_update);
    auto max_drive_output = this->get_target_velocity();
    auto ld = get_look_ahead_distance(max_drive_output);
    this->update_look_ahead_point(robot_position, ld, this->config.max_change_per_update);

    // log points
    this->position_log.emplace_back(robot_position);
    this->look_ahead_log.emplace_back(this->get_look_ahead_point());
    this->nearest_log.emplace_back(this->get_nearest_point());

    // cosine scaling
    auto heading_error = this->get_heading_error(robot_position, robot_heading);
    auto cosine_scale = cosf(to_rad(heading_error));

    switch (this->config.direction) {
        case FORWARD:
            cosine_scale = fmaxf(0.0f, cosine_scale);
            break;
        case REVERSE:
            cosine_scale = fminf(0.0f, -cosine_scale);
            break;
        default:
            heading_error = reduce_negative_90_to_90(heading_error);
            break;
    }

    // compute output
    const auto drive_error = this->get_drive_error(robot_position);
    const auto max_accel = fabsf(drive_error - old_error) / this->config.acceleration_distance * 12.0f;

    drive_output = clamp(
        this->drivePID.compute(drive_error), 
        -max_drive_output, 
        fminf(max_drive_output, this->previous_drive_output + max_accel)) * cosine_scale;

    this->previous_drive_output = drive_output;

    heading_output = clamp(
        this->headingPID.compute(heading_error), 
        -this->config.heading_pid.max_output, 
        this->config.heading_pid.max_output);

    // check settle conditions
    if (fabsf(drive_error) <= this->config.settle_conditions.settle_error) {
        this->time_spent_settled += 10;
        heading_output = 0;
    } else {
        this->time_spent_settled = 0;
    }
}   

Vector2 PurePursuit::get_look_ahead_point() const {
    if (this->look_ahead_index > 0)
        return lerp<Vector2>(this->waypoints[this->look_ahead_index - 1], this->waypoints[this->look_ahead_index], this->look_ahead_interpolation_parameter);
    return this->waypoints.front();
}

Vector2 PurePursuit::get_nearest_point() const {
    if (this->nearest_index > 0)
        return lerp<Vector2>(this->waypoints[this->nearest_index - 1], this->waypoints[this->nearest_index], this->nearest_interpolation_parameter);
    return this->waypoints.front();
}

float PurePursuit::get_remaining_distance() const {
    if (this->nearest_index > 0)
        return ((float)(this->waypoints.size() - this->nearest_index + 1) - this->nearest_interpolation_parameter) * this->config.point_spacing;
    return this->waypoints.size() * this->config.point_spacing;
}

float PurePursuit::get_target_velocity() const {
    if (this->nearest_index > 0)
        return lerp<float>(
            this->velocity_profile[this->nearest_index - 1], 
            this->velocity_profile[this->nearest_index], 
            this->nearest_interpolation_parameter);
    return this->velocity_profile.front();
}

float PurePursuit::get_look_ahead_distance() const {
    return this->config.max_ld * this->get_target_velocity() / 12.0f;
}

float PurePursuit::get_look_ahead_distance(float velocity) const {
    return this->config.max_ld * velocity / 12.0f;
}

float PurePursuit::get_heading_error(Vector2 robot_position, float robot_heading) const {
    const auto look_ahead_point = this->get_look_ahead_point();
    const auto target_heading = to_deg((look_ahead_point - robot_position).heading()) + (this->config.direction == REVERSE ? 180 : 0);
    return reduce_negative_180_to_180(target_heading - robot_heading);
}

float PurePursuit::get_drive_error(Vector2 robot_position) const {
    if (this->look_ahead_index < this->waypoints.size()-1)
        return this->get_remaining_distance();
    return (robot_position - this->waypoints.back()).norm();
}

bool PurePursuit::is_settled() const {
    return 
        this->time_spent_settled >= this->config.settle_conditions.settle_time ||
        this->time_spent_running >= this->config.settle_conditions.timeout ||
        chassis.DriveL.current() >= this->config.settle_conditions.max_current ||
        chassis.DriveR.current() >= this->config.settle_conditions.max_current;
}

void PurePursuit::generate_velocity_profile() {
    this->velocity_profile = std::vector<float>(this->waypoints.size());
    int i;
    for (i = 1; i < this->waypoints.size() - 1; ++i) {
        const auto r = compute_discrete_radius_of_curvature(this->waypoints[i-1], this->waypoints[i], this->waypoints[i+1]);
        this->velocity_profile[i] = clamp(this->config.drive_pid.max_output * r / this->config.max_full_speed_turn_radius, this->config.drive_min_voltage, this->config.drive_pid.max_output);
    }
    
    this->velocity_profile[0] = this->velocity_profile[1];
    this->velocity_profile.back() = 0;

    // deceleration
    const auto max_accel = 12.0f * this->config.point_spacing / this->config.acceleration_distance;
    for (i = this->velocity_profile.size() - 2; i >= 0; --i) {
        this->velocity_profile[i] = fminf(this->velocity_profile[i], this->velocity_profile[i+1] + max_accel);
    }
}

void PurePursuit::update_nearest_point(Vector2 robot_position, float max_change) {
    const auto max_index_change = (int)(max_change / this->config.point_spacing + 0.5);
    const auto min_index = std::max(this->nearest_index - max_index_change, 1);
    const auto max_index = std::min(this->nearest_index + max_index_change, (int)this->waypoints.size() - 1);

    float least_error = std::numeric_limits<float>::infinity();

    if (this->nearest_index == 0)
        least_error = (robot_position - this->waypoints.front()).norm();

    for (int i = min_index; i <= max_index; ++i) {
        auto path_direction_vector = this->waypoints[i] - this->waypoints[i-1];
        auto vector_to_robot = robot_position - this->waypoints[i-1];

        // make sure the nearest point is on the line segment
        if (vector_to_robot.dot(path_direction_vector) >= 0 && (robot_position-this->waypoints[i]).dot(path_direction_vector) <= 0) {
            // check error
            auto error = fabsf(vector_to_robot.orthogonal_comp(path_direction_vector));
            if (error < least_error) {
                this->nearest_index = i;
                least_error = error;
            }
        }
    }

    // compute interpolation parameter
    if (this->nearest_index > 0) {
        const auto anchor = this->waypoints[this->nearest_index-1];
        const auto vector_to_robot = robot_position - anchor;
        const auto path_direction_vector = this->waypoints[this->nearest_index] - anchor;
        const auto projection = vector_to_robot.proj(path_direction_vector);
        this->nearest_interpolation_parameter = (projection.x + projection.y) / (path_direction_vector.x + path_direction_vector.y);
    } else {
        this->nearest_interpolation_parameter = 0;
        return;
    }
}

void PurePursuit::update_look_ahead_point(Vector2 robot_position, float ld, float max_change) {
    auto distance_change = 0;
    auto ld_squared = ld * ld;

    auto max_index = std::min(this->look_ahead_index + (int)(max_change / this->config.point_spacing + 0.5), (int)this->waypoints.size() - 1);
    while ((robot_position - this->waypoints[this->look_ahead_index]).norm_squared() < ld_squared && this->look_ahead_index <= max_index) {
        ++this->look_ahead_index;
    }

    if (this->look_ahead_index > 0) {
        bool found;

        Vector2 look_ahead_point = line_circle_intersection(
            this->waypoints[this->look_ahead_index-1], 
            this->waypoints[this->look_ahead_index], 
            robot_position, 
            ld, 
            &found,
            &this->look_ahead_interpolation_parameter);

        if (!found && this->look_ahead_index == max_index) {
            this->look_ahead_interpolation_parameter = 1.0;
            std::cerr
                << "Not enough room to find look-ahead point. Distance to next point is " 
                << (this->waypoints[this->look_ahead_index] - robot_position).norm() 
                << ", ld = " << ld 
                << ", robot position = " << robot_position.latex()
                << std::endl;
        }
        
        // for debugging purposes only
        if (!found && this->look_ahead_index != max_index) {
            std::cerr 
                << "Look-ahead point is too far away. Distance to next point is " 
                << (this->waypoints[this->look_ahead_index] - robot_position).norm() 
                << ", ld = " << ld 
                << ", robot position = " << robot_position.latex()
                << std::endl;
        }
    }
}

void PurePursuit::print_logs() const {
    std::cout << "P_{ath} = \\left[";
    for (auto &v: this->waypoints) {
        std::cout << v.latex() << ",";
    }
    std::cout << "\b\\right]\n";
    
    std::cout << "R_{obot} = \\left[";
    for (auto &v: this->position_log) {
        std::cout << v.latex() << ",";
    }
    std::cout << "\b\\right]\n";

    std::cout << "L_{a} = \\left[";
    for (auto &v: this->look_ahead_log) {
        std::cout << v.latex() << ",";
    }
    std::cout << "\b\\right]\n";

    std::cout << "N_{earest} = \\left[";
    for (auto &v: this->nearest_log) {
        std::cout << v.latex() << ",";
    }
    std::cout << "\b\\right]" << std::endl;
}