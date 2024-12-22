#pragma once

//#include "vex.h"
#include "vector2.h"
#include <vector>
#include "config-structs.h"
#include "PID.h"

//class PID;
//struct FollowConfig;

class PurePursuit {
public:
    PurePursuit(Vector2 robot_position, float robot_heading, std::vector<Vector2> waypoints, FollowConfig config);

    void compute(Vector2 robot_position, float robot_heading, float &driveOutput, float &headingOutput);

    Vector2 get_look_ahead_point() const;
    Vector2 get_nearest_point() const;
    float get_remaining_distance() const;
    float get_target_velocity() const;
    float get_look_ahead_distance() const;
    bool is_settled(float current = 0.0f) const;

    void print_logs() const;

    FollowConfig config;
private:
    int look_ahead_index = 0; // index of the endpoint of the segment containing the look ahead point
    int nearest_index = 0;    // index of the endpoint of the segment nearest to the robot
    float look_ahead_interpolation_parameter = 0;
    float nearest_interpolation_parameter = 0;
    float previous_drive_output = 0;

    PID drivePID;
    PID headingPID;
    Settle settle;

    float get_look_ahead_distance(float velocity) const;
    float get_heading_error(Vector2 robot_position, float robot_heading) const;
    float get_drive_error(Vector2 robot_position) const;

    void generate_velocity_profile();
    void compute_distances();

    void update_nearest_point(Vector2 robot_position, float max_change);
    void update_look_ahead_point(Vector2 robot_position, float ld, float max_change);

    std::vector<Vector2> waypoints;
    std::vector<float> velocity_profile{};

    std::vector<Vector2> position_log{};
    std::vector<Vector2> look_ahead_log{};
    std::vector<Vector2> nearest_log{};
};