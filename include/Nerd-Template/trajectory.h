#pragma once
#include "vex.h"
#include <vector>

class Trajectory {
public:
    Trajectory(const std::vector<Waypoint>& waypoints, const TrajectoryConfig& config);
    [[nodiscard]] State sample(float t) const;
    [[nodiscard]] float get_duration() const;
private:
    struct UnprocessedTrajectory {
        float kv;                                   // curvature gain
        float velocity_ratio;                       // actual velocity (in / update) / target velocity
        TrajectoryConfig config;                    // configuration
        std::vector<float> velocity_keyframes{};    // velocity keyframes isolated
        std::vector<CubicBezier> path{};            // path as a list of cubic Bezier splines (should be natural cubic Bezier splines for C2 continuity)
    };

    void generate_from_raw(UnprocessedTrajectory& raw);
    
    static State compute_state(float u, float velocity, UnprocessedTrajectory& raw);
    static float compute_velocity(float u, float s, float segment_length, float remaining_distance, UnprocessedTrajectory& raw);
    static float advance_param_by_distance(float u_start, float ds, const UnprocessedTrajectory& raw);
    static std::vector<float> compute_centripetal_parameter(const std::vector<Vector2> &points);
    static void compute_natural_cubic_spline(const std::vector<float> &t, const std::vector<float> &y, std::vector<float> &a, std::vector<float> &b, std::vector<float> &c, std::vector<float> &d);
    static CubicBezier polynomial_segment_to_bezier(float ax, float bx, float cx, float dx, float ay, float by, float cy, float dy, float t0, float t1);
    static std::vector<CubicBezier> compute_cubic_beziers_from_waypoints(const std::vector<Vector2> &waypoints);

    float dt;             // change in time between updates
    float duration{};     // expected total path time

    std::vector<State> trajectory{};
};