#include "vex.h"

RAMSETE::RAMSETE(const RAMSETEConfig& config) : 
        b(config.b),
        zeta(config.zeta),
        direction(config.direction),
        m_per_sec_to_target_velocity_ratio(
            config.trajectory_config.max_target_velocity / 
            (config.trajectory_config.max_speed_in_per_sec * 0.0254)) {}

SteerCommand RAMSETE::compute(Pose robot_pose, State target) const {
    robot_pose.theta = to_rad(robot_pose.theta);
    
    if (this->direction == REVERSE) {
        target.v = -target.v;
        target.theta += M_PI;
    }

    // compute error in meters and radians
    auto error = target - robot_pose;
    error = 0.0254f * error.rotate(-robot_pose.theta);
    target.v *= 0.0254f;

    // compute dynamic gain
    const auto k = 2.0f * this->zeta * sqrtf(target.omega * target.omega + this->b * target.v * target.v);

    // compute outputs
    const auto velocity = target.v * cosf(error.theta) + k * error.x;

    const auto theta_factor = (fabsf(error.theta) < 1e-4f) ? 1.f - error.theta * error.theta / 6.f : sinf(error.theta) / error.theta; // used taylor series for numerical stability for small θ
    const auto angular_velocity = target.omega + k * error.theta + this->b * target.v * theta_factor * error.y;

    return {velocity * this->m_per_sec_to_target_velocity_ratio, angular_velocity};
}