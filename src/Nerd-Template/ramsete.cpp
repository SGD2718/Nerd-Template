#include "vex.h"

RAMSETE::RAMSETE(float b, float zeta) : 
        b(b), 
        zeta(zeta), 
        k2_b(b), 
        k3_b(b) {}

RAMSETE::RAMSETE(RAMSETEConfig config) : 
        b(config.b), 
        zeta(config.zeta), 
        k2_b(config.k2_b_weight * b), 
        k3_b(config.k3_b_weight * b), 
        k2_v_weight(config.k2_v_weight), 
        k3_omega_weight(config.k3_omega_weight) {}

SteerCommand RAMSETE::compute(const Pose &robot_pose, const State &target) const {
    // compute error
    auto error = target - robot_pose;
    error = error.rotate(to_rad(robot_pose.theta));

    // compute dynamic gains
    const auto k1 = 2.0f * this->zeta * sqrtf(target.omega * target.omega + this->b * target.v * target.v);
    const auto k2 = this->k2_b + this->k2_v_weight * target.v;
    const auto k3 = this->k3_b + this->k3_omega_weight * target.omega;

    // compute outputs
    const auto velocity = target.v * cosf(error.theta) + k1 * error.x;

    const auto theta_factor = (fabsf(error.theta) < 1e-4f) ? 1.f - error.theta * error.theta / 6.f : sinf(error.theta) / error.theta; // used taylor series for numerical stability for small θ
    const auto angular_velocity = target.omega + k2 * target.v * theta_factor + k3 * error.theta;

    return {velocity, angular_velocity};
}