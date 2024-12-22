#pragma once

#include "vex.h"

class RAMSETE {
public:
    RAMSETE() = default;
    RAMSETE(float b = 2.0, float zeta = 0.7);
    RAMSETE(RAMSETEConfig config);

    SteerCommand compute(const Pose &robot_pose, const State &target) const;

private:
    float b = 2.0f;
    float zeta = 0.7f;
    float k2_b = 2.0f;
    float k3_b = 2.0f;
    float k2_v_weight = 0.0f;
    float k3_omega_weight = 0.0f;
};