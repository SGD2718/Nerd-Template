#pragma once

#include "vex.h"

class RAMSETE {
public:
    RAMSETE(const RAMSETEConfig &config = {});

    SteerCommand compute(Pose robot_pose, State target) const;

    Direction direction = FLEXIBLE;
private:
    float b;
    float zeta;
    float m_per_sec_to_target_velocity_ratio;
};