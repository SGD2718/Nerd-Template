#pragma once
#include "vex.h"

class MogoClamp {
public:
    MogoClamp(digital_out* mogo_clamp_piston, distance* distance_sensor);

    void update(bool state, bool override = false);

    void close();
    void open();

    digital_out *mogo_clamp_piston;
    distance *distance_sensor;

private:
    static const bool OPEN = true;

    bool target_state = !OPEN;
    bool holding_mogo = false;
    float max_clamp_distance = 7.25; // TODO: tune this
};