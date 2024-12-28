#pragma once
#include "vex.h"

enum LadyBrownState {
    LB_REST,     // out of the way
    LB_IDLE,     // ready to receive ring
    LB_SCORE,    // in the middle of scoring the ring
};

class LadyBrown {
public:
    LadyBrown(motor *lb_motor, rotation *rotation_sensor);

    void update();

    void rest();
    void idle();
    void score();
    
    LadyBrownState get_state() const;
    
private:
    LadyBrownState state = LB_REST;
    motor *lb_motor;
    rotation *rotation_sensor;

    // TODO: Tune these
    PID lb_pid{0, 0.6, 0, 0, 0};

    float rest_position = 0;     // resting
    float score_position = -165;  // position to score on wall stake
    float idle_position = -26;    // ready for ring
};