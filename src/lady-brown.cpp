#include "vex.h"
#include <iostream>

LadyBrown::LadyBrown(motor *lb_motor, rotation *rotation_sensor):
    lb_motor(lb_motor), rotation_sensor(rotation_sensor)
{   
    std::cout << "lady brown initialized" << std::endl;
}

void LadyBrown::update() {
    float target;
    switch (this->state) {
        case LB_REST:
            target = this->rest_position;
            break;
        case LB_IDLE:
            target = this->idle_position;
            break;
        case LB_SCORE:
            target = this->score_position;
            break;
    }

    auto error = reduce_0_to_360(target - this->rotation_sensor->position(deg) + 30) - 30;
    auto speed = this->lb_pid.compute(error);
    
    if (this->lb_motor->current() < 2.5f) {
        this->lb_motor->spin(forward, speed, volt);
        this->lb_motor->setStopping(brake);
        if (this->state == LB_IDLE && fabsf(error) < 15.f) {
            this->lb_motor->setStopping(coast);
            this->lb_motor->spin(fwd, fminf(0, speed), volt);
        }
    } else {
        if (this->state == LB_REST)
            this->state = LB_IDLE;
        this->lb_motor->spin(forward, 0, volt);
    }
}

void LadyBrown::rest() {
    this->state = LB_REST;
}

void LadyBrown::idle() {
    this->state = LB_IDLE;
}

void LadyBrown::score() {
    if (this->state == LB_IDLE)
        this->state = LB_SCORE;
}

LadyBrownState LadyBrown::get_state() const {
    return this->state;
}