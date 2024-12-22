#include "vex.h"

MogoClamp::MogoClamp(digital_out *mogo_clamp_piston, distance *distance_sensor):
        mogo_clamp_piston(mogo_clamp_piston), distance_sensor(distance_sensor) {

}

void MogoClamp::update(bool state, bool override) {
    auto sees_goal = this->distance_sensor->isObjectDetected() && this->distance_sensor->objectDistance(inches) <= this->max_clamp_distance;
    if (!state) { // open
        this->holding_mogo = false;
        this->open();
    }
    // close
    else if (state && this->mogo_clamp_piston->value() == OPEN) {
        if (override || sees_goal) {
            this->close();
            this->holding_mogo = sees_goal;
        }
    }

    if (this->holding_mogo && (!this->distance_sensor->isObjectDetected() || this->distance_sensor->objectDistance(inches) > max_clamp_distance)) {
        Controller1.rumble("-");
        this->holding_mogo = false;
    } else if (this->mogo_clamp_piston->value() == OPEN && sees_goal) {
        Controller1.rumble(".");
    }

}

void MogoClamp::open() {
    this->mogo_clamp_piston->set(OPEN);
}

void MogoClamp::close() {
    this->mogo_clamp_piston->set(!OPEN);
}