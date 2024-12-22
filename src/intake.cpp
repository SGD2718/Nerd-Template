#include "vex.h"

bool HueWindow::in_window(float hue) const {
    return 
        (this->min_hue <= hue && hue <= this->max_hue) || 
        (this->min_hue <= hue - 360 && hue - 360 <= this->max_hue);
}

Intake::Intake(motor* intake_motor, optical* color_sensor, digital_out* intake_piston):
        intake_motor(intake_motor),
        color_sensor(color_sensor),
        intake_piston(intake_piston),
        target_velocity(new float (0.0)),
        filter_mode(new RingColor (ALL_RINGS)) {
    std::cout << "intake initialized" << std::endl;
}

void Intake::update() {
    // detect rings
    this->color_sensor->setLightPower(100);
    this->color_sensor->setLight(ledState::on);
    this->update_color_detections();
    auto color_detected = this->detected_color();
    bool detecting_ring = color_detected != NO_RINGS;

    this->avg_current = this->avg_current * 0.95 + this->intake_motor->current() * 0.05;
    /*if ((vex::timer::system() / 10) % 10 == 1)
        std::cout << "intake current = " << avg_current << std::endl;*/
    
    if (detecting_ring) { // currently detecting ring
        this->current_ring_color = color_detected;

        if (!this->detected_ring) {
            if (this->detected_color() == *this->filter_mode) {
                this->rotation_to_eject = this->intake_motor->position(rev) + 0.63;
                this->ejection_pending = true;
            }
        }
    } else if (this->detected_ring && !detecting_ring) { // ring just left detection
        if (this->direction != -1) { // paused or upward
            this->rotation_when_ring_lost = intake_motor->position(rev) + this->ejection_revs;
        } else {
            this->current_ring_color = NO_RINGS;
        }
    } else if (this->intake_motor->position(rev) > this->rotation_when_ring_lost) { // ring was scored
        this->current_ring_color = NO_RINGS;
        this->rotation_when_ring_lost = std::numeric_limits<float>::lowest();
    }

    // reversing conditions
    if (this->ejection_pending && this->intake_motor->position(rev) > this->rotation_to_eject && this->direction == 1 && this->intake_motor->velocity(rpm) > 0)
        this->time_to_stop_reversing = vex::timer::system() + 333;
    //else if (this->avg_current > 2.5)
        //this->time_to_stop_reversing = vex::timer::system() + 333;

    // spin
    if (vex::timer::system() < this->time_to_stop_reversing) {
        if (this->ejection_pending) {
            this->ejection_pending = false;
            this->ring_going_up = false;
            this->current_ring_color = NO_RINGS;
            this->rotation_when_ring_lost = std::numeric_limits<float>::lowest();
        }
        this->intake_motor->spin(forward, -12, volt);
    } else {
        this->ring_going_up = this->current_ring_color != NO_RINGS;
        this->intake_motor->spin(forward, *this->target_velocity, volt);
    }

    // update state
    this->detected_ring = detecting_ring;
    this->direction = *this->target_velocity > 0 ? 1 : *this->target_velocity < 0 ? -1 : 0;
/*
    if ((vex::timer::system() / 10) % 10 == 1) {
        std::cout 
            //<< "direction = " << this->direction
            << "filter = " << this->filter_mode
            << ", reading = " << this->detected_color()
            << ", color on intake = " << this->current_ring_color << std::endl;
            //<< ", reverse time remaining = " << this->time_to_stop_reversing - vex::timer::system()
            //<< ", current = " << this->intake_motor->current() << std::endl;
    }*/
}

void Intake::intake() {
    *this->target_velocity = 12;
    this->intake_motor->spin(forward, 12, volt);
}

void Intake::outtake() {
    *this->target_velocity = -12;
    this->intake_motor->spin(forward, -12, volt);
}

void Intake::pickup() {
    this->intake_motor->setVelocity(100., pct);
    this->intake_motor->spinFor(forward, 1.75, rev, false);
};

void Intake::update_color_detections() {
    float red_reading = 0, blue_reading = 0, detection_reading = 0;
    if (this->color_sensor->isNearObject()) {
        auto reading = this->color_sensor->hue();
        if (this->blue_hue.in_window(reading)) { // blue
            detection_reading = 1.0f;
            red_reading = 0.0f;
            blue_reading = 1.0f;
        } else if (this->red_hue.in_window(reading)) { // red
            detection_reading = 1.0f;
            red_reading = 1.0f;
            blue_reading = 0.0f;
        }
    }

    this->detection_confidence = lerp<float>(this->detection_confidence, detection_reading, this->new_reading_weight);
    this->red_confidence = lerp<float>(this->red_confidence, red_reading, this->new_reading_weight);
    this->blue_confidence = lerp<float>(this->blue_confidence, blue_reading, this->new_reading_weight);
}

bool Intake::sees_red() const {
    return this->color_sensor->isNearObject() && this->red_hue.in_window(this->color_sensor->hue());
}

bool Intake::sees_blue() const {
    return this->color_sensor->isNearObject() && this->blue_hue.in_window(this->color_sensor->hue());
}

RingColor Intake::detected_color() const {
    if (this->sees_red()) return RED_RINGS;
    if (this->sees_blue()) return BLUE_RINGS;
    return NO_RINGS;
}

void Intake::clear_ring() {
    this->current_ring_color = NO_RINGS;
    this->rotation_when_ring_lost = std::numeric_limits<float>::lowest();
    this->ring_going_up = false;
}

void Intake::on_jam(void (*callback)()) {
    this->on_jam_callback = callback;
}

void Intake::raise() {
    this->intake_piston->set(true);
}

void Intake::lower() {
    this->intake_piston->set(false);
}

void Intake::set_raised(bool is_raised) {
    this->intake_piston->set(is_raised);
}

bool Intake::has_ring_at_top() const {
    return this->detected_ring;
}

RingColor Intake::get_current_ring_color() const {
    return this->current_ring_color;
}

bool Intake::has_ring_going_up() const {
    return this->ring_going_up;
}

void Intake::set_filter_mode(RingColor color) {
    *this->filter_mode = color;
}

void Intake::set_filter_mode_current() {
    *this->filter_mode = this->current_ring_color == RED_RINGS ? BLUE_RINGS : this->current_ring_color == BLUE_RINGS ? RED_RINGS : ALL_RINGS;
}

bool Intake::is_spinning() const {
    return fabsf(this->intake_motor->velocity(rpm)) > 10;
}

void Intake::off() {
    this->target_velocity = 0;
    this->intake_motor->spin(fwd, 0, volt);
}

void Intake::set_velocity(float velocity) {
    *this->target_velocity = velocity;
}