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
        filter_mode(new RingColor (NO_RINGS)) {}

void Intake::update() {
    // detect rings
    this->color_sensor->setLightPower(100);
    this->color_sensor->setLight(ledState::on);
    //this->update_color_detections();
    auto color_detected = this->detected_color();
    bool detecting_ring = color_detected != NO_RINGS;

    auto position = this->get_position();
    this->direction = *this->target_velocity > 0 ? 1 : *this->target_velocity < 0 ? -1 : 0;
    
    if (detecting_ring) { // currently detecting ring
        this->current_ring_color = color_detected;

        if (!this->detected_ring) {
            // should detect most of the way to the top, we want to go the next time that the ring passes the eject position
            // assume that the hook is about 18.5 < (distance between things) links from the bottom
            auto local_position = fmodf(position, this->links_per_hook); // chain local position
            if (local_position < this->detect_position_ref - 8) {
                local_position += this->links_per_hook;
            }

            this->bottom_position = position - local_position;
            this->eject_position = this->bottom_position + this->eject_position_ref;
            this->score_position = this->bottom_position + this->top_position_ref;
            if (this->detected_color() == *this->filter_mode) {
                this->ejection_pending = true;
            }
        }
    }

    float ring_position = this->get_ring_position();

    // check if ring left the intake from the top or bottom
    if (position > this->score_position || position < this->bottom_position) {
        this->clear_ring();
    }

    switch (this->state) {
        case IntakeState::CALIBRATING:
            if (detecting_ring) {
                this->intake_motor->setPosition(this->detect_position_ref / this->links_per_rev, rev);
                this->set_filter_mode_from_current_ring();
                this->intake_motor->setVelocity(50, pct);
                this->intake_motor->setStopping(hold);
                this->intake_motor->spinTo(0, rev, true);
                this->state = IntakeState::INPUT;
            } else {
                this->intake_motor->spin(forward, this->calibration_velocity_pct, percent);
            }
            break;        
        case IntakeState::UNJAMMING:
            this->intake_motor->spin(fwd, -100, pct);
            if (ring_position > this->previous_ring_position + 0.5f * this->links_per_hook)
                this->state = IntakeState::INPUT;
            break;
        default:
            // reversing conditions
            if (this->ejection_pending && position >= this->eject_position - (this->intake_motor->velocity(rpm) / 6000) * this->links_per_rev && this->direction == 1) {
                this->reverse_end_time = vex::timer::system() + 188;
                this->state = IntakeState::EJECTING;
                this->clear_ring();
            }

            if (vex::timer::system() < this->reverse_end_time || (this->ejection_pending && this->direction == 1)) {
                this->intake_motor->setStopping(brake);
                this->intake_motor->spin(forward, 5.375 * (this->eject_position - position), volt);
                this->velocity_error_buildup = 0.f;
            } else {
                this->intake_motor->spin(forward, *this->target_velocity, volt);

                this->velocity_error_buildup = lerp<float>(
                    this->velocity_error_buildup, 
                    *this->target_velocity - rpm_to_volt(this->intake_motor->velocity(rpm), 
                                                        get_rpm(*this->intake_motor)), 
                                                        0.2);
                // check if the intake has been stuck
                if (fabsf(this->velocity_error_buildup) > 6) {
                    this->state = IntakeState::UNJAMMING;
                }
            }
            break;
    }

    this->previous_ring_position = ring_position;
    // update state
    this->detected_ring = detecting_ring;

    /*if (vex::timer::system() / 10 % 10 == 0) {
        std::cout << ring_position << ", is calibrating = " << (this->state == IntakeState::CALIBRATING) << ", is unjamming = " << (this->state == IntakeState::UNJAMMING) << ", is ejecting = " << (vex::timer::system() < this->reverse_end_time) << ", filter = " << *this->filter_mode << std::endl;
    }*/
}

void Intake::intake() {
    this->state = IntakeState::INPUT;
    *this->target_velocity = 12;
}

void Intake::outtake() {
    this->state = IntakeState::INPUT;
    *this->target_velocity = -12;
}

void Intake::pickup() {
    this->intake_motor->setVelocity(100., pct);
    this->intake_motor->spinTo(this->get_position() + this->links_per_hook / this->links_per_rev, rev, false);
}

bool Intake::sees_red() const {
    return this->color_sensor->isNearObject() && this->red_hue.in_window(this->color_sensor->hue());
}

bool Intake::sees_blue() const {
    return this->color_sensor->isNearObject() && this->blue_hue.in_window(this->color_sensor->hue());
}

float Intake::get_position() {
    return this->intake_motor->position(rev) * this->links_per_rev; 
}

RingColor Intake::detected_color() const {
    if (!this->color_sensor->isNearObject()) return NO_RINGS;
    auto hue = this->color_sensor->hue();
    if (this->red_hue.in_window(hue)) return RED_RINGS;
    if (this->blue_hue.in_window(hue)) return BLUE_RINGS;
    return NO_RINGS;
}

void Intake::clear_ring() {
    this->current_ring_color = NO_RINGS;
    this->ejection_pending = false;
    if (this->state == IntakeState::EJECTING)
        this->state = IntakeState::INPUT;
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

RingColor Intake::get_current_ring_color() const {
    return this->current_ring_color;
}

bool Intake::has_ring() const {
    return this->current_ring_color != NO_RINGS;
}

void Intake::set_filter_mode(RingColor color) {
    *this->filter_mode = color;
}

void Intake::set_filter_mode_from_current_ring() {
    *this->filter_mode = this->current_ring_color == RED_RINGS ? BLUE_RINGS : this->current_ring_color == BLUE_RINGS ? RED_RINGS : NO_RINGS;
}

bool Intake::is_spinning() const {
    return fabsf(this->intake_motor->velocity(rpm)) > 10;
}

void Intake::off() {
    this->target_velocity = 0;
    this->state = IntakeState::INPUT;
    this->intake_motor->spin(fwd, 0, volt);
}

void Intake::set_velocity(float velocity) {
    if (this->state == IntakeState::CALIBRATING && velocity == 0)
        return;
    this->state = IntakeState::INPUT;
    *this->target_velocity = velocity;
}

void Intake::calibrate() {
    this->state = IntakeState::CALIBRATING;
}

float Intake::get_ring_position() {
    auto position = this->get_position();
    if (this->current_ring_color != NO_RINGS)
        return position - this->bottom_position;
    else
        return fmod(position, this->links_per_hook);
}