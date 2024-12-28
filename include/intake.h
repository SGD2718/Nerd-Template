#pragma once

#include "vex.h"

enum RingColor {
    NO_RINGS = 0b00,
    RED_RINGS = 0b01,
    BLUE_RINGS = 0b10,
};

enum class IntakeState {
    INPUT = 0,
    EJECTING = 1,
    UNJAMMING = 2,
    CALIBRATING = 3
};

struct HueWindow {
    float min_hue;
    float max_hue;

    bool in_window(float hue) const;
};

class Intake {
public:
    Intake(Intake&) = default;
    Intake(Intake&&) = default;
    Intake(motor* intake_motor, optical* color_sensor, digital_out* intake_piston);

    void set_filter_mode(RingColor exclude_color);
    void set_filter_mode_from_current_ring();

    void set_velocity(float velocity);
    void set_raised(bool is_raised);
    void update();
    void raise();
    void lower();
    void clear_ring();

    RingColor get_current_ring_color() const;
    
    bool has_ring() const;
    bool is_spinning() const;

    void pickup();
    void intake();
    void outtake();
    void off();
    void calibrate();

    float get_ring_position();

    RingColor *filter_mode;
    motor* intake_motor;
    optical* color_sensor;
    digital_out* intake_piston;
private:
    float get_position();

    // reading colors
    RingColor detected_color() const;
    bool sees_red() const;
    bool sees_blue() const;

    IntakeState state = IntakeState::INPUT;
    int direction = 0;
    float velocity_error_buildup = 0;
    float previous_ring_position = 0;
    float *target_velocity = 0;

    bool detected_ring = false;
    bool ejection_pending = false;
    //bool is_unjamming = false;
    RingColor current_ring_color = NO_RINGS;

    // constants
    HueWindow red_hue{5.f, 90.f};
    HueWindow blue_hue{150.f, 230.f};

    const float eject_position_ref = 29.5; // TODO: tune this
    const float detect_position_ref = 18.5; // TODO: tune this
    const float top_position_ref = 34;
    const float links_per_hook = 21;
    const float links_per_rev = 12;
    const float calibration_velocity_pct = 4.f;

    unsigned int reverse_end_time = 0;
    float bottom_position = 0;
    float score_position = 0;
    float eject_position = 0;
};