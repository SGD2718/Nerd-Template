#pragma once

#include "vex.h"

enum RingColor {
    ALL_RINGS,
    RED_RINGS,
    BLUE_RINGS,
    NO_RINGS,
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

    void set_filter_mode(RingColor ring_color);

    void update();
    void set_velocity(float velocity);
    void raise();
    void lower();
    void set_raised(bool is_raised);

    void clear_ring();

    void on_jam(void (*callback)());

    void set_filter_mode_current();

    bool has_ring_at_top() const;
    RingColor get_current_ring_color() const;
    bool has_ring_going_up() const;

    bool is_spinning() const;

    void pickup();
    void intake();
    void outtake();
    void off();

    RingColor *filter_mode;
    motor* intake_motor;
    optical* color_sensor;
    digital_out* intake_piston;

private:
    // reading colors
    void update_color_detections();
    RingColor detected_color() const;
    bool sees_red() const;
    bool sees_blue() const;

    void (*on_jam_callback)();

    int direction = 0;
    unsigned int time_to_stop_reversing = 0;
    float rotation_when_ring_lost = 0;
    float rotation_to_eject = 0;
    float *target_velocity = 0;
    bool detected_ring = false;
    bool ring_going_up = false;
    bool ejection_pending = false;
    RingColor current_ring_color = NO_RINGS;

    // used to ensure confidence in detection with a moving average
    float detection_confidence = 0;
    float red_confidence = 0;
    float blue_confidence = 0;

    // constants
    // TODO: tune these
    HueWindow red_hue{5.f, 30.f};
    HueWindow blue_hue{215.f, 230.f};
    float ejection_revs = 1; 
    float new_reading_weight = 1;
    float confidence_threshold = 0.7;
    float avg_current = 0;
};