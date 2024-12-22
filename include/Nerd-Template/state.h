#pragma once

#include "vex.h"

class State: public Pose, public SteerCommand {
public:
    State() = default;
    State(const Pose& p, const SteerCommand& sc);
    State(float x, float y, float theta, float velocity, float omega);

    State operator+(const Pose& p) const;
    State operator-(const Pose& p) const;
    State operator+(const SteerCommand& sc) const;
    State operator-(const SteerCommand& sc) const;
    State operator+(const State& state) const;
    State operator-(const State& state) const;
    State operator*(float scalar) const;
    State operator/(float scalar) const;
    State operator-() const;

    State operator+=(const Pose& p);
    State operator-=(const Pose& p);
    State operator+=(const SteerCommand& sc);
    State operator-=(const SteerCommand& sc);
    State operator+=(const State& state);
    State operator-=(const State& state);
    State operator*=(float scalar);
    State operator/=(float scalar);

    State operator=(const Vector2& v);
    State operator=(const Pose& p);
    State operator=(const SteerCommand& sc);
    State operator=(const State& state);

    [[nodiscard]] Waypoint to_waypoint() const;
    [[nodiscard]] Pose to_pose() const;
    [[nodiscard]] SteerCommand to_steer_command() const;

    float heading() const;

    void set_position(const Vector2& v);

    float theta;
    float omega;
};

State operator*(float scalar, const State& s);