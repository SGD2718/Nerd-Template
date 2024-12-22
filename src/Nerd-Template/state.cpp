#include "vex.h"

State::State(const Pose& p, const SteerCommand& sc): Pose(p), SteerCommand(sc) {}
State::State(float x, float y, float v, float theta, float omega): Pose(x, y, theta), SteerCommand(v, omega) {}

State State::operator+(const Pose& p) const {
    return {Pose::operator+(p), this->to_steer_command()};
}

State State::operator-(const Pose& p) const {
    return {Pose::operator-(p), this->to_steer_command()};
}

State State::operator+(const SteerCommand& sc) const {
    return {this->to_pose(), SteerCommand::operator+(sc)};
}

State State::operator-(const SteerCommand& sc) const {
    return {this->to_pose(), SteerCommand::operator-(sc)};
}

State State::operator+(const State& state) const {
    return {Pose::operator+(state), SteerCommand::operator+(state)};
}

State State::operator-(const State& state) const {
    return {Pose::operator-(state), SteerCommand::operator-(state)};
}

State State::operator*(float scalar) const {
    return {Pose::operator*(scalar), SteerCommand::operator*(scalar)};
}

State State::operator/(float scalar) const {
    return {Pose::operator/(scalar), SteerCommand::operator/(scalar)};
}

State State::operator-() const {
    return {Pose::operator-(), SteerCommand::operator-()};
}

State State::operator+=(const Pose& p) {
    Pose::operator+=(p);
    return *this;
}

State State::operator-=(const Pose& p) {
    Pose::operator-=(p);
    return *this;
}

State State::operator+=(const SteerCommand& sc) {
    SteerCommand::operator+=(sc);
    return *this;
}

State State::operator-=(const SteerCommand& sc) {
    SteerCommand::operator-=(sc);
    return *this;
}

State State::operator+=(const State& state) {
    Pose::operator+=(state);
    SteerCommand::operator+=(state);
    return *this;
}

State State::operator-=(const State& state) {
    Pose::operator-=(state);
    SteerCommand::operator-=(state);
    return *this;
}

State State::operator*=(float scalar) {
    Pose::operator*=(scalar);
    SteerCommand::operator*=(scalar);
    return *this;
}

State State::operator/=(float scalar) {
    Pose::operator/=(scalar);
    SteerCommand::operator/=(scalar);
    return *this;
}

State State::operator=(const Vector2& v) {
    Vector2::operator=(v);
    return *this;
}

State State::operator=(const Pose& p) {
    Pose::operator=(p);
    return *this;
}

State State::operator=(const SteerCommand& sc) {
    SteerCommand::operator=(sc);
    return *this;
}

State State::operator=(const State& state) {
    Pose::operator=(state);
    SteerCommand::operator=(state);
    return *this;
}

Pose State::to_pose() const {
    return *this;
}

SteerCommand State::to_steer_command() const {
    return *this;
}

Waypoint State::to_waypoint() const {
    return {this->to_vector2(), this->v};
}