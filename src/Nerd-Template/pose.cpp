#include "vex.h"


Pose::Pose(float x, float y, float theta): Vector2(x, y), theta(theta) {}
Pose::Pose(const Vector2& v, float theta): Vector2(v), theta(theta) {}

Pose Pose::operator+(const Vector2& v) const {
    return {Vector2::operator+(v), this->theta};
}

Pose Pose::operator-(const Vector2& v) const {
    return {Vector2::operator-(v), this->theta};
}

Pose Pose::operator*(float scalar) const {
    return {Vector2::operator*(scalar), this->theta};
}

Pose Pose::operator/(float scalar) const {
    return {Vector2::operator/(scalar), this->theta};
}

Pose Pose::operator-() const {
    return {Vector2::operator-(), -this->theta};
}

Pose Pose::operator+(const Pose& p) const {
    return {Vector2::operator+(p), reduce_negative_180_to_180(this->theta + p.theta)};
}

Pose Pose::operator-(const Pose& p) const {
    return {Vector2::operator-(p), reduce_negative_180_to_180(this->theta - p.theta)};
}

Pose Pose::operator+=(const Vector2& v) {
    Vector2::operator+=(v);
    return *this;
}

Pose Pose::operator-=(const Vector2& v) {
    Vector2::operator-=(v);
    return *this;
}

Pose Pose::operator+=(const Pose& p) {
    Vector2::operator+=(p);
    this->theta = reduce_negative_180_to_180(this->theta + p.theta);
    return *this;
}

Pose Pose::operator-=(const Pose& p) {
    Vector2::operator-=(p);
    this->theta = reduce_negative_180_to_180(this->theta - p.theta);
    return *this;
}

Pose Pose::operator*=(float scalar) {
    Vector2::operator*=(scalar);
    return *this;
}

Pose Pose::operator/=(float scalar) {
    Vector2::operator/=(scalar);
    return *this;
}

Pose Pose::operator=(const Vector2& v) {
    Vector2::operator=(v);
    return *this;
}

Pose Pose::operator=(const Pose& p) {
    Vector2::operator=(p);
    this->theta = p.theta;
    return *this;
}

Vector2 Pose::to_vector2() const {
    return *this;
}

float Pose::heading() const {
    return this->theta;
}