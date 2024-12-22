#include "vex.h"

Waypoint::Waypoint(const Vector2& v, float velocity): Vector2(v), v(velocity) {}
Waypoint::Waypoint(float x, float y, float velocity): Vector2(x, y), v(velocity) {}

Waypoint Waypoint::operator +(const Vector2 &p) const {
    return {this->x + p.x, this->y + p.y, this->v};
}

Waypoint Waypoint::operator -(const Vector2 &p) const {
    return {this->x - p.x, this->y - p.y, this->v};
}

Waypoint Waypoint::operator *(float scalar) const {
    return {this->x * scalar, this->y * scalar, this->v * scalar};
}

Waypoint Waypoint::operator /(float scalar) const {
    auto tmp = 1.0f / scalar;
    return {this->x * tmp, this->y * tmp, this->v * tmp};
}

Waypoint Waypoint::operator -() const {
    return {-this->x, -this->y, -this->v};
}

Waypoint operator*(float scalar, const Waypoint& p) {
    return p * scalar;
}

Waypoint Waypoint::operator +=(const Vector2 &p) {
    this->x += p.x;
    this->y += p.y;
    return *this;
}

Waypoint Waypoint::operator -=(const Vector2 &p) {
    this->x -= p.x;
    this->y -= p.y;
    return *this;
}

Waypoint Waypoint::operator *=(float scalar) {
    this->x *= scalar;
    this->y *= scalar;
    this->v *= scalar;
    return *this;
}

Waypoint Waypoint::operator /=(float scalar) {
    auto tmp = 1.0f / scalar;
    this->x *= tmp;
    this->y *= tmp;
    this->v *= tmp;
    return *this;
}

Vector2 Waypoint::to_vector2() const {
    return {this->x, this->y};
}