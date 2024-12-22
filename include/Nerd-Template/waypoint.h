#pragma once
#include "vex.h"

class Waypoint: public Vector2 {
public:
    Waypoint(const Vector2& v, float velocity);
    Waypoint(float x, float y, float velocity);

    Waypoint operator +(const Vector2 &p) const;
    Waypoint operator -(const Vector2 &p) const;
    Waypoint operator *(float scalar) const;
    Waypoint operator /(float scalar) const;
    Waypoint operator -() const;

    Waypoint operator +=(const Vector2 &p);
    Waypoint operator -=(const Vector2 &p);
    Waypoint operator *=(float scalar);
    Waypoint operator /=(float scalar);

    Vector2 to_vector2() const;

    float v;
};

Waypoint operator*(float scalar, const Waypoint& v);