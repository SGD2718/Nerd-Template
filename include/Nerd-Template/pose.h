#include "vex.h"

class Pose: public Vector2 {
public:
    Pose() = default;
    Pose(float x, float y, float theta);
    Pose(const Vector2& v, float theta);

    Pose operator+(const Vector2& v) const;
    Pose operator-(const Vector2& v) const;
    Pose operator+(const Pose& p) const;
    Pose operator-(const Pose& p) const;

    Pose operator*(float scalar) const;
    Pose operator/(float scalar) const;

    Pose operator-() const;

    Pose operator+=(const Vector2& v);
    Pose operator-=(const Vector2& v);
    Pose operator+=(const Pose& p);
    Pose operator-=(const Pose& p);

    Pose operator*=(float scalar);
    Pose operator/=(float scalar);

    Pose operator=(const Vector2& v);
    Pose operator=(const Pose& p);

    Vector2 to_vector2() const;

    float heading() const;

    float theta;
};