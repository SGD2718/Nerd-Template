#pragma once

#include "vex.h"

class CubicBezier {
public:
    // Control points
    Vector2 p0, p1, p2, p3;

    CubicBezier() = default;
    CubicBezier(const Vector2 &p0, const Vector2 &p1, const Vector2 &p2, const Vector2 &p3);

    // Evaluate position at parameter u in [0,1]
    [[nodiscard]] Vector2 position(float t) const;
    
    // First derivative P'(u)
    [[nodiscard]] Vector2 first_derivative(float t) const;

    // Second derivative P''(u)
    [[nodiscard]] Vector2 second_derivative(float t) const;

    // Radius of curvature at u
    [[nodiscard]] float radius_of_curvature(float t) const;

    // get an extremely rough approximation for arc length
    [[nodiscard]] float approximate_arc_length() const;
    
    // get an accurate approximation of arc length from times u0 to u1
    [[nodiscard]] float arc_length_from_to(float u0, float u1, float density = 8.0f) const;

    [[nodiscard]] float find_u_for_arc_distance(float u_start, float distance, float distance_to_end, float density = 8.0f) const;
};
