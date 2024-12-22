#include "vex.h"

CubicBezier::CubicBezier(const Vector2 &p0, const Vector2 &p1, const Vector2 &p2, const Vector2 &p3)
        : p0(p0), p1(p1), p2(p2), p3(p3) {}

Vector2 CubicBezier::position(float t) const {
    float t2 = t * t;
    float t3 = t2 * t;

    float one_minus_t = 1.0f - t;
    float one_minus_t2 = one_minus_t * one_minus_t;
    float one_minus_t3 = one_minus_t2 * one_minus_t;

    float c0 = one_minus_t3;
    float c1 = 3.0f * one_minus_t2 * t;
    float c2 = 3.0f * one_minus_t * t2;
    float c3 = t3;

    return p0 * c0 + p1 * c1 + p2 * c2 + p3 * c3;
}

Vector2 CubicBezier::first_derivative(float t) const {
    float t2 = t * t;
    float one_minus_t = 1.0f - t;
    float one_minus_t2 = one_minus_t * one_minus_t;

    float c0 = 3.0f * one_minus_t2;
    float c1 = 6.0f * one_minus_t * t;
    float c2 = 3.0f * t2;

    return c0 * (p1 - p0) + c1 * (p2 - p1) + c2 * (p3 - p2);
}

Vector2 CubicBezier::second_derivative(float t) const {
    float one_minus_t = 1.0f - t;

    float c0 = 6.0f * one_minus_t;
    float c1 = 6.0f * t;

    Vector2 v1 = (p2 - p1 * 2.0f + p0);
    Vector2 v2 = (p3 - p2 * 2.0f + p1);

    return c0 * v1 + c1 * v2;
}

float CubicBezier::radius_of_curvature(float t) const {
    Vector2 d1 = first_derivative(t);
    Vector2 d2 = second_derivative(t);

    float numerator = std::pow(d1.norm_squared(), 1.5f);

    float denom = d1.cross(d2);
    if (fabsf(denom) < 1e-12f) {
        return std::numeric_limits<float>::infinity();
    }
    return numerator / denom;
}

// Rough approximation of the arc length using both the control polygon and endpoint distance
float CubicBezier::approximate_arc_length() const {
    auto a2 = (p1 - p0).norm_squared();
    auto b2 = (p2 - p1).norm_squared();
    auto c2 = (p3 - p2).norm_squared();
    auto d2 = (p3 - p0).norm_squared();

    auto a = sqrtf(a2);
    auto b = sqrtf(b2);
    auto c = sqrtf(c2);
    auto d = sqrtf(d2);

    // most of the time the QM of all the lengths is a superior approximation
    if (a + b + c > 2 * d) {
        a2 = fmaxf(a2, (p1 - p3).norm_squared());
        c2 = fmaxf(c2, (p2 - p3).norm_squared());
        return sqrtf((a2 + b2 + c2 + d2) * 0.5f);
    }

    // just the AM of the control net length and the distance between endpoints
    return (a + b + c + d) * 0.5f;
}

float CubicBezier::arc_length_from_to(float u0, float u1, float density) const {
    // Ensure u0 <= u1
    if(u1 < u0) std::swap(u0, u1);
    
    // Edge case: if they are nearly equal, arc length is ~0
    if(std::fabs(u1 - u0) < 1e-9f) {
        return 0.0f;
    }

    auto steps = std::roundf(this->approximate_arc_length() * (u1 - u0) * density);
    
    // Sample the curve in 'steps' intervals from u0 to u1
    float total_length = 0.0f;
    Vector2 prev = this->position(u0);

    for(int i = 1; i <= steps; ++i) {
        float t = u0 + (u1 - u0)* (static_cast<float>(i)/steps);
        Vector2 cur = this->position(t);
        total_length += (cur - prev).norm();
        prev = cur;
    }
    return total_length;
}

float CubicBezier::find_u_for_arc_distance(float u_start, float distance, float distance_to_end, float density) const {
    if (distance >= distance_to_end)
        return 1.0f;

    // use newton's method to find the u value that corresponds to the distance

    float u = std::min(u_start + (1.0f-u_start) * (distance / distance_to_end), 1.0f);

    const int max_iters = 10;
    const float tol = 1e-6f;

    for (int i = 0; i < max_iters; ++i) {
        float f = this->arc_length_from_to(u_start, u, density) - distance;

        if (fabsf(f) < tol) // close enough to finish
            break;

        auto df = this->first_derivative(u).norm();
        if (df < 1e-9f) // this should never happen
            break;

        // Newton step: u_{n+1} = u_n - f(u_n)/f'(u_n)
        float step = f / df;
        u = clamp(u - step, u_start, 1.0f);
    }

    return u;
}