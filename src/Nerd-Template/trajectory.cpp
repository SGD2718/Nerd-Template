#include "vex.h"

Trajectory::Trajectory(const std::vector<Waypoint>& waypoints, const TrajectoryConfig& config):
        dt(config.dt), duration(0.0f)
{
    UnprocessedTrajectory raw;
    raw.config = config;

    raw.velocity_ratio = (config.max_speed_in_per_sec * this->dt) * (1e-3f / config.max_target_velocity);
    raw.kv = raw.config.max_target_velocity / sqrtf(config.max_full_speed_turn_radius);

    std::vector<Vector2> points;
    points.reserve(waypoints.size());
    raw.velocity_keyframes.reserve(waypoints.size());

    for (auto& wp: waypoints) {
        points.emplace_back(wp.to_vector2());
        raw.velocity_keyframes.emplace_back(wp.v);
    }
    raw.path = compute_cubic_beziers_from_waypoints(points);
    this->generate_from_raw(raw);
}

State Trajectory::sample(float t) const {
    t /= this->dt;
    auto i = (int)t;
    t -= (float)i;
    return lerp<State>(this->trajectory[i], this->trajectory[i+1], t);
}

float Trajectory::get_duration() const {
    return this->duration;
}

void Trajectory::generate_from_raw(UnprocessedTrajectory& raw) {
    if (raw.path.empty())
        return;

    // Start at param = 0
    const int max_steps = 60000 / (int)this->dt; // 60 seconds max
    int step;
    float u = 0.0f;
    float s = 0;
    
    // get total arc length
    float total_length = 0.0f;
    
    std::vector<float> segment_lengths;
    segment_lengths.reserve(raw.path.size());
    
    for (auto& curve: raw.path) {
        segment_lengths.emplace_back(curve.arc_length_from_to(0.f, 1.f));
        total_length += segment_lengths.back();
    }
    
    float current_segment_length = segment_lengths.front();
    float current_length = 0.0f;

    for (step = 0; step < max_steps; ++step) {
        float v = compute_velocity(u, s, current_segment_length, (current_length + s),
                                   total_length - (current_length + s), raw);
        this->trajectory.emplace_back(compute_state(u, v, raw));

        // Check if we've basically finished
        if (u >= (float) raw.path.size())
            break;

        // update
        float ds = v * raw.velocity_ratio;
        s += ds;

        float u_next = advance_param_by_distance(u, ds, raw);
        if ((int) u_next > (int) u) {
            s -= current_segment_length;
            current_length += current_segment_length;
            current_segment_length = segment_lengths[(int) u_next];
        }

        u = u_next;
    }

    this->duration = (float)step * this->dt;
}

State Trajectory::compute_state(float u, float velocity, UnprocessedTrajectory& raw) {
    int i;

    if (u < (float)raw.path.size()) {
        i = (int)u;
        u -= (float)i;
    } else {
        i = (int)raw.path.size() - 1;
        u = 1.0f;
    }

    auto radius_of_curvature = raw.path[i].radius_of_curvature(u);
    auto pos = raw.path[i].position(u);
    auto theta = raw.path[i].first_derivative(u).heading();
    auto omega = (raw.velocity_ratio / raw.config.dt * 1000.0f) * (velocity / radius_of_curvature);
    auto v = (velocity / raw.config.max_target_velocity) * raw.config.max_speed_in_per_sec;

    return {{pos, theta}, {v, omega}};
}

float Trajectory::compute_velocity(float u, float s, float segment_length, float distance_traveled, float remaining_distance, UnprocessedTrajectory& raw) {
    if (u >= (float) raw.path.size())
        return 0;

    auto i = (int) u;
    u -= (float) i;

    // v_max = kv * sqrt(curvature).
    float v = fmaxf(raw.config.min_target_velocity,
                    raw.kv * sqrtf(fabsf(raw.path[i].radius_of_curvature(u))));

    // v_f^2 = v_i^2 + 2ad => a = (v_f^2 - v_i^2) / 2d
    // v^2 = v_i^2 + 2as = v_i^2 + 2s(v_f^2 - v_i^2) / 2d = v_i^2 + (s/d)(v_f^2 - v_i^2) => v = sqrt[v_i^2 + (s/d)(v_f^2 - v_i^2)]
    float v_i2 = raw.velocity_keyframes[i] * raw.velocity_keyframes[i];
    float v_f2 = raw.velocity_keyframes[i + 1] * raw.velocity_keyframes[i + 1];
    v = fminf(v, fmaxf(raw.config.min_target_velocity, sqrtf(v_i2 + (s / segment_length) * (v_f2 - v_i2))));
    
    // end deceleration
    if (remaining_distance < raw.config.acceleration_distance)
        // v = sqrt[v_max^2 + (1 - s / d)(0 - v_max^2)] = v_max * sqrt(1 - (1 - s / d)) = v_max * sqrt(s / d)
        v = fminf(v, raw.config.max_target_velocity *
                     sqrtf(remaining_distance / raw.config.acceleration_distance));
    
    // start acceleration
    if (distance_traveled < raw.config.acceleration_distance && v > raw.config.min_target_velocity) {
        // v = sqrt[v_min^2 + (s/d)(v_max^2 - v_min^2)] = v_max * sqrt(s/d)
        auto v_min2 = raw.config.min_target_velocity * raw.config.min_target_velocity;
        auto v_max2 = raw.config.max_target_velocity * raw.config.max_target_velocity;
        v = fminf(v, sqrtf(v_min2 + (distance_traveled / raw.config.acceleration_distance) * (v_max2 - v_min2)));
    }

    return v;
}

// The key function to move along the spline by ds
float Trajectory::advance_param_by_distance(float u_start, float ds, const UnprocessedTrajectory& raw) {
    float u = u_start;
    float remaining = ds;
    while(fabsf(remaining) > 1e-6) {
        auto seg = (int)u;
        if (seg >= (int)raw.path.size()) {
            return (float)raw.path.size();
        }

        float local_u = u - (float)seg;
        float dist_to_segment_end = raw.path[seg].arc_length_from_to(local_u, 1.0f);

        if(remaining < dist_to_segment_end) {
            float new_local = raw.path[seg].find_u_for_arc_distance(local_u, remaining, dist_to_segment_end);
            return (float)seg + new_local;
        } else {
            // go to next segment
            u = (float)(seg + 1);
            remaining -= dist_to_segment_end;
            if(u >= (float)raw.path.size()) {
                return (float)raw.path.size();
            }
        }
    }
    return u;
}

// Compute centripetal parameterization
std::vector<float> Trajectory::compute_centripetal_parameter(const std::vector<Vector2> &points) {
    std::vector<float> t(points.size());
    t[0] = 0.0f;
    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        float dist = std::sqrt(dx * dx + dy * dy);
        t[i] = t[i - 1] + std::sqrt(dist);
    }
    return t;
}

// Compute natural cubic spline coefficients (for x or y separately)
void Trajectory::compute_natural_cubic_spline(const std::vector<float> &t, const std::vector<float> &y, std::vector<float> &a, std::vector<float> &b, std::vector<float> &c, std::vector<float> &d) {
    size_t n = t.size();
    a = y;
    b.assign(n, 0.0f);
    c.assign(n, 0.0f);
    d.assign(n, 0.0f);

    if (n < 2)
        return;

    std::vector<float> h(n - 1), alpha(n, 0.0f), l(n, 1.0f), mu(n, 0.0f), z(n, 0.0f);
    for (size_t i = 0; i < n - 1; ++i) {
        h[i] = t[i + 1] - t[i];
        if (h[i] <= 0) {
            std::cerr << "t must be strictly increasing." << std::endl;
        }
    }

    for (size_t i = 1; i < n - 1; ++i) {
        alpha[i] = (3.0f / h[i]) * (a[i + 1] - a[i]) - (3.0f / h[i - 1]) * (a[i] - a[i - 1]);
    }

    for (size_t i = 1; i < n - 1; ++i) {
        l[i] = 2.0f * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0f;
    z[n - 1] = 0.0f;
    c[n - 1] = 0.0f;

    for (int j = static_cast<int>(n) - 2; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0f * c[j]) / 3.0f;
        d[j] = (c[j + 1] - c[j]) / (3.0f * h[j]);
    }
}

// Given polynomial coefficients (a, b, c, d) for a segment and knot interval [t_i, t_{i+1}],
// convert to a Bezier segment.
CubicBezier Trajectory::polynomial_segment_to_bezier(float ax, float bx, float cx, float dx, float ay, float by, float cy, float dy, float t0, float t1) {
    float delta = t1 - t0;
    float delta2 = delta * delta;
    float delta3 = delta2 * delta;

    float a_x = ax;
    float b_x = bx * delta;
    float c_x = cx * delta2;
    float d_x = dx * delta3;

    float a_y = ay;
    float b_y = by * delta;
    float c_y = cy * delta2;
    float d_y = dy * delta3;

    Vector2 p0(a_x, a_y);
    Vector2 p1(a_x + b_x / 3.0f, a_y + b_y / 3.0f);
    Vector2 p2(a_x + (2.0f * b_x + c_x) / 3.0f, a_y + (2.0f * b_y + c_y) / 3.0f);
    Vector2 p3(a_x + b_x + c_x + d_x, a_y + b_y + c_y + d_y);

    return {p0, p1, p2, p3};
}

// Main function to compute CubicBeziers from waypoints
std::vector<CubicBezier> Trajectory::compute_cubic_beziers_from_waypoints(const std::vector<Vector2> &waypoints) {
    if (waypoints.size() < 2) {
        return {};
    }

    std::vector<float> t = compute_centripetal_parameter(waypoints);

    std::vector<float> x_arr, y_arr;
    x_arr.reserve(waypoints.size());
    y_arr.reserve(waypoints.size());
    for (auto &p : waypoints) {
        x_arr.push_back(p.x);
        y_arr.push_back(p.y);
    }

    std::vector<float> ax, bx, cx, dx;
    std::vector<float> ay, by, cy, dy;
    compute_natural_cubic_spline(t, x_arr, ax, bx, cx, dx);
    compute_natural_cubic_spline(t, y_arr, ay, by, cy, dy);

    std::vector<CubicBezier> segments;
    segments.reserve(t.size() - 1);

    for (size_t i = 0; i < t.size() - 1; ++i) {
        segments.emplace_back(polynomial_segment_to_bezier(
                ax[i], bx[i], cx[i], dx[i],
                ay[i], by[i], cy[i], dy[i],
                t[i], t[i + 1]
        ));
    }

    return segments;
}