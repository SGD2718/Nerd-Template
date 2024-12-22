#include "../../include/Nerd-Template/smooth-path.h"
#include <cmath>
#include <stdexcept>
#include <limits>
#include <algorithm>

// Computes the discrete radius of curvature for three points.
float compute_discrete_radius_of_curvature(Vector2 a, Vector2 b, Vector2 c, float value_if_nan) {
    Vector2 ab = b - a;
    Vector2 bc = c - b;
    Vector2 ca = a - c;

    auto cross_prod = std::fabs(ab.cross(ca));

    if (cross_prod != 0)
        return std::sqrt(ab.norm_squared() * bc.norm_squared() * ca.norm_squared()) / (2 * cross_prod);
    
    return value_if_nan;
}


Vector2 line_circle_intersection(Vector2 lineStart, Vector2 lineEnd, Vector2 circleCenter, float radius, bool* flag, float *timeParameter) {
    Vector2 D = lineEnd - lineStart;
    Vector2 F = lineStart - circleCenter;

    float a = D.dot(D);
    float b = 2.0f * F.dot(D);
    float c = F.dot(F) - radius * radius;

    float discriminant = b*b - 4.0f*a*c;

    // If no intersection with the infinite line
    if (discriminant < 0.0f) {
        // Find the closest point on the segment to the circleCenter
        if (flag != nullptr) *flag = false;
        return {nanf_val, nanf_val};
    }

    float sqrtDiscriminant = std::sqrt(discriminant);
    float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
    float t2 = (-b + sqrtDiscriminant) / (2.0f * a);

    // Check if t2 is on the line segment (t2 will always be further along)
    if (t2 >= 0.0f && t2 <= 1.0f) {
        if (flag != nullptr) *flag = true;
        if (timeParameter != nullptr) *timeParameter = t2;
        return lineStart + D * t2;
    }

    // Check if t2 is on the line segment (t2 will always be further along)
    if (t1 >= 0.0f && t1 <= 1.0f) {
        if (flag != nullptr) *flag = true;
        if (timeParameter != nullptr)
            *timeParameter = t1;
        return lineStart + D * t1;
    }

    if (flag != nullptr) *flag = false;

    // If the intersection does not lie on the line segment, return {nan, nan}
    return {nanf_val, nanf_val};
}


// Compute centripetal parameterization for given waypoints.
// t[0] = 0, t[i] = t[i-1] + sqrt(|P_i - P_{i-1}|).
static std::vector<float> compute_centripetal_parameter(const std::vector<Vector2>& points) {
    std::vector<float> t;
    t.resize(points.size());
    t[0] = 0.0f;
    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        float dist = std::sqrt(dx * dx + dy * dy);
        t[i] = t[i - 1] + std::sqrt(dist);
    }
    return t;
}

// Compute natural cubic spline coefficients for 1D data y(t)
static void compute_natural_cubic_spline(
    const std::vector<float>& t,
    const std::vector<float>& y,
    std::vector<float>& a,
    std::vector<float>& b,
    std::vector<float>& c,
    std::vector<float>& d
) {
    size_t n = t.size();
    if (n < 2) {
        std::cerr << "Not enough points for spline." << std::endl;
    }

    a = y;
    b.assign(n, 0.0f);
    c.assign(n, 0.0f);
    d.assign(n, 0.0f);

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

// Evaluate spline at t_val
static float eval_cubic_spline(
    const std::vector<float>& t,
    const std::vector<float>& a,
    const std::vector<float>& b,
    const std::vector<float>& c,
    const std::vector<float>& d,
    float t_val
) {
    size_t n = t.size();
    if (t_val <= t.front()) return a.front();
    if (t_val >= t.back()) return a.back();

    // Binary search for interval
    size_t iLow = 0;
    size_t iHigh = n - 1;
    while (iHigh - iLow > 1) {
        size_t iMid = (iLow + iHigh) / 2;
        if (t_val < t[iMid]) {
            iHigh = iMid;
        }
        else {
            iLow = iMid;
        }
    }

    float dt = t_val - t[iLow];
    return a[iLow] + b[iLow] * dt + c[iLow] * dt * dt + d[iLow] * dt * dt * dt;
}

// Evaluate 2D spline (given separate x and y splines)
static Vector2 eval_cubic_spline_2d(
    const std::vector<float>& T,
    const std::vector<float>& ax, const std::vector<float>& bx, const std::vector<float>& cx, const std::vector<float>& dx,
    const std::vector<float>& ay, const std::vector<float>& by, const std::vector<float>& cy, const std::vector<float>& dy,
    float t_val
) {
    float X = eval_cubic_spline(T, ax, bx, cx, dx, t_val);
    float Y = eval_cubic_spline(T, ay, by, cy, dy, t_val);
    return Vector2{ X, Y };
}

// Approximate arc length by sampling and storing cumulative length
// We will sample the spline at a high resolution, compute cumulative arc length and then do binary searches.
static void build_arc_length_table(
    const std::vector<float>& T,
    const std::vector<float>& ax, const std::vector<float>& bx, const std::vector<float>& cx, const std::vector<float>& dx,
    const std::vector<float>& ay, const std::vector<float>& by, const std::vector<float>& cy, const std::vector<float>& dy,
    std::vector<float>& arcT, std::vector<float>& arcLen
) {
    int samplesPerSegment = 50; // more samples for smoother arc length approximation
    size_t n = T.size();
    arcT.clear();
    arcLen.clear();

    arcT.push_back(T.front());
    arcLen.push_back(0.0f);

    for (size_t i = 0; i < n - 1; ++i) {
        float startT = T[i];
        float endT = T[i + 1];
        float dt = (endT - startT) / samplesPerSegment;
        Vector2 prev = eval_cubic_spline_2d(T, ax, bx, cx, dx, ay, by, cy, dy, startT);
        for (int k = 1; k <= samplesPerSegment; ++k) {
            float tVal = startT + k * dt;
            if (tVal > endT) tVal = endT;
            Vector2 curr = eval_cubic_spline_2d(T, ax, bx, cx, dx, ay, by, cy, dy, tVal);
            float dist = std::sqrt((curr.x - prev.x) * (curr.x - prev.x) + (curr.y - prev.y) * (curr.y - prev.y));
            arcLen.push_back(arcLen.back() + dist);
            arcT.push_back(tVal);
            prev = curr;
        }
    }
}

// Given a desired arc length s, find parameter t that corresponds to it using binary search
static float find_t_for_arc_length(const std::vector<float>& arcT, const std::vector<float>& arcLen, float s) {
    if (s <= 0.0f) return arcT.front();
    if (s >= arcLen.back()) return arcT.back();
    size_t low = 0;
    size_t high = arcLen.size() - 1;
    while (high - low > 1) {
        size_t mid = (low + high) / 2;
        if (arcLen[mid] < s) {
            low = mid;
        }
        else {
            high = mid;
        }
    }
    // Linear interpolation
    float L0 = arcLen[low];
    float L1 = arcLen[high];
    float t0 = arcT[low];
    float t1 = arcT[high];
    if (L1 == L0) return t0;
    float ratio = (s - L0) / (L1 - L0);
    return t0 + (t1 - t0) * ratio;
}

// Generates a discrete G2 path given waypoint controls and desired spacing.
std::vector<Vector2> generate_discrete_g2_path(const std::vector<Vector2>& waypointControls, float spacing) {
    if (waypointControls.size() < 2) {
        return waypointControls;
    }

    // 1. Compute centripetal parameterization
    std::vector<float> T = compute_centripetal_parameter(waypointControls);

    // Extract x, y
    std::vector<float> xArr, yArr;
    xArr.reserve(waypointControls.size());
    yArr.reserve(waypointControls.size());
    for (const auto& p : waypointControls) {
        xArr.push_back(p.x);
        yArr.push_back(p.y);
    }

    // 2. Compute natural cubic spline for x(t) and y(t)
    std::vector<float> ax, bx, cx, dx;
    std::vector<float> ay, by, cy, dy;
    compute_natural_cubic_spline(T, xArr, ax, bx, cx, dx);
    compute_natural_cubic_spline(T, yArr, ay, by, cy, dy);

    // 3. If spacing <= 0 or too small, choose a default spacing
    if (spacing <= 0.0f) {
        // default spacing based on total length
        // estimate length by sampling endpoints
        Vector2 startP = { xArr.front(), yArr.front() };
        Vector2 endP = { xArr.back(), yArr.back() };
        float deltaX = endP.x - startP.x;
        float deltaY = endP.y - startP.y;
        float totalDist = std::sqrt(deltaX * deltaX + deltaY * deltaY);
        spacing = totalDist / static_cast<float>(waypointControls.size() * 10);
    }

    // 4. Build arc length table
    std::vector<float> arcT, arcLen;
    build_arc_length_table(T, ax, bx, cx, dx, ay, by, cy, dy, arcT, arcLen);
    float totalLength = arcLen.back();

    // 5. Resample at uniform spacing along arc length
    int numSamples = static_cast<int>(std::ceil(totalLength / spacing));
    std::vector<Vector2> result;
    result.reserve(numSamples + 1);

    for (int i = 0; i <= numSamples; ++i) {
        float s = i * spacing;
        if (s > totalLength) s = totalLength;
        float tVal = find_t_for_arc_length(arcT, arcLen, s);
        Vector2 P = eval_cubic_spline_2d(T, ax, bx, cx, dx, ay, by, cy, dy, tVal);
        result.push_back(P);
    }

    return result;
}


