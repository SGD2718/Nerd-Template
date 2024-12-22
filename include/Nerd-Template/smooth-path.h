#pragma once

#include "vex.h"
#include <vector>
#include <cmath>
#include <limits>

static float nanf_val = std::numeric_limits<float>::quiet_NaN();

// Computes the discrete radius of curvature for three points.
float compute_discrete_radius_of_curvature(Vector2 a, Vector2 b, Vector2 c, float value_if_nan = __builtin_huge_valf());

Vector2 line_circle_intersection(Vector2 lineStart, Vector2 lineEnd, Vector2 circleCenter, float radius, bool* flag = nullptr, float *timeParameter = nullptr);

// Generates a discrete G2 path given waypoint controls and desired spacing.
std::vector<Vector2> generate_discrete_g2_path(const std::vector<Vector2>& waypoint_controls, float spacing);