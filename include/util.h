#pragma once

#include "vex.h"
#include <functional>
#include <vector>
#include <string>

//#define INFINITY __builtin_huge_valf()

enum AngleUnits: std::uint_fast8_t {
    DEGREES,
    RADIANS
};

float reduce_0_to_360(float angle);

float reduce_negative_180_to_180(float angle);

float reduce_negative_90_to_90(float angle);

float to_rad(float angle_deg);

float to_deg(float angle_rad);

float get_rpm(gearSetting gear_cart);

float get_rpm(motor m);

float get_rpm(motor_group m);

float rpm_to_volt(float rotational_velocity, float motor_rpm=200);

float volt_to_rpm(float rotational_velocity, float motor_rpm=200);

float clamp(float input, float min, float max);

bool is_reversed(double input);

float to_volt(float percent);

float to_volt_from_value(float value);

float integrate(float a, float b, std::function<float(float)> &f, int partitions);

float newton_raphson_1_var(float x0, std::function<float(float)> &f, std::function<float(float)> &f_prime,
                           int maxIterations, float minX = -INFINITY, float maxX = INFINITY, float tolerance = 0.001f);

bool has_plateaued(const std::vector<std::pair<float, float>>& data, int sampleSize, float threshold);

float fsign(float x);

template<typename T>
T lerp(const T &a, const T &b, float t) {
    return a + (b - a) * t;
}

template<typename T>
int binary_search(const std::vector<T> &container, const T &value) {
    int lower = 0;
    int upper = container.size();
    int midpoint = 0;

    while (upper > lower) {
        // midpoint of upper and lower bounds
        midpoint = (lower + upper) >> 1;

        // divide range in half
        if (container[midpoint] > value) {
            upper = midpoint - 1;
        } else if (container[midpoint] < value) {
            lower = midpoint + 1;
        } else {
            return midpoint;
        }
    }

    // return the midpoint between the upper and lower bounds
    // this will be the lower bound if the value is not found
    return midpoint;
}

float three_point_circle_radius(Vector2 a, Vector2 b, Vector2 c);

namespace std {
  float stof(const std::string &str);
}
