//
// Created by Benjamin Lee on 11/9/23.
//

#include "vex.h"
#include <iostream>

float reduce_0_to_360(float angle) {
    while(!(angle >= 0 && angle < 360)) {
        if( angle < 0 ) { angle += 360; }
        if(angle >= 360) { angle -= 360; }
    }
    return(angle);
}

float reduce_negative_180_to_180(float angle) {
    while(!(angle >= -180 && angle < 180)) {
        if( angle < -180 ) { angle += 360; }
        if(angle >= 180) { angle -= 360; }
    }
    return(angle);
}

float reduce_negative_90_to_90(float angle) {
    while(!(angle >= -90 && angle < 90)) {
        if( angle < -90 ) { angle += 180; }
        if(angle >= 90) { angle -= 180; }
    }
    return(angle);
}

float to_rad(float angle_deg){
    return(angle_deg / (180.0f / static_cast<float>(M_PI)));
}

float to_deg(float angle_rad){
    return(angle_rad * (180.0f / static_cast<float>(M_PI)));
}

float clamp(float input, float min, float max){
    if( input > max ){ return(max); }
    if(input < min){ return(min); }
    return(input);
}

bool is_reversed(double input){
    if(input<0) return(true);
    return(false);
}

float get_rpm(gearSetting gear_cart) {
    switch (gear_cart) {
        case gearSetting::ratio6_1:
            return 600.f;
        case gearSetting::ratio18_1:
            return 200.f;
        case gearSetting::ratio36_1:
            return 100.f;
    }

    return 0;
}

float get_rpm(motor m) {
    return get_rpm(m.getMotorCartridge());
}

float to_volt(float percent){
    return(percent /127 * 12);
}

float to_volt_from_value(float value){
    return(value * 12 / 127);
}

// numerically approximate the integral of the function f(t) from a to b
// with `partitions` partitions using Simpson's Rule
float integrate(float a, float b, std::function<float(float)> &f, int partitions) {
    /*
     * Simpson's Rule:
     * let n be the number of partitions
     * let Δt = 1 / n
     * let x_i = a + iΔt
     * let m_i = (x_i + x_{i+1}) / 2
     * ∫(a, b) f(x) dx ≈ (b - a) / 6n * [f(a) + f(b) + 4f(a + ∆t/2) + ∑( 2f(x_i) + 4f(x_i + ∆t/2) )]
     */

    float dt = (b-a) / static_cast<float>(partitions);
    float halfDt = dt * 0.5f;
    float integral = f(a) + f(b) + 4.0f * f(a + halfDt);

    // we are reusing `a` as `x_i` so that we don't need to make another float
    for (int i = 1; i < partitions; ++i) {
        a += dt;
        integral += 2*f(a) + 4*f(a + halfDt);
    }

    return integral * dt * 0.166667f;
}

// For some reason, std::stof is not implemented, so it is being added to the std namespace here
namespace std {
  float stof(const std::string& str) {
    float result = 0.0f;
    float decimalFactor = 0.1f;
    bool isNegative = false;
    bool decimalFound = false;
    bool exponentFound = false;
    int exponent = 0;
    bool exponentNegative = false;

    // Check for negative sign
    size_t i = 0;
    if (str[i] == '-') {
        isNegative = true;
        ++i;
    } else if (str[i] == '+') {
        ++i; // Skip leading plus sign
    }

    for (; i < str.length(); ++i) {
        char c = str[i];

        if (c >= '0' && c <= '9') {
            if (exponentFound) {
                exponent = exponent * 10 + (c - '0');
            } else if (decimalFound) {
                result = result + static_cast<float>(c - '0') * decimalFactor;
                decimalFactor *= 0.1f;
            } else {
                result = result * 10.0f + static_cast<float>(c - '0');
            }
        } else if (c == '.') {
            decimalFound = true;
        } else if (c == 'e' || c == 'E') {
            exponentFound = true;

            // Check for exponent sign
            if (str[++i] == '-') {
                exponentNegative = true;
            } else if (str[i] == '+') {
                ++i; // Skip leading plus sign
            }

            // Parse exponent value
            for (; i < str.length(); i++) {
                char exponentChar = str[i];
                if (exponentChar >= '0' && exponentChar <= '9') {
                    exponent = exponent * 10 + (exponentChar - '0');
                } else {
                    // Handle invalid characters in the exponent here, if needed
                    std::cerr << "Invalid character in exponent: " << exponentChar << std::endl;
                    return 0.0f; // You can choose an appropriate error value
                }
            }
        } else {
            // Handle invalid characters here, if needed
            std::cerr << "Invalid character encountered: " << c << std::endl;
            return 0.0f; // You can choose an appropriate error value
        }
    }

    if (isNegative) {
        result = -result;
    }

    // Apply exponent if found
    if (exponentFound) {
        float exponentMultiplier = 1.0f;
        for (int e = 0; e < exponent; ++e) {
            exponentMultiplier *= 10.0f;
        }
        if (exponentNegative) {
            result /= exponentMultiplier;
        } else {
            result *= exponentMultiplier;
        }
    }

    return result;
  }
}

// https://en.wikipedia.org/wiki/Newton%27s_method
float newton_raphson_1_var(float x0, std::function<float(float)> &f, std::function<float(float)> &f_prime,
                           int maxIterations, float minX, float maxX, float tolerance) {
    auto x = clamp(x0, minX, maxX);
    float fOfX;
    float fPrimeOfX;

    auto perturbation = 0.01f;

    for (auto i = 0; i < maxIterations; ++i) {
        // clamp the value of t to the boundaries to bring it back in bounds faster
        x = clamp(x, minX, maxX);

        /* NEWTON'S METHOD
         * let xᵢ be the i-th estimate of the value of x at which f(x) = 0
         * xᵢ₊₁ = xᵢ - f(xᵢ) / f'(xᵢ)
         */

        fOfX = f(x);
        fPrimeOfX = f_prime(x);

        /* terminate early when one of the following conditions are met:
         * 1) f(x) is within tolerance
         * 2) the only way to make f(x) closer to zero is to continue going out of bounds
         */
        if (fabsf(fOfX) <= tolerance ||
            (x <= minX && (fPrimeOfX > 0) == (fOfX > 0)) ||
            (x >= maxX && (fPrimeOfX < 0) == (fOfX > 0))) {
            break;
        }

        // if the derivative is 0, then we don't know which way to go,
        // so we add a small perturbation to t and try again
        if (fPrimeOfX == 0) {
            while (x - perturbation == x) perturbation *= 2; // double the perturbation until it's impact is non-zero
            x -= perturbation; // apply a small perturbation to the time to get a non-zero second derivative
            continue;
        } else perturbation = 0.01f; // reset the perturbation

        // update x
        x0 -= fOfX / fPrimeOfX;
    }

    // get the closest point at the time
    if (fabsf(fOfX) > tolerance) return x0;
    return x;
}

float rpm_to_volt(float rotational_velocity, float motor_rpm) {
  return 12.0f / motor_rpm * rotational_velocity;
}

float volt_to_rpm(float rotational_velocity, float motor_rpm) {
  return motor_rpm / 12.0f * rotational_velocity;
}

float three_point_circle_radius(Vector2 P, Vector2 Q, Vector2 R) {
  auto x1 = P.x, y1 = P.y, x2 = Q.x, y2 = Q.y, x3 = R.x, y3 = R.y;

  if (x1 == x2) {
    x1 += 0.001;
  }
  
  auto x12 = x1 - x2;
  auto x13 = x1 - x3;

  auto y12 = y1 - y2;
  auto y13 = y1 - y3;

  auto y31 = y3 - y1;
  auto y21 = y2 - y1;

  auto x31 = x3 - x1;
  auto x21 = x2 - x1;
 
  // x1^2 - x3^2
  auto sx13 = x1 * x1 - x3 * x3;

  // y1^2 - y3^2
  auto sy13 = y1 * y1 - y3 * y3;

  auto sx21 = x2 * x2 - x1 * x1;
  auto sy21 = y2 * y2 - y1 * y1;

  auto f = ((sx13) * (x12)
            + (sy13) * (x12)
            + (sx21) * (x13)
            + (sy21) * (x13))
          / (2 * ((y31) * (x12) - (y21) * (x13)));
  auto g = ((sx13) * (y12)
            + (sy13) * (y12)
            + (sx21) * (y13)
            + (sy21) * (y13))
          / (2 * ((x31) * (y12) - (x21) * (y13)));

  auto c = -x1 * x1 - y1 * y1 - 2 * g * x1 - 2 * f * y1;

  // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
  // where centre is (h = -g, k = -f) and radius r
  // as r^2 = h^2 + k^2 - c
  auto h = -g;
  auto k = -f;
  auto r_squared = h * h + k * k - c;

  // r is the radius
  return sqrt(r_squared);
}

/**
 * Check if the data has plateaued using slope of the least squares regression line predicting the last `sampleSize` data points
 * @param data the data to check
 * @param sampleSize the number of samples to check
 * @param tolerance the tolerance for the slope
 * @return true if the data has plateaued, false otherwise
 */
bool has_plateaued(const std::vector<std::pair<float, float>>& data, int sampleSize, float tolerance) {
    auto end = (int)data.size() - 1;

    if (end + 1 < sampleSize)
        return false;

    // calculate the mean X and Y values
    float meanX = 0;
    float meanY = 0;
    for (int i = 0; i < sampleSize; ++i) {
        meanX += (float)data[end - i].first;
        meanY += data[end - i].second;
    }
    meanX /= (float)sampleSize;
    meanY /= (float)sampleSize;

    // calculate the slope
    float rise = 0;
    float run = 0;

    for (int i = 0; i < sampleSize; ++i) {
        rise += ((float)data[end - i].first - meanY) * (data[end - i].second - meanY);
        run += ((float)data[end - i].first - meanX) * ((float)data[end - i].first - meanX);
    }
    auto slope = rise / run;

    return fabsf(slope) < tolerance;
}

/**
 * Check if the data is roughly constant by making sure that the mean change in value is roughly zero and that the values are close enough to the mean
 * @param data the data to check
 * @param percent_tolerance the percent of mean to use as the maximum allowable deviation. Defaults to 5%
 * @param t_score the t-score to use for the confidence interval to check if the mean change in value is roughly zero
 * @return true if the data has plateaued, false otherwise
 */
bool is_approximately_constant(const std::vector<float>& data, float percent_tolerance = 5, float t_score = 1.96) {
    // check if all values are within tolerance of the mean
    float mean = 0;
    for (auto value: data)
        mean += value;
    mean /= (float)data.size();

    percent_tolerance *= 0.01;
    float lower_bound = mean * (1 - percent_tolerance);
    float upper_bound = mean * (1 + percent_tolerance);

    for (auto value: data)
        if (value < lower_bound || value > upper_bound)
            return false;

    // check that the mean change in amplitude is roughly zero by checking the confidence interval
    std::vector<float> changes;
    changes.reserve(data.size() - 1);
    float mean_change = 0;
    float mean_sqr_change = 0;
    for (int i = 1; i < data.size(); ++i) {
        changes.push_back(data[i] - data[i - 1]);
        mean_change += changes.back();
        mean_sqr_change += powf(changes.back(), 2);
    }
    mean_change /= changes.size();
    mean_sqr_change /= changes.size();

    auto std_dev = sqrtf(mean_sqr_change - powf(mean_change, 2));
    auto std_err = std_dev / sqrtf((float)changes.size());
    auto confidence_interval = t_score * std_err;

    // check if 0 falls outside the confidence interval
    return mean_change - confidence_interval <= 0 && mean_change + confidence_interval >= 0;
}

float fsign(float x) {
    return x > 0 ? 1 : x < 0 ? -1 : 0;
}