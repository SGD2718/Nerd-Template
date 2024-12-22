#include "vex.h"


SteerCommand::SteerCommand(float v, float omega): v(v), omega(omega) {}

SteerCommand SteerCommand::operator+(const SteerCommand& sc) const {
    return {this->v + sc.v, this->omega + sc.omega};
}

SteerCommand SteerCommand::operator-(const SteerCommand& sc) const {
    return {this->v - sc.v, this->omega - sc.omega};
}

SteerCommand SteerCommand::operator*(float scalar) const {
    return {this->v * scalar, this->omega * scalar};
}

SteerCommand SteerCommand::operator/(float scalar) const {
    return {this->v / scalar, this->omega / scalar};
}

SteerCommand SteerCommand::operator-() const {
    return {-this->v, -this->omega};
}

SteerCommand SteerCommand::operator+=(const SteerCommand& sc) {
    this->v += sc.v;
    this->omega += sc.omega;
    return *this;
}

SteerCommand SteerCommand::operator-=(const SteerCommand& sc) {
    this->v -= sc.v;
    this->omega -= sc.omega;
    return *this;
}

SteerCommand SteerCommand::operator*=(float scalar) {
    this->v *= scalar;
    this->omega *= scalar;
    return *this;
}

SteerCommand SteerCommand::operator/=(float scalar) {
    this->v /= scalar;
    this->omega /= scalar;
    return *this;
}