#include "vex.h"

Vector2 Vector2::operator +(const Vector2 &p) const {
    return {this->x + p.x, this->y + p.y};
}

Vector2 Vector2::operator -(const Vector2 &p) const {
    return {this->x - p.x, this->y - p.y};
}

Vector2 Vector2::operator *(float scalar) const {
    return {this->x * scalar, this->y * scalar};
}

Vector2 Vector2::operator /(float scalar) const {
    return {this->x / scalar, this->y / scalar};
}

Vector2 Vector2::operator -() const {
    return {-this->x, -this->y};
}

Vector2 operator*(float scalar, const Vector2& v) {
    return v * scalar;
}

bool Vector2::operator == (const Vector2 &p) const {
    return this->x == p.x && this->y == p.y;
}

bool Vector2::operator != (const Vector2 &p) const {
    return this->x != p.x || this->y != p.y;
}

Vector2 Vector2::operator +=(const Vector2 &p) {
    this->x += p.x;
    this->y += p.y;
    return *this;
}

Vector2 Vector2::operator -=(const Vector2 &p) {
    this->x -= p.x;
    this->y -= p.y;
    return *this;
}

Vector2 Vector2::operator *=(float scalar) {
    this->x *= scalar;
    this->y *= scalar;
    return *this;
}

Vector2 Vector2::operator /=(float scalar) {
    this->x /= scalar;
    this->y /= scalar;
    return *this;
}

Vector2 Vector2::normal() const {
    return {-this->y, this->x};
}

Vector2 Vector2::unit_normal() const {
    return Vector2(-this->y, this->x) / this->norm();
}

float Vector2::norm() const {
    return sqrtf(this->x * this->x + this->y * this->y);
}

float Vector2::norm_squared() const {
    return this->x * this->x + this->y * this->y;
}

float Vector2::dot(const Vector2 &v) const {
    return this->x * v.x + this->y * v.y;
}

float Vector2::cross(const Vector2 &v) const {
    return this->x * v.y - this->y * v.x;
}

Vector2 Vector2::hadamard(const Vector2& v) const {
  return Vector2(this->x * v.x, this->y * v.y);
}

float Vector2::angle() const {
    return atan2f(this->y, this->x);
}

float Vector2::angle_between(const Vector2 &v) const {
    return atan2f(this->cross(v), this->dot(v));
}

bool Vector2::is_parallel(const Vector2 &v, float tolerance) const {
    return fabsf(this->cross(v)) <= tolerance;
}

Vector2 Vector2::unit_vector() const {
  return *this / this->norm();
}

bool Vector2::is_undef() const{
    return this->x == INFINITY || this->y == INFINITY;
}

Vector2 Vector2::rescale(float length) const {
    return *this * (length / this->norm());
}

float Vector2::comp(Vector2 direction) const {
    return this->dot(direction) / direction.norm();
}

Vector2 Vector2::proj(Vector2 direction) const {
    return direction * (this->dot(direction) / direction.norm_squared());
}

float Vector2::orthogonal_comp(Vector2 direction) const {
    return std::fabs(this->cross(direction) / direction.norm());
}

Vector2 Vector2::rotate(float theta) const {
    auto c = cos(theta);
    auto s = sin(theta);
    return {(float)(c * this->x - s * this->y), (float)(s * this->x + c * this->y)};
}

float Vector2::heading() const {
    return atan2(this->y, this->x);
}

/**
 * @brief reflect this vector about an axis
 * @param axis axis of reflection
 * @return reflected vector
 */
Vector2 Vector2::reflect_about(Vector2 axis) const {
    return *this - axis * ( this->dot(axis) * 2 );
}

