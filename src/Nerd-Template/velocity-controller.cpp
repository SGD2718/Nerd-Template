#include "vex.h"

VelocityController::VelocityController(float target, float kv, float ka, float kf, float kp, float ki, float kd, float integral_range, float dt):
  previous_target(target), kv(kv), ka(ka / dt * 1000), kf(kf), pid(target, kp, ki, kd, integral_range, 0, 0, 0, dt) {}

VelocityController::VelocityController(float target, float kv, float ka, float kf, const PIDConfig& pid, float dt):
  previous_target(target), kv(kv), ka(ka / dt * 1000), kf(kf), pid(target, pid, dt) {}

VelocityController::VelocityController(float target, const VelocityControllerConfig& config, float dt):
  previous_target(target), kv(config.kp), ka(config.ka / dt * 1000), kf(config.kf), pid(target, config.kp, config.ki, config.kd, config.integral_range, 0, 0, 0, dt) {}


float VelocityController::compute(float target, float error) {
  this->output = this->kv * target + this->ka * (target - this->previous_target) + this->kf * fsign(target) + this->pid.compute(error);
  this->previous_target = target;
  return this->output;
}