#include "vex.h"
    
PID::PID(float error, float kp, float ki, float kd, float integral_max_rate) :
  previous_error(error),
  kp(kp),
  ki(ki * 0.01),
  kd(kd * 100),
  integral_range(integral_max_rate)
{};

PID::PID(float error, float kp, float ki, float kd, float integral_max_rate, float settle_error, float settle_time, float timeout, float dt) :
  previous_error(error),
  kp(kp),
  ki(ki * dt * 0.001),
  kd(kd / dt * 1000),
  integral_range(integral_max_rate),
  settle({settle_error, settle_time, timeout, 2.5}, 10)
{};

PID::PID(float error, const PIDConfig& config, float dt): 
  previous_error(error),
  kp(config.kp),
  ki(config.ki * dt * 0.001),
  kd(config.kd / dt * 1000),
  integral_range(config.integral_range),
  settle({}, dt)
{}

PID::PID(float error, const PIDConfig& config, const SettleConfig& settle, float dt): 
  previous_error(error),
  kp(config.kp),
  ki(config.ki * dt * 0.001),
  kd(config.kd / dt * 1000),
  integral_range(config.integral_range),
  settle(settle, dt)
{}

float PID::compute(float error) {
  if ((error > 0) != (this->previous_error > 0) || error == 0)
    this->accumulated_error = 0;
  if (fabsf(error) <= this->integral_range)
    this->accumulated_error += error;

  this->output = this->kp * error + this->ki * this->accumulated_error + this->kd * (error - this->previous_error);

  this->previous_error = error;

  this->settle.update(error);

  return this->output;
}

bool PID::is_settled(float current){
  return this->settle.is_exit(current);
}

PIDConfig PID::to_PIDConfig() const {
  return {this->kp, this->ki, this->kd, this->integral_range};
}