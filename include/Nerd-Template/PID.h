#pragma once
#include "vex.h"

class PID {
public:
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float integral_range = 0;
  
  float accumulated_error = 0;
  float previous_error = 0;
  float output = 0;

  Settle settle;

  PID(float error, float kp, float ki, float kd, float integral_range, float settle_error, float settle_time, float timeout, float dt = 10.0f);

  PID(float error, float kp, float ki, float kd, float integral_range);

  PID(float error, const PIDConfig& config, float dt = 10.0f);
  PID(float error, const PIDConfig& config, const SettleConfig& settle, float dt = 10.0f);

  float compute(float error);

  bool is_settled(float current = 0.0f);

  PIDConfig to_PIDConfig() const;
};