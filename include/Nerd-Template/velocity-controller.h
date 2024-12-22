#pragma once
#include "vex.h"

#define LOG_VELOCITY_CONTROLLER true

// proportion-velocity-acceleration velocity controller
class VelocityController
{
public:
  float kv;               // velocity gain
  float ka;               // acceleration gain
  float kf;               // friction gain
  PID pid;                // pid feedback term
  float output = 0;
  float previous_target;

  #if LOG_VELOCITY_CONTROLLER
    std::vector<Vector2> ff_log;
    std::vector<Vector2> fb_log;
  #endif

  VelocityController(float target, float kv, float ka, float kf, float kp, float ki, float kd, float integral_range, float dt = 10.0f);
  VelocityController(float target, float kv, float ka, float kf, const PIDConfig& pid, float dt = 10.0f);
  VelocityController(float target, const VelocityControllerConfig& config, float dt = 10.0f);

  float compute(float target, float error);
};