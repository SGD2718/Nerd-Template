#include "vex.h"

struct PathPoint {
  PathPoint() = default;
  PathPoint(Vector2 pos, float voltage): pos(pos), voltage(voltage) {}
  PathPoint(float x, float y, float voltage): pos(x, y), voltage(voltage) {}
  Vector2 pos;
  float voltage = 0;
};