#include "vex.h"


class SteerCommand {
public:
    SteerCommand() = default;
    SteerCommand(float v, float omega);
    
    SteerCommand operator+(const SteerCommand& sc) const;
    SteerCommand operator-(const SteerCommand& sc) const;

    SteerCommand operator*(float scalar) const;
    SteerCommand operator/(float scalar) const;

    SteerCommand operator-() const;

    SteerCommand operator+=(const SteerCommand& sc);
    SteerCommand operator-=(const SteerCommand& sc);

    SteerCommand operator*=(float scalar);
    SteerCommand operator/=(float scalar);

    float v;
    float omega;
};