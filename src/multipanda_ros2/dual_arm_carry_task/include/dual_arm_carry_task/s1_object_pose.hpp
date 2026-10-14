#pragma once

#include <cmath>

namespace s1
{
constexpr double kPi = 3.14159265358979323846;

// rod_target_yaw_deg is a world-frame absolute yaw. Return the shortest signed
// world-Z rotation that takes the measured initial yaw to that absolute target.
inline double absoluteYawDelta(double initial_yaw_rad, double target_yaw_rad)
{
    const double delta = target_yaw_rad - initial_yaw_rad;
    return std::atan2(std::sin(delta), std::cos(delta));
}

inline double absoluteYawReached(double initial_yaw_rad, double target_yaw_rad)
{
    const double reached = initial_yaw_rad + absoluteYawDelta(initial_yaw_rad, target_yaw_rad);
    return std::atan2(std::sin(reached), std::cos(reached));
}

// Geometric support gate for the existing MuJoCo body-truth interface. It
// accepts only the expected support-height band and rejects penetration.
inline bool supportHeightEligible(double center_z, double half_height,
                                  double support_z, double tolerance,
                                  double* clearance_out = nullptr)
{
    const double clearance = center_z - half_height - support_z;
    if (clearance_out) *clearance_out = clearance;
    return std::isfinite(clearance) && clearance >= -1e-4 &&
           clearance <= tolerance;
}
}  // namespace s1
