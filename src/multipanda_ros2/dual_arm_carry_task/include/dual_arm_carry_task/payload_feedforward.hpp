#pragma once

// Rod payload feedforward used by the MMC hold bridge.
//
// The rod is not part of the robot model (which runs with body gravcomp=1),
// so its weight is not compensated by the controller. This file holds the
// pure math: the per-arm share, the world->base force transform, the
// Jacobian mapping and the 0.5 s linear enable ramp. The controller applies
// J_linear^T F to the raw joint torque before the existing magnitude/rate
// limits; nothing here writes a force into the rod or the MuJoCo state.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace payload_ff
{
constexpr double kGravity = 9.81;

// Centred, symmetric grasp: each arm carries half the rod weight. Disabled or
// invalid mass gives exactly zero, i.e. the original control law.
inline double perArmForceN(double rod_mass_kg, bool enabled)
{
  if (!enabled || !std::isfinite(rod_mass_kg) || rod_mass_kg <= 0.0) return 0.0;
  return rod_mass_kg * kGravity * 0.5;
}

// A world-frame force expressed in an arm's link0 frame, whose axes are the
// ones the controller's zero Jacobian and pose error use: R_base_world^T F.
inline Eigen::Vector3d worldToBaseForce(const Eigen::Quaterniond& base_orientation_in_world,
                                        const Eigen::Vector3d& world_force)
{
  const Eigen::Quaterniond q = base_orientation_in_world.normalized();
  return q.conjugate() * world_force;
}

// Joint-space torque of a base-frame force through the 3x7 linear Jacobian.
inline Eigen::Matrix<double, 7, 1> jointTorque(const Eigen::Matrix<double, 3, 7>& linear_jacobian,
                                               const Eigen::Vector3d& base_force)
{
  return linear_jacobian.transpose() * base_force;
}

// Linear 0 -> 1 ramp in simulation time. A non-positive ramp time is treated
// as an immediate step so the caller cannot divide by zero.
inline double rampScale(double elapsed_s, double ramp_s)
{
  if (!std::isfinite(elapsed_s)) return 0.0;
  if (!(ramp_s > 0.0)) return elapsed_s >= 0.0 ? 1.0 : 0.0;
  return std::clamp(elapsed_s / ramp_s, 0.0, 1.0);
}

}  // namespace payload_ff
