#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace panda_controllers
{
// Symmetric per-joint magnitude limiting plus a per-joint rate limit against
// the torque that was actually applied in the previous cycle.
//
// The limits come from panda_limits (Franka control-parameters page), not from
// a generic constant: tau_max = (87,87,87,87,12,12,12) Nm and
// tauD_max = 1000 Nm/s for every joint.
//
// A non-finite request is never forwarded: the previously applied torque is
// held, because zeroing every joint would drop a suspended arm.
struct TorqueLimitResult
{
  Eigen::Matrix<double, 7, 1> tau = Eigen::Matrix<double, 7, 1>::Zero();
  bool nonfinite_input{false};
  bool saturated{false};
  bool rate_limited{false};
};

inline TorqueLimitResult applyTorqueLimits(const Eigen::Matrix<double, 7, 1>& raw,
                                           const Eigen::Matrix<double, 7, 1>& previous,
                                           double dt,
                                           const Eigen::Matrix<double, 7, 1>& tau_max,
                                           const Eigen::Matrix<double, 7, 1>& tau_rate_max)
{
  TorqueLimitResult result;
  const Eigen::Matrix<double, 7, 1> prev =
      previous.allFinite() ? previous : Eigen::Matrix<double, 7, 1>::Zero();
  if (!raw.allFinite())
  {
    result.nonfinite_input = true;
    result.tau = prev;
    return result;
  }
  // A missing or absurd period must not turn into an unbounded step.
  if (!std::isfinite(dt) || dt <= 0.0) dt = 1e-3;
  dt = std::min(dt, 0.1);
  for (int i = 0; i < 7; ++i)
  {
    const double cap = std::abs(tau_max[i]);
    const double magnitude_limited = std::clamp(raw[i], -cap, cap);
    if (magnitude_limited != raw[i]) result.saturated = true;
    const double step = std::abs(tau_rate_max[i]) * dt;
    const double limited = std::clamp(magnitude_limited, prev[i] - step, prev[i] + step);
    if (limited != magnitude_limited) result.rate_limited = true;
    result.tau[i] = limited;
  }
  return result;
}

}  // namespace panda_controllers
