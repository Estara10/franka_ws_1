#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace s1
{
struct TimedJointPoint
{
  double time{0.0};
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

struct DirectionalJointLimits
{
  double min_velocity{0.0};
  double max_velocity{0.0};
  double min_acceleration{0.0};
  double max_acceleration{0.0};
  bool velocity_bounded{false};
  bool acceleration_bounded{false};
};

struct TimeScalingReport
{
  double velocity_ratio_before{0.0};
  double acceleration_ratio_before{0.0};
  double velocity_ratio_after{0.0};
  double acceleration_ratio_after{0.0};
  double scale{1.0};
  double duration_before{0.0};
  double duration_after{0.0};
};

inline bool scaleTimedTrajectory(std::vector<TimedJointPoint>& points,
                                 const std::vector<DirectionalJointLimits>& limits,
                                 const double velocity_scale,
                                 const double acceleration_scale,
                                 const double margin,
                                 TimeScalingReport& report,
                                 std::string& error)
{
  error.clear();
  report = {};
  if (points.size() < 2 || limits.empty() || !std::isfinite(velocity_scale) ||
      !std::isfinite(acceleration_scale) || !std::isfinite(margin) || velocity_scale <= 0.0 ||
      acceleration_scale <= 0.0 || margin < 0.0) {
    error = "invalid trajectory/scaling arguments";
    return false;
  }

  for (std::size_t j = 0; j < limits.size(); ++j) {
    const auto& limit = limits[j];
    if (!limit.velocity_bounded || !limit.acceleration_bounded ||
        !std::isfinite(limit.min_velocity) || !std::isfinite(limit.max_velocity) ||
        !std::isfinite(limit.min_acceleration) || !std::isfinite(limit.max_acceleration) ||
        limit.min_velocity >= 0.0 || limit.max_velocity <= 0.0 ||
        limit.min_acceleration >= 0.0 || limit.max_acceleration <= 0.0) {
      error = "missing or invalid directional joint limits at index " + std::to_string(j);
      return false;
    }
  }

  const auto dimension = limits.size();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& point = points[i];
    if (!std::isfinite(point.time) || point.time < 0.0 ||
        point.positions.size() != dimension || point.velocities.size() != dimension ||
        point.accelerations.size() != dimension) {
      error = "invalid time or derivative dimension at point " + std::to_string(i);
      return false;
    }
    if (i > 0 && !(point.time > points[i - 1].time)) {
      error = "non-increasing time at point " + std::to_string(i);
      return false;
    }
    for (std::size_t j = 0; j < dimension; ++j) {
      if (!std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j]) ||
          !std::isfinite(point.accelerations[j])) {
        error = "non-finite trajectory value at point " + std::to_string(i) +
            " joint " + std::to_string(j);
        return false;
      }
    }
  }

  report.duration_before = points.back().time - points.front().time;
  for (const auto& point : points) {
    for (std::size_t j = 0; j < dimension; ++j) {
      const auto& limit = limits[j];
      const double v_bound = point.velocities[j] >= 0.0 ?
          velocity_scale * limit.max_velocity : velocity_scale * -limit.min_velocity;
      const double a_bound = point.accelerations[j] >= 0.0 ?
          acceleration_scale * limit.max_acceleration : acceleration_scale * -limit.min_acceleration;
      if (!(v_bound > 0.0) || !(a_bound > 0.0)) {
        error = "non-positive effective joint limit";
        return false;
      }
      report.velocity_ratio_before = std::max(report.velocity_ratio_before,
          std::abs(point.velocities[j]) / v_bound);
      report.acceleration_ratio_before = std::max(report.acceleration_ratio_before,
          std::abs(point.accelerations[j]) / a_bound);
    }
  }

  report.scale = std::max({1.0, report.velocity_ratio_before,
                           std::sqrt(report.acceleration_ratio_before)});
  if (report.scale > 1.0) report.scale *= (1.0 + margin);
  if (!std::isfinite(report.scale) || report.scale < 1.0) {
    error = "invalid computed time scale";
    return false;
  }
  if (report.scale > 1.0) {
    const double scale_sq = report.scale * report.scale;
    for (auto& point : points) {
      point.time *= report.scale;
      for (auto& velocity : point.velocities) velocity /= report.scale;
      for (auto& acceleration : point.accelerations) acceleration /= scale_sq;
    }
  }

  report.duration_after = points.back().time - points.front().time;
  for (const auto& point : points) {
    for (std::size_t j = 0; j < dimension; ++j) {
      const auto& limit = limits[j];
      const double v_bound = point.velocities[j] >= 0.0 ?
          velocity_scale * limit.max_velocity : velocity_scale * -limit.min_velocity;
      const double a_bound = point.accelerations[j] >= 0.0 ?
          acceleration_scale * limit.max_acceleration : acceleration_scale * -limit.min_acceleration;
      report.velocity_ratio_after = std::max(report.velocity_ratio_after,
          std::abs(point.velocities[j]) / v_bound);
      report.acceleration_ratio_after = std::max(report.acceleration_ratio_after,
          std::abs(point.accelerations[j]) / a_bound);
    }
  }
  if (report.velocity_ratio_after > 1.0 + 1e-9 || report.acceleration_ratio_after > 1.0 + 1e-9) {
    error = "time-scaled trajectory still exceeds effective derivative limits";
    return false;
  }
  return true;
}
}  // namespace s1
