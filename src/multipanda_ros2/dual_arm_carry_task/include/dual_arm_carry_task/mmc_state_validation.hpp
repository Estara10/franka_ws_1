#pragma once

#include <franka_msgs/msg/franka_state.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <cstdint>
#include <utility>

namespace dual_arm_carry_task
{
struct MmcStateValidation
{
  bool valid{false};
  std::string reason{"invalid_response"};
};

inline MmcStateValidation validateJointSampleFreshness(
    double sample_simulation_time, double current_simulation_time,
    double wall_age_seconds, double max_wall_age_seconds = 0.25,
    double max_simulation_lag_seconds = 0.5)
{
  if (!std::isfinite(sample_simulation_time) || sample_simulation_time <= 0.0 ||
      !std::isfinite(current_simulation_time) || current_simulation_time < 0.0)
    return {false, "invalid_joint_sample_time"};
  if (!std::isfinite(wall_age_seconds) || wall_age_seconds < 0.0 ||
      wall_age_seconds > max_wall_age_seconds)
    return {false, "stale_joint_sample_wall"};
  const double lag = current_simulation_time - sample_simulation_time;
  if (!std::isfinite(lag) || lag < 0.0 || lag > max_simulation_lag_seconds)
    return {false, "stale_joint_sample_simulation"};
  return {true, "ok"};
}

// Replace the task's shared feedback cache only after the sample's simulation
// timestamp has been checked. Rejected samples leave both its sequence and
// original steady-clock receipt time untouched, so they cannot refresh age.
template<typename JointSample>
inline MmcStateValidation acceptJointSampleIfFresh(
    JointSample incoming, JointSample& cached, double sample_simulation_time,
    double current_simulation_time, double received_steady_seconds)
{
  const auto validation = validateJointSampleFreshness(
      sample_simulation_time, current_simulation_time, 0.0);
  if (!validation.valid) return validation;
  incoming.seq = cached.seq + 1;
  incoming.recv_steady_sec = received_steady_seconds;
  incoming.stamp_valid = true;
  cached = std::move(incoming);
  return {true, "ok"};
}

inline std::int64_t systemWallNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

inline MmcStateValidation validateMmcRobotStates(
    const std::vector<franka_msgs::msg::FrankaState>& states,
    double current_simulation_time,
    std::int64_t now_wall_ns = systemWallNowNs(), std::chrono::nanoseconds max_age =
        std::chrono::milliseconds(500), double max_sample_lag_sec = 0.5)
{
  if (states.size() != 2) return {false, "invalid_arm_count_or_order"};
  const char* expected_frames[] = {"mj_left_link0", "mj_right_link0"};
  for (std::size_t i = 0; i < states.size(); ++i) {
    const auto& state = states[i];
    if (state.header.frame_id != expected_frames[i])
      return {false, i == 0 ? "source_mismatch_left" : "source_mismatch_right"};
    const std::int64_t stamp_ns = static_cast<std::int64_t>(state.header.stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(state.header.stamp.nanosec);
    if (stamp_ns <= 0 || now_wall_ns < stamp_ns || now_wall_ns - stamp_ns > max_age.count())
      return {false, i == 0 ? "stale_state_left" : "stale_state_right"};
    if (!std::isfinite(state.time) || state.time < 0.0 ||
        !std::isfinite(current_simulation_time) || current_simulation_time < 0.0 ||
        std::abs(current_simulation_time - state.time) > max_sample_lag_sec)
      return {false, i == 0 ? "stale_state_sample_left" : "stale_state_sample_right"};
    if (!std::all_of(state.o_t_ee.begin(), state.o_t_ee.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(state.q.begin(), state.q.end(),
                     [](double value) { return std::isfinite(value); }))
      return {false, "invalid_non_finite_pose_or_q"};
    const Eigen::Matrix4d transform = Eigen::Matrix4d::Map(state.o_t_ee.data());
    const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    const Eigen::Quaterniond quaternion(rotation);
    if (!transform.allFinite() || std::abs(transform(3, 0)) > 1e-6 ||
        std::abs(transform(3, 1)) > 1e-6 || std::abs(transform(3, 2)) > 1e-6 ||
        std::abs(transform(3, 3) - 1.0) > 1e-6 || !quaternion.coeffs().allFinite() ||
        quaternion.norm() < 0.99 || quaternion.norm() > 1.01 ||
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-3 ||
        std::abs(rotation.determinant() - 1.0) > 1e-3)
      return {false, "invalid_transform_or_quaternion"};
  }
  return {true, "ok"};
}
}  // namespace dual_arm_carry_task
