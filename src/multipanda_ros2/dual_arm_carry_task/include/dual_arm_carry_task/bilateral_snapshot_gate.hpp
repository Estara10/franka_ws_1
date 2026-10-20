#pragma once

#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>

namespace dual_arm_carry_task
{
struct BilateralGateArm
{
  std::uint64_t seq{0};
  double simulation_time{0.0};
  double age_seconds{0.0};
  bool valid{false};
  bool left_contact{false};
  bool right_contact{false};
  double left_normal_n{0.0};
  double right_normal_n{0.0};
};

enum class BilateralGateResult
{
  kPass,
  kActionFailed,
  kNotFresh,
  kInvalid,
  kTimeSkew,
  kMissingContact,
  kInsufficientNormalForce
};

inline BilateralGateResult evaluateBilateralSnapshot(
    bool left_action_succeeded, bool right_action_succeeded,
    const std::array<std::uint64_t, 2>& action_completion_seq,
    const std::array<BilateralGateArm, 2>& arms)
{
  if (!left_action_succeeded || !right_action_succeeded) {
    return BilateralGateResult::kActionFailed;
  }
  for (std::size_t i = 0; i < arms.size(); ++i) {
    const auto& arm = arms[i];
    if (!arm.valid || !std::isfinite(arm.age_seconds) || arm.age_seconds < 0.0 ||
        !std::isfinite(arm.simulation_time) ||
        !std::isfinite(arm.left_normal_n) || !std::isfinite(arm.right_normal_n)) {
      return BilateralGateResult::kInvalid;
    }
    if (arm.age_seconds > 0.25) return BilateralGateResult::kInvalid;
    if (arm.seq <= action_completion_seq[i]) return BilateralGateResult::kNotFresh;
  }
  if (std::abs(arms[0].simulation_time - arms[1].simulation_time) > 0.020) {
    return BilateralGateResult::kTimeSkew;
  }
  if (!(arms[0].left_contact && arms[0].right_contact &&
        arms[1].left_contact && arms[1].right_contact)) {
    return BilateralGateResult::kMissingContact;
  }
  for (const auto& arm : arms) {
    if (arm.left_normal_n < 0.25 || arm.right_normal_n < 0.25) {
      return BilateralGateResult::kInsufficientNormalForce;
    }
  }
  return BilateralGateResult::kPass;
}
}  // namespace dual_arm_carry_task
