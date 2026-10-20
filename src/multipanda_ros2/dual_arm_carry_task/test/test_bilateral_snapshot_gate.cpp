#include <gtest/gtest.h>
#include <limits>

#include "dual_arm_carry_task/bilateral_snapshot_gate.hpp"

using dual_arm_carry_task::BilateralGateArm;
using dual_arm_carry_task::BilateralGateResult;
using dual_arm_carry_task::evaluateBilateralSnapshot;

namespace
{
std::array<BilateralGateArm, 2> eligible()
{
  std::array<BilateralGateArm, 2> arms{};
  for (std::size_t i = 0; i < arms.size(); ++i) {
    arms[i].seq = 11 + i;
    arms[i].simulation_time = 3.0 + 0.001 * i;
    arms[i].age_seconds = 0.01;
    arms[i].valid = true;
    arms[i].left_contact = true;
    arms[i].right_contact = true;
    arms[i].left_normal_n = 0.5;
    arms[i].right_normal_n = 0.7;
  }
  return arms;
}
constexpr std::array<std::uint64_t, 2> kFence{{10, 10}};
}  // namespace

TEST(BilateralSnapshotGate, AcceptsOneFreshQualifiedSnapshot)
{
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, eligible()), BilateralGateResult::kPass);
}

TEST(BilateralSnapshotGate, RejectsActionFailure)
{
  EXPECT_EQ(evaluateBilateralSnapshot(false, true, kFence, eligible()), BilateralGateResult::kActionFailed);
}

TEST(BilateralSnapshotGate, RejectsOldAndRepeatedCachedSequence)
{
  auto arms = eligible();
  arms[0].seq = kFence[0];
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, arms), BilateralGateResult::kNotFresh);
  arms[0].seq = 11;
  arms[1].seq = kFence[1];
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, arms), BilateralGateResult::kNotFresh);
}

TEST(BilateralSnapshotGate, RejectsExcessiveSimulationTimeSkew)
{
  auto arms = eligible();
  arms[1].simulation_time += 0.020001;
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, arms), BilateralGateResult::kTimeSkew);
}

TEST(BilateralSnapshotGate, RejectsMissingContact)
{
  auto arms = eligible();
  arms[1].right_contact = false;
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, arms), BilateralGateResult::kMissingContact);
}

TEST(BilateralSnapshotGate, RejectsLowOrNonFiniteNormalForce)
{
  auto arms = eligible();
  arms[0].left_normal_n = 0.249;
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, arms),
            BilateralGateResult::kInsufficientNormalForce);
  arms = eligible();
  arms[1].right_normal_n = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, arms), BilateralGateResult::kInvalid);
}

TEST(BilateralSnapshotGate, RejectsStaleSnapshot)
{
  auto arms = eligible();
  arms[0].age_seconds = 0.250001;
  EXPECT_EQ(evaluateBilateralSnapshot(true, true, kFence, arms), BilateralGateResult::kInvalid);
}
