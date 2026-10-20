// Focused tests for the rod payload feedforward math: mass split, force
// coordinates, Jacobian mapping direction, the disabled branch and the
// enable ramp. The controller-side limit integration is the existing
// applyTorqueLimits path, covered by test_torque_limits.
#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>

#include "dual_arm_carry_task/payload_feedforward.hpp"

namespace
{
constexpr double kEps = 1e-12;
}

TEST(PayloadFeedforward, DisabledBranchIsExactlyZero)
{
  EXPECT_EQ(payload_ff::perArmForceN(0.64, false), 0.0);
  EXPECT_EQ(payload_ff::perArmForceN(0.0, true), 0.0);
  EXPECT_EQ(payload_ff::perArmForceN(-1.0, true), 0.0);
  EXPECT_EQ(payload_ff::perArmForceN(std::numeric_limits<double>::quiet_NaN(), true), 0.0);
}

TEST(PayloadFeedforward, MassIsSplitEquallyBetweenTheTwoArms)
{
  const double per_arm = payload_ff::perArmForceN(0.64, true);
  EXPECT_NEAR(per_arm, 0.64 * 9.81 / 2.0, kEps);
  EXPECT_NEAR(per_arm, 3.1392, 1e-4);
  EXPECT_NEAR(2.0 * per_arm, 0.64 * 9.81, 1e-12);
}

TEST(PayloadFeedforward, WorldForceTransformsIntoTheArmBaseFrame)
{
  // Base rotated +90 deg about world X: base z points along world -y, so a
  // world +Z force is +Y in the base frame.
  const Eigen::Quaterniond base(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d in_base =
      payload_ff::worldToBaseForce(base, Eigen::Vector3d(0.0, 0.0, 3.0));
  EXPECT_NEAR(in_base.x(), 0.0, 1e-12);
  EXPECT_NEAR(in_base.y(), 3.0, 1e-12);
  EXPECT_NEAR(in_base.z(), 0.0, 1e-12);
  // Identity base (the two mj_*_link0 frames in this scene) is a no-op.
  const Eigen::Vector3d same = payload_ff::worldToBaseForce(
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 3.0));
  EXPECT_NEAR((same - Eigen::Vector3d(0.0, 0.0, 3.0)).norm(), 0.0, 1e-12);
}

TEST(PayloadFeedforward, JacobianMappingDirectionIsLinearTranspose)
{
  Eigen::Matrix<double, 3, 7> jacobian = Eigen::Matrix<double, 3, 7>::Zero();
  jacobian(0, 1) = 1.0;  // base x velocity from joint 2
  jacobian(1, 3) = 1.0;  // base y velocity from joint 4
  jacobian(2, 5) = 1.0;  // base z velocity from joint 6
  const Eigen::Vector3d force(0.0, 0.0, 3.0);
  const Eigen::Matrix<double, 7, 1> tau = payload_ff::jointTorque(jacobian, force);
  EXPECT_NEAR(tau[5], 3.0, 1e-12);
  for (int i = 0; i < 7; ++i) {
    if (i != 5) { EXPECT_NEAR(tau[i], 0.0, 1e-12); }
  }
  // Opposite force gives opposite joint torque.
  const Eigen::Matrix<double, 7, 1> opposite =
      payload_ff::jointTorque(jacobian, Eigen::Vector3d(0.0, 0.0, -3.0));
  EXPECT_NEAR(opposite[5], -3.0, 1e-12);
}

TEST(PayloadFeedforward, RampIsLinearAndSaturates)
{
  EXPECT_NEAR(payload_ff::rampScale(0.0, 0.5), 0.0, kEps);
  EXPECT_NEAR(payload_ff::rampScale(0.25, 0.5), 0.5, kEps);
  EXPECT_NEAR(payload_ff::rampScale(0.5, 0.5), 1.0, kEps);
  EXPECT_NEAR(payload_ff::rampScale(2.0, 0.5), 1.0, kEps);
  EXPECT_NEAR(payload_ff::rampScale(-0.1, 0.5), 0.0, kEps);
  EXPECT_NEAR(payload_ff::rampScale(0.0, 0.0), 1.0, kEps);
}
