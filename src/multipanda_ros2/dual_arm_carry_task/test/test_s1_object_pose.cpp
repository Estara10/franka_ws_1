#include <gtest/gtest.h>

#include <dual_arm_carry_task/s1_object_pose.hpp>

TEST(S1ObjectPose, AbsoluteWorldYawFromNonzeroInitialYaw)
{
  constexpr double deg = s1::kPi / 180.0;
  EXPECT_NEAR(s1::absoluteYawDelta(10.0 * deg, 90.0 * deg), 80.0 * deg, 1e-12);
  EXPECT_NEAR(s1::absoluteYawReached(10.0 * deg, 90.0 * deg), 90.0 * deg, 1e-12);
}

TEST(S1ObjectPose, UsesShortestWrappedWorldRotation)
{
  constexpr double deg = s1::kPi / 180.0;
  EXPECT_NEAR(s1::absoluteYawDelta(170.0 * deg, -170.0 * deg), 20.0 * deg, 1e-12);
}

TEST(S1ObjectPose, SupportBandRejectsHighPoseAndGroundPenetration)
{
  double clearance = 0.0;
  EXPECT_TRUE(s1::supportHeightEligible(0.0207, 0.020, 0.0, 0.003, &clearance));
  EXPECT_NEAR(clearance, 0.0007, 1e-12);
  EXPECT_FALSE(s1::supportHeightEligible(0.030, 0.020, 0.0, 0.003));
  EXPECT_FALSE(s1::supportHeightEligible(0.019, 0.020, 0.0, 0.003));
}
