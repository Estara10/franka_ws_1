#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <dual_arm_carry_task/grasp_contact_acceptance.hpp>

TEST(GraspContactAcceptance, RequiresFreshFullDurationWindowAndChecksRange) {
  a1g::JointStabilityWindow window(0.30, 0.002);
  std::vector<double> base(14, 0.0);
  base[6] = 0.0060;  // already displaced cumulatively before this observation
  EXPECT_TRUE(window.update(1, 10.0, base));
  EXPECT_FALSE(window.update(1, 10.31, base));  // duplicate feedback is not time
  EXPECT_FALSE(window.stable());
  std::vector<double> moved = base;
  moved[6] = 0.0069;
  EXPECT_TRUE(window.update(2, 10.15, moved));
  moved[6] = 0.0079;
  EXPECT_TRUE(window.update(3, 10.301, moved));
  EXPECT_TRUE(window.stable());
  EXPECT_NEAR(window.maxSpread(), 0.0019, 1e-12);
  double cumulative_delta = 0.0;
  for (std::size_t i = 0; i < moved.size(); ++i) {
    cumulative_delta = std::max(cumulative_delta, std::abs(moved[i]));
  }
  EXPECT_NEAR(cumulative_delta, 0.0079, 1e-12);
  EXPECT_GT(cumulative_delta, 0.002);  // reported separately; not the short-window verdict
}

TEST(GraspContactAcceptance, OldOrNonFiniteFeedbackCannotCompleteWindow) {
  a1g::JointStabilityWindow window(0.30, 0.002);
  const std::vector<double> values(14, 0.0);
  ASSERT_TRUE(window.update(10, 2.0, values));
  EXPECT_FALSE(window.update(9, 2.31, values));
  auto invalid = values;
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(window.update(11, 2.32, invalid));
  EXPECT_FALSE(window.stable());
}

TEST(GraspContactAcceptance, EndEffectorPositionAndAngleLimitsAreIndependent) {
  EXPECT_TRUE(a1g::poseWithinLimits(0.0049, 0.03, 0.005, 0.034906585));
  EXPECT_FALSE(a1g::poseWithinLimits(0.0051, 0.0, 0.005, 0.034906585));
  EXPECT_FALSE(a1g::poseWithinLimits(0.0, 0.035, 0.005, 0.034906585));
  EXPECT_FALSE(a1g::poseWithinLimits(std::numeric_limits<double>::infinity(), 0.0,
                                    0.005, 0.034906585));
}
