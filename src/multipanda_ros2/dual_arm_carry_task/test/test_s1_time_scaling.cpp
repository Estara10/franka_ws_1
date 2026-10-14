#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <dual_arm_carry_task/s1_time_scaling.hpp>

namespace
{
s1::DirectionalJointLimits limits()
{
  return {-2.0, 3.0, -4.0, 5.0, true, true};
}

std::vector<s1::TimedJointPoint> validPoints()
{
  return {{0.0, {0.1}, {0.0}, {0.0}},
          {2.0, {0.2}, {1.0}, {3.0}}};
}
}  // namespace

TEST(S1TimeScaling, StretchesFiniteOverspeedAndOveraccelerationAndPreservesGeometry)
{
  auto points = validPoints();
  const auto original = points;
  s1::TimeScalingReport report;
  std::string error;
  ASSERT_TRUE(s1::scaleTimedTrajectory(points, {limits()}, 0.15, 0.12, 0.01, report, error)) << error;

  // Effective positive limits are 0.45 rad/s and 0.60 rad/s^2.
  EXPECT_NEAR(report.velocity_ratio_before, 1.0 / 0.45, 1e-12);
  EXPECT_NEAR(report.acceleration_ratio_before, 3.0 / 0.60, 1e-12);
  EXPECT_NEAR(report.scale, std::sqrt(5.0) * 1.01, 1e-12);
  EXPECT_DOUBLE_EQ(report.duration_before, 2.0);
  EXPECT_NEAR(report.duration_after, 2.0 * report.scale, 1e-12);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(points[i].positions, original[i].positions);
    EXPECT_NEAR(points[i].time, original[i].time * report.scale, 1e-12);
    EXPECT_NEAR(points[i].velocities[0], original[i].velocities[0] / report.scale, 1e-12);
    EXPECT_NEAR(points[i].accelerations[0],
                original[i].accelerations[0] / (report.scale * report.scale), 1e-12);
  }
  EXPECT_LE(report.velocity_ratio_after, 1.0);
  EXPECT_LE(report.acceleration_ratio_after, 1.0);
}

TEST(S1TimeScaling, UsesDirectionalNegativeLimits)
{
  auto points = validPoints();
  points[1].velocities[0] = -0.5;
  points[1].accelerations[0] = -1.0;
  s1::TimeScalingReport report;
  std::string error;
  ASSERT_TRUE(s1::scaleTimedTrajectory(points, {limits()}, 0.15, 0.12, 0.01, report, error)) << error;
  EXPECT_NEAR(report.velocity_ratio_before, 0.5 / 0.30, 1e-12);
  EXPECT_NEAR(report.acceleration_ratio_before, 1.0 / 0.48, 1e-12);
  EXPECT_NEAR(report.scale, (0.5 / 0.30) * 1.01, 1e-12);
}

TEST(S1TimeScaling, RejectsNonFiniteValuesAndIllegalTime)
{
  s1::TimeScalingReport report;
  std::string error;
  auto non_finite = validPoints();
  non_finite[1].accelerations[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(s1::scaleTimedTrajectory(non_finite, {limits()}, 0.15, 0.12, 0.01, report, error));
  EXPECT_NE(error.find("non-finite"), std::string::npos);

  auto illegal_time = validPoints();
  illegal_time[1].time = illegal_time[0].time;
  EXPECT_FALSE(s1::scaleTimedTrajectory(illegal_time, {limits()}, 0.15, 0.12, 0.01, report, error));
  EXPECT_NE(error.find("non-increasing"), std::string::npos);
}

TEST(S1TimeScaling, RejectsMissingBoundsAndBadDimensions)
{
  s1::TimeScalingReport report;
  std::string error;
  auto no_bounds = limits();
  no_bounds.acceleration_bounded = false;
  auto no_bounds_points = validPoints();
  EXPECT_FALSE(s1::scaleTimedTrajectory(no_bounds_points, {no_bounds}, 0.15, 0.12, 0.01, report, error));

  auto bad_dimension = validPoints();
  bad_dimension[1].velocities.clear();
  EXPECT_FALSE(s1::scaleTimedTrajectory(bad_dimension, {limits()}, 0.15, 0.12, 0.01, report, error));
}
