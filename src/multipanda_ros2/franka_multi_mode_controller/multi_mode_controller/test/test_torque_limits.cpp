#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <multi_mode_controller/utils/panda_limits.h>
#include <multi_mode_controller/utils/torque_limits.h>

// Pure-math coverage for the impedance torque protection. The controller path
// itself is checked in the no-load simulation run.

namespace
{
using Vec7 = Eigen::Matrix<double, 7, 1>;
}

TEST(ImpedanceTorqueLimits, SymmetricMagnitudeLimitUsesTheDeclaredJointLimits)
{
  Vec7 raw;
  raw << 200.0, -200.0, 50.0, -50.0, 30.0, -30.0, 13.0;
  const Vec7 previous = Vec7::Zero();
  // A long period so the rate limit cannot bind and only the magnitude does.
  const auto result = panda_controllers::applyTorqueLimits(
      raw, previous, 10.0, panda_limits::tau_max, panda_limits::tauD_max);

  EXPECT_TRUE(result.saturated);
  EXPECT_FALSE(result.rate_limited);
  // Both signs are clipped: this is not min(10, tau).
  EXPECT_DOUBLE_EQ(result.tau[0], 87.0);
  EXPECT_DOUBLE_EQ(result.tau[1], -87.0);
  EXPECT_DOUBLE_EQ(result.tau[2], 50.0);
  EXPECT_DOUBLE_EQ(result.tau[3], -50.0);
  EXPECT_DOUBLE_EQ(result.tau[4], 12.0);
  EXPECT_DOUBLE_EQ(result.tau[5], -12.0);
  EXPECT_DOUBLE_EQ(result.tau[6], 12.0);
}

TEST(ImpedanceTorqueLimits, RateLimitBoundsTheStepInBothDirections)
{
  Vec7 raw;
  raw << 87.0, -87.0, 0.0, 0.0, 12.0, -12.0, 0.0;
  Vec7 previous;
  previous << 0.0, 0.0, 5.0, -5.0, 0.0, 0.0, 0.0;
  const double dt = 0.001;  // 1 kHz control loop
  const auto result = panda_controllers::applyTorqueLimits(
      raw, previous, dt, panda_limits::tau_max, panda_limits::tauD_max);

  EXPECT_TRUE(result.rate_limited);
  // 1000 Nm/s * 1 ms = 1 Nm per cycle, in both directions.
  EXPECT_NEAR(result.tau[0], 1.0, 1e-12);
  EXPECT_NEAR(result.tau[1], -1.0, 1e-12);
  // Moving TOWARDS zero is also limited to one step: 5 -> 4, not 5 -> 6.
  EXPECT_NEAR(result.tau[2], 4.0, 1e-12);
  EXPECT_NEAR(result.tau[3], -4.0, 1e-12);
  EXPECT_NEAR(result.tau[4], 1.0, 1e-12);
  EXPECT_NEAR(result.tau[5], -1.0, 1e-12);

  // A longer period really does allow a larger step.
  const auto coarse = panda_controllers::applyTorqueLimits(
      raw, previous, 0.05, panda_limits::tau_max, panda_limits::tauD_max);
  EXPECT_NEAR(coarse.tau[0], 50.0, 1e-9);
}

TEST(ImpedanceTorqueLimits, NonFiniteRequestHoldsThePreviousTorque)
{
  Vec7 raw = Vec7::Constant(std::numeric_limits<double>::quiet_NaN());
  Vec7 previous;
  previous << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0;
  const auto result = panda_controllers::applyTorqueLimits(
      raw, previous, 0.001, panda_limits::tau_max, panda_limits::tauD_max);

  EXPECT_TRUE(result.nonfinite_input);
  // Holding is required: zeroing every joint would drop a suspended arm.
  for (int i = 0; i < 7; ++i) EXPECT_DOUBLE_EQ(result.tau[i], previous[i]);
  EXPECT_TRUE(result.tau.allFinite());
}

TEST(ImpedanceTorqueLimits, NonFinitePreviousStateFallsBackToZero)
{
  Vec7 raw = Vec7::Zero();
  Vec7 previous = Vec7::Constant(std::numeric_limits<double>::infinity());
  const auto result = panda_controllers::applyTorqueLimits(
      raw, previous, 0.001, panda_limits::tau_max, panda_limits::tauD_max);
  EXPECT_TRUE(result.tau.allFinite());
  for (int i = 0; i < 7; ++i) EXPECT_DOUBLE_EQ(result.tau[i], 0.0);
}

TEST(ImpedanceTorqueLimits, InvalidPeriodFallsBackToABoundedStep)
{
  Vec7 raw = Vec7::Constant(87.0);
  const Vec7 previous = Vec7::Zero();
  for (const double dt : {0.0, -0.001, std::numeric_limits<double>::quiet_NaN(), 5.0}) {
    const auto result = panda_controllers::applyTorqueLimits(
        raw, previous, dt, panda_limits::tau_max, panda_limits::tauD_max);
    EXPECT_TRUE(result.tau.allFinite());
    EXPECT_LE(std::abs(result.tau[0]), 87.0);
    EXPECT_GE(result.tau[0], 1.0 - 1e-12);
  }
}

// The controller adds the payload feedforward to the raw impedance torque
// before this function. Its sum is limited exactly like any other request:
// the feedforward does not bypass the magnitude or rate limit.
TEST(ImpedanceTorqueLimits, PayloadFeedforwardSharesTheSameLimiter)
{
  Vec7 impedance;
  impedance << 80.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0;
  Vec7 payload_ff;
  payload_ff << 20.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0;
  const auto result = panda_controllers::applyTorqueLimits(
      impedance + payload_ff, Vec7::Zero(), 10.0, panda_limits::tau_max, panda_limits::tauD_max);
  EXPECT_TRUE(result.saturated);
  EXPECT_DOUBLE_EQ(result.tau[0], 87.0);
  EXPECT_DOUBLE_EQ(result.tau[4], 12.0);
}
