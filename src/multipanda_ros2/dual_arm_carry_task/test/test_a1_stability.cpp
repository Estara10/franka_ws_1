// A1 P1a offline tests for the settling criterion.
//
// The negative cases matter most: a paused simulation read repeatedly, a slow
// monotonic drift, a clock jump and a window with too little simulated time
// must all fail. None of these require a simulator.

#include "dual_arm_carry_task/a1_sampling.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace {

constexpr double kRodRestZ = 0.02;

a1::StabilityCriteria baseCriteria()
{
  a1::StabilityCriteria c;
  c.min_sim_window_sec = 0.10;
  c.max_window_pos_tol = 0.0005;
  c.max_window_angle_tol_deg = 0.20;
  c.max_twist_linear_tol = 0.01;
  c.max_twist_angular_tol = 0.05;
  c.resting_z_tol = 0.003;
  c.resting_tilt_tol_deg = 2.0;
  c.min_clock_advance_ratio = 0.5;
  return c;
}

// Builds a window of `n` samples with the given per-sample step in simulated
// time and wall time, a pose function and a twist magnitude.
std::vector<a1::PoseSample> makeWindow(int n,
                                       double dt_sim,
                                       double dt_steady,
                                       double z,
                                       double yaw,
                                       double twist_lin,
                                       double twist_ang,
                                       bool ros_valid = true)
{
  std::vector<a1::PoseSample> w;
  w.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    a1::PoseSample s;
    s.ros_time_valid = ros_valid;
    s.ros_time_sec = ros_valid ? dt_sim * static_cast<double>(i) : 0.0;
    s.steady_time_sec = dt_steady * static_cast<double>(i);
    s.pos = {0.50, 0.00, z};
    s.quat = a1::quatFromYaw(yaw);
    s.twist_lin_norm = twist_lin;
    s.twist_ang_norm = twist_ang;
    s.twist_valid = true;
    w.push_back(s);
  }
  return w;
}

}  // namespace

TEST(A1Stability, AcceptsAQuietRestingWindow)
{
  const auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.1, 0.0, 0.0);
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::STABLE) << r.reason_code << ": " << r.detail;
  EXPECT_EQ(r.reason_code, "E_OK");
  EXPECT_NEAR(r.sim_window_sec, 0.14, 1e-12);
  EXPECT_NEAR(r.clock_advance_ratio, 1.0, 1e-12);
}

// A paused simulation read repeatedly looks perfectly "steady" if you only
// compare adjacent samples. It must still be rejected.
TEST(A1Stability, RejectsFrozenClockEvenWhenReadsLookSteady)
{
  const auto w = makeWindow(8, 0.0, 0.02, kRodRestZ, 0.1, 0.0, 0.0);
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_NE(r.outcome, a1::StabilityOutcome::STABLE);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::CLOCK_NOT_ADVANCING);
  EXPECT_EQ(r.reason_code, "E_CLOCK_NOT_ADVANCING");
}

TEST(A1Stability, RejectsPausedFlagFromSimInfo)
{
  const auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  const a1::StabilityResult r = a1::evaluateStability(w, baseCriteria(), kRodRestZ, true, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::PAUSED);
  EXPECT_EQ(r.reason_code, "E_SIM_PAUSED");
}

TEST(A1Stability, RejectsClockJumpBackwards)
{
  auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  w[4].ros_time_sec = w[3].ros_time_sec - 0.5;  // clock jump
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::CLOCK_JUMP);
  EXPECT_EQ(r.reason_code, "E_CLOCK_JUMP");
}

TEST(A1Stability, RejectsInvalidSimulationClock)
{
  const auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0, /*ros_valid=*/false);
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::CLOCK_INVALID);
}

TEST(A1Stability, RejectsWindowWithTooLittleSimulatedTime)
{
  // 8 samples at 5 ms of simulated time = 35 ms, below the 100 ms minimum,
  // even though wall time also advanced normally.
  const auto w = makeWindow(8, 0.005, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::SIM_WINDOW_TOO_SHORT);
}

TEST(A1Stability, RejectsSimulationAdvancingFarSlowerThanWallClock)
{
  // 8 samples at 20 ms sim / 200 ms wall = ratio 0.1, below 0.5.
  const auto w = makeWindow(8, 0.02, 0.2, kRodRestZ, 0.0, 0.0, 0.0);
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::CLOCK_RATIO_TOO_LOW);
}

// Slow monotonic creep: every adjacent difference is well under the
// tolerance, but the whole window moves far more than the limit.
TEST(A1Stability, RejectsSlowMonotonicDriftThatAdjacentChecksWouldMiss)
{
  auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  // 0.2 mm per sample: every adjacent difference is inside the 0.5 mm
  // tolerance, but over 8 samples the rod creeps 1.4 mm.
  const double per_step = 0.0002;
  for (std::size_t i = 0; i < w.size(); ++i) {
    w[i].pos.x += per_step * static_cast<double>(i);
  }
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::DRIFT_EXCEEDED);
  EXPECT_EQ(r.reason_code, "E_DRIFT_EXCEEDED");
  EXPECT_GT(r.max_window_pos_delta, baseCriteria().max_window_pos_tol);
  // Adjacent differences alone would have stayed inside the tolerance.
  EXPECT_LT(per_step, baseCriteria().max_window_pos_tol);
}

TEST(A1Stability, RejectsOrientationDrift)
{
  auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < w.size(); ++i) {
    w[i].quat = a1::quatFromYaw(0.02 * static_cast<double>(i));  // ~1.15 deg/sample
  }
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::DRIFT_EXCEEDED);
  EXPECT_GT(r.max_window_angle_delta_deg, baseCriteria().max_window_angle_tol_deg);
}

TEST(A1Stability, RejectsExcessiveTwist)
{
  auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  w[3].twist_lin_norm = 0.5;
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::TWIST_EXCEEDED);

  auto w2 = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  w2[2].twist_ang_norm = 1.0;
  EXPECT_EQ(a1::evaluateStability(w2, baseCriteria(), kRodRestZ, false, false).outcome,
            a1::StabilityOutcome::TWIST_EXCEEDED);
}

TEST(A1Stability, RejectsNonFiniteTwistSample)
{
  auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  w[1].twist_valid = false;
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::INVALID_TWIST);
}

TEST(A1Stability, RejectsRodThatIsNotRestingAtTheExpectedHeight)
{
  const double bad_z = kRodRestZ + 0.05;
  const auto w = makeWindow(8, 0.02, 0.02, bad_z, 0.0, 0.0, 0.0);
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::NOT_RESTING);
}

TEST(A1Stability, RejectsRodThatIsNotLyingFlat)
{
  // 10 degrees of roll: still "stable" in position but not flat on the floor.
  a1::Quat tilted = a1::quatFromYaw(0.0);
  const double half = (10.0 * M_PI / 180.0) / 2.0;
  tilted.x = std::sin(half);
  tilted.w = std::cos(half);
  auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  for (auto& s : w) {
    s.quat = tilted;
  }
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::NOT_RESTING);
}

TEST(A1Stability, RejectsModelReloadBeforeAnythingElse)
{
  const auto w = makeWindow(8, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  const a1::StabilityResult r = a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, true);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::MODEL_RELOADED);
  EXPECT_EQ(r.reason_code, "E_MODEL_RELOADED");
}

TEST(A1Stability, RejectsTooFewSamples)
{
  const auto w = makeWindow(2, 0.02, 0.02, kRodRestZ, 0.0, 0.0, 0.0);
  const a1::StabilityResult r =
      a1::evaluateStability(w, baseCriteria(), kRodRestZ, false, false);
  EXPECT_EQ(r.outcome, a1::StabilityOutcome::INSUFFICIENT_SAMPLES);
}

TEST(A1Stability, ReasonCodesAreDistinctPerOutcome)
{
  EXPECT_EQ(a1::stabilityReasonCode(a1::StabilityOutcome::STABLE), "E_OK");
  EXPECT_EQ(a1::stabilityReasonCode(a1::StabilityOutcome::MODEL_RELOADED), "E_MODEL_RELOADED");
  EXPECT_EQ(a1::stabilityReasonCode(a1::StabilityOutcome::CLOCK_JUMP), "E_CLOCK_JUMP");
  EXPECT_EQ(a1::stabilityReasonCode(a1::StabilityOutcome::PAUSED), "E_SIM_PAUSED");
  EXPECT_EQ(a1::stabilityReasonCode(a1::StabilityOutcome::NOT_RESTING), "E_NOT_RESTING");
}
