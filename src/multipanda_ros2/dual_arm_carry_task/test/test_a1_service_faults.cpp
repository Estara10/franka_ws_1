// A1 P1a offline tests: single-candidate orchestration under service faults.
//
// IMPORTANT: these use an in-process FAKE transport and a FAKE clock. They
// exercise the production orchestration code in a1_sampling.cpp, but they are
// NOT simulation runs and prove nothing about the real /set_body_state
// behaviour. Real-simulation evidence is recorded separately in
// docs/deliverables/A1_P1a_验证记录.md.
//
// The behaviour under test that matters most: a write that fails or times out
// must stop the run. It must not be retried and must not fall through to
// another candidate, because a ROS service timeout does not cancel
// server-side work and a failed response does not imply the write had no
// effect.

#include "dual_arm_carry_task/a1_sampling.hpp"

#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

a1::SamplingConfig baseConfig()
{
  a1::SamplingConfig cfg;
  cfg.algorithm_version = "a1-uniform-splitmix64-v1";
  cfg.sampling_range_version = "test-v1";
  cfg.seed_raw = 4242;
  cfg.x_min = 0.45;
  cfg.x_max = 0.55;
  cfg.y_min = -0.05;
  cfg.y_max = 0.05;
  cfg.yaw_deg_min = -15.0;
  cfg.yaw_deg_max = 15.0;
  cfg.rod_half_length = 0.20;
  cfg.rod_half_width = 0.02;
  cfg.rod_half_height = 0.02;
  cfg.support_surface_z = 0.0;
  cfg.apply_clearance_m = 0.002;
  cfg.max_total_candidates = 200;
  return cfg;
}

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

a1::ApplyPolicy basePolicy()
{
  a1::ApplyPolicy p;
  p.poll_rate_hz = 50.0;
  p.service_wait_timeout_sec = 1.0;
  p.stability_timeout_sec = 0.30;
  // Must not exceed the settling budget, or validation rejects the policy.
  p.service_call_timeout_sec = 0.10;
  p.sim_info_check_every = 2;
  p.max_window_samples = 50;
  p.max_write_attempts_per_run = 1;
  return p;
}

// Deterministic virtual clock. `sim_per_steady` lets a test simulate a paused
// or slowed simulation without touching real time.
class FakeClock : public a1::Clock {
 public:
  double steady {0.0};
  double ros {0.0};
  bool ros_valid {true};
  double sim_per_steady {1.0};
  // Optional hook so a test can act between polls (e.g. raise an abort flag).
  std::function<void(int)> on_sleep;
  int sleep_calls {0};

  // Lets a fake transport simulate blocking latency.
  void advance(double dt)
  {
    if (dt > 0.0) {
      steady += dt;
      ros += dt * sim_per_steady;
    }
  }

  double steadyNowSec() override { return steady; }
  double rosNowSec() override { return ros; }
  bool rosTimeValid() override { return ros_valid; }
  void sleepUntil(double target) override
  {
    if (target > steady) {
      const double delta = target - steady;
      steady += delta;
      ros += delta * sim_per_steady;
    }
    ++sleep_calls;
    if (on_sleep) {
      on_sleep(sleep_calls);
    }
  }
};

class FakeTransport : public a1::BodyStateTransport {
 public:
  explicit FakeTransport(FakeClock* clock = nullptr) : clock_(clock) {}

  // Scripted behaviour
  a1::WriteOutcome write_outcome {a1::WriteOutcome::SUCCESS};
  std::string write_status_message {"ok"};
  bool sim_info_ok {true};
  bool model_valid {true};
  bool state_ok {true};
  bool paused {false};
  uint32_t load_count {3};
  std::string model_path {"/fake/dual_scene.xml"};
  // When > 0, the load_count reported from the Nth sim-info call onward changes.
  int reload_at_sim_info_call {0};
  // When > 0, the model_path reported from the Nth sim-info call onward changes.
  int model_path_change_at_sim_info_call {0};
  // Model validity only from the Nth sim-info call onward.
  int model_invalid_at_sim_info_call {0};
  // Sim-info failure only from the Nth call onward.
  int sim_info_fail_at_call {0};
  // Seconds of blocking latency per read, used to overrun the budget.
  double read_latency_sec {0.0};
  // Latency of a sim-info query, applied from the Nth call onward (0 = never).
  // Used to overrun the budget while the final health query is in flight.
  int sim_info_latency_from_call {0};
  double sim_info_latency_sec {0.0};
  // Hook invoked with the call number, e.g. to cancel during the final query.
  std::function<void(int)> on_sim_info;
  // Latency of the write call itself.
  double write_latency_sec {0.005};

  // Observation
  int write_calls {0};
  int state_calls {0};
  int sim_info_calls {0};

  // Reported pose (defaults to the rod resting correctly)
  a1::Vec3 pos {0.50, 0.0, 0.02};
  a1::Quat quat {0.0, 0.0, 0.0, 1.0};

  bool waitForServices(double, std::string&) override { return true; }

  a1::SimInfo getSimInfo(double) override
  {
    ++sim_info_calls;
    a1::SimInfo info;
    info.ok = sim_info_ok;
    if (sim_info_fail_at_call > 0 && sim_info_calls >= sim_info_fail_at_call) {
      info.ok = false;
      info.status_message = "sim info unavailable";
      return info;
    }
    info.model_valid = model_valid;
    if (model_invalid_at_sim_info_call > 0 &&
        sim_info_calls >= model_invalid_at_sim_info_call) {
      info.model_valid = false;
    }
    info.model_path = model_path;
    if (model_path_change_at_sim_info_call > 0 &&
        sim_info_calls >= model_path_change_at_sim_info_call) {
      info.model_path = "/fake/OTHER_scene.xml";
    }
    info.paused = paused;
    info.load_count = load_count;
    if (reload_at_sim_info_call > 0 && sim_info_calls >= reload_at_sim_info_call) {
      info.load_count = load_count + 1;
    }
    if (on_sim_info) {
      on_sim_info(sim_info_calls);
    }
    if (clock_ != nullptr && sim_info_latency_from_call > 0 &&
        sim_info_calls >= sim_info_latency_from_call) {
      clock_->advance(sim_info_latency_sec);
    }
    return info;
  }

  a1::BodyState getState(double, a1::PoseSample& timing_out) override
  {
    ++state_calls;
    timing_out.request_steady_sec = steady();
    timing_out.request_ros_valid = rosValid();
    timing_out.request_ros_sec = timing_out.request_ros_valid ? ros() : 0.0;
    if (clock_ != nullptr) {
      clock_->advance(read_latency_sec);
    }
    timing_out.steady_time_sec = steady();
    timing_out.ros_time_valid = rosValid();
    timing_out.ros_time_sec = timing_out.ros_time_valid ? ros() : 0.0;

    a1::BodyState st;
    st.ok = state_ok;
    st.pos = pos;
    st.quat = quat;
    st.twist_lin = {0.0, 0.0, 0.0};
    st.twist_ang = {0.0, 0.0, 0.0};
    st.mass = 0.64;
    st.pose_stamp_is_zero = true;
    if (!state_ok) {
      st.status_message = "get_body_state failed";
      timing_out.ros_time_valid = false;
    }
    return st;
  }

  a1::WriteOutcome setPose(const a1::Candidate&, double,
                           a1::BodyStateTransport::WriteTiming& timing,
                           std::string& status_message) override
  {
    ++write_calls;
    timing.request_ros_valid = rosValid();
    timing.request_ros_sec = timing.request_ros_valid ? ros() : 0.0;
    timing.request_steady_sec = steady();
    if (clock_ != nullptr) {
      clock_->advance(write_latency_sec);  // the call is not instantaneous
    }
    timing.response_ros_valid = rosValid();
    timing.response_ros_sec = timing.response_ros_valid ? ros() : 0.0;
    timing.response_steady_sec = steady();
    status_message = write_status_message;
    return write_outcome;
  }

 private:
  double steady() const { return clock_ != nullptr ? clock_->steadyNowSec() : 0.0; }
  double ros() const { return clock_ != nullptr ? clock_->rosNowSec() : 0.0; }
  bool rosValid() const { return clock_ != nullptr ? clock_->rosTimeValid() : true; }

  FakeClock* clock_ {nullptr};
};

}  // namespace

// ---------------------------------------------------------------------------
// Write failures: stop the run, never retry, never move to another candidate
// ---------------------------------------------------------------------------

TEST(A1ServiceFaults, WriteUnavailableStopsTheRun)
{
  FakeClock c;
  FakeTransport t(&c);
  t.write_outcome = a1::WriteOutcome::UNAVAILABLE;
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_APPLY_FAILED");
  EXPECT_EQ(r.outcome_code, "E_WRITE_UNAVAILABLE");
  EXPECT_TRUE(r.write_attempted);
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_TRUE(r.samples.empty());
}

TEST(A1ServiceFaults, WriteRejectedStopsTheRun)
{
  FakeClock c;
  FakeTransport t(&c);
  t.write_outcome = a1::WriteOutcome::REJECTED;
  t.write_status_message = "body not found";
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_APPLY_FAILED");
  EXPECT_EQ(r.outcome_code, "E_WRITE_REJECTED");
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_EQ(r.write_status_message, "body not found");
}

TEST(A1ServiceFaults, WriteTimeoutStopsTheRunWithoutRetryOrNextCandidate)
{
  FakeClock c;
  FakeTransport t(&c);
  t.write_outcome = a1::WriteOutcome::TIMEOUT;
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 7, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_APPLY_FAILED");
  EXPECT_EQ(r.outcome_code, "E_WRITE_TIMEOUT");
  // Exactly one write: a timeout must not be retried, and the run must not
  // advance to candidate 8.
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_EQ(r.candidate.index, 7);
  EXPECT_NE(r.detail.find("without retrying"), std::string::npos);
  EXPECT_TRUE(r.samples.empty());
}

// ---------------------------------------------------------------------------
// Preconditions: nothing is written when they fail
// ---------------------------------------------------------------------------

TEST(A1ServiceFaults, SimInfoUnavailableStopsBeforeAnyWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  t.sim_info_ok = false;
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.outcome_code, "E_SIM_INFO_UNAVAILABLE");
  EXPECT_EQ(t.write_calls, 0);
}

TEST(A1ServiceFaults, InvalidModelStopsBeforeAnyWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  t.model_valid = false;
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.outcome_code, "E_MODEL_INVALID");
  EXPECT_EQ(t.write_calls, 0);
}

TEST(A1ServiceFaults, BaselineReadFailureStopsBeforeAnyWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  t.state_ok = false;
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.outcome_code, "E_BASELINE_READ_FAILED");
  EXPECT_EQ(t.write_calls, 0);
}

TEST(A1ServiceFaults, BadCandidateIndexIsRejectedWithoutWriting)
{
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 999, t, c);

  EXPECT_EQ(r.scene_state, "NOT_RUN");
  EXPECT_EQ(r.outcome_code, "E_BAD_CANDIDATE_INDEX");
  EXPECT_EQ(t.write_calls, 0);
  EXPECT_FALSE(r.candidate_generated);
}

TEST(A1ServiceFaults, InvalidConfigIsRejectedWithoutWriting)
{
  a1::SamplingConfig cfg = baseConfig();
  cfg.x_max = std::numeric_limits<double>::quiet_NaN();
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r = a1::applyCandidateAndSettle(cfg, baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "NOT_RUN");
  EXPECT_EQ(r.outcome_code, "E_CONFIG_NOT_FINITE");
  EXPECT_EQ(t.write_calls, 0);
}

// ---------------------------------------------------------------------------
// Settling outcomes
// ---------------------------------------------------------------------------

TEST(A1ServiceFaults, HappyPathReachesSceneReadyUnvalidatedAfterExactlyOneWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 3, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_READY_UNVALIDATED") << r.outcome_code << ": " << r.detail;
  EXPECT_EQ(r.outcome_code, "SCENE_READY_UNVALIDATED");
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_FALSE(r.samples.empty());
  EXPECT_EQ(r.stability.outcome, a1::StabilityOutcome::STABLE);
  EXPECT_TRUE(r.readback.ok);
  // P1a never claims sample validity.
  EXPECT_NE(r.scene_state, "accepted");
}

TEST(A1ServiceFaults, SettleTimeoutFailsAfterExactlyOneWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  c.sim_per_steady = 0.0;  // simulated time never advances
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_SCENE_SETTLE_TIMEOUT");
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_FALSE(r.samples.empty());
  // The measured reason is preserved alongside the timeout.
  EXPECT_EQ(r.stability.reason_code, "E_CLOCK_NOT_ADVANCING");
}

TEST(A1ServiceFaults, NonRestingRodTimesOutInsteadOfSucceeding)
{
  FakeClock c;
  FakeTransport t(&c);
  t.pos = {0.50, 0.0, 0.20};  // hovering, not on the support surface
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_SCENE_SETTLE_TIMEOUT");
  EXPECT_EQ(r.stability.reason_code, "E_NOT_RESTING");
}

TEST(A1ServiceFaults, PausedSimulationIsRejectedBeforeAnyWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  t.paused = true;
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_APPLY_FAILED");
  EXPECT_EQ(r.outcome_code, "E_SIM_PAUSED_BEFORE_WRITE");
  EXPECT_EQ(t.write_calls, 0);
}

TEST(A1ServiceFaults, ModelReloadDuringSettlingIsAHardStop)
{
  FakeClock c;
  FakeTransport t(&c);
  t.reload_at_sim_info_call = 2;  // first in-loop query reports a new load_count
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_MODEL_RELOADED");
  EXPECT_EQ(t.write_calls, 1);
}

TEST(A1ServiceFaults, ReadbackIsNotUsedWhenItIsUnavailable)
{
  FakeClock c;
  FakeTransport t(&c);
  t.state_ok = false;  // baseline read fails, so we never reach settling
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);
  EXPECT_FALSE(r.readback.ok);
  EXPECT_EQ(t.write_calls, 0);
}

TEST(A1ServiceFaults, WriteTimingIsRecordedForASuccessfulWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);
  EXPECT_TRUE(r.write_timing.request_ros_valid);
  EXPECT_GT(r.write_timing.response_steady_sec, r.write_timing.request_steady_sec);
}

TEST(A1ServiceFaults, ExplicitCandidateUsesTheCandidateOverload)
{
  a1::Candidate explicit_candidate;
  ASSERT_TRUE(a1::makeExplicitCandidate(baseConfig(), 0, 0.51, 0.01, 0.0, explicit_candidate));
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(),
                                                      explicit_candidate, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_READY_UNVALIDATED") << r.outcome_code << ": " << r.detail;
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_DOUBLE_EQ(r.candidate.x, 0.51);
  EXPECT_DOUBLE_EQ(r.candidate.y, 0.01);
  EXPECT_DOUBLE_EQ(r.candidate.yaw_rad, 0.0);
}

TEST(A1ServiceFaults, NonFiniteExplicitCandidateIsRejectedWithoutWriting)
{
  a1::Candidate bad;
  bad.x = std::numeric_limits<double>::quiet_NaN();
  bad.y = 0.0;
  bad.z = 0.022;
  bad.quat = {0.0, 0.0, 0.0, 1.0};
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), bad, t, c);

  EXPECT_EQ(r.scene_state, "NOT_RUN");
  EXPECT_EQ(r.outcome_code, "E_BAD_CANDIDATE");
  EXPECT_EQ(t.write_calls, 0);
}

// An interrupted run must still be recorded, with the already-applied pose
// acknowledged, rather than disappearing without a manifest.
TEST(A1ServiceFaults, AbortFlagProducesAnInterruptedRunWithARecord)
{
  FakeClock c;
  FakeTransport t(&c);
  std::atomic<bool> abort {false};
  a1::ApplyPolicy p = basePolicy();
  p.abort_flag = &abort;

  // Request the abort after a few polls by steering the clock: the loop checks
  // the flag once per iteration.
  c.on_sleep = [&abort](int count) {
    if (count >= 3) {
      abort.store(true);
    }
  };

  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);

  EXPECT_EQ(r.scene_state, "INTERRUPTED");
  EXPECT_EQ(r.outcome_code, "E_INTERRUPTED");
  EXPECT_EQ(t.write_calls, 1);  // the write already happened and stands
  EXPECT_FALSE(r.samples.empty());
}

// ---------------------------------------------------------------------------
// Defects reproduced by the planner: cancellation and deadline ordering
// ---------------------------------------------------------------------------

TEST(A1ServiceFaults, PreCancelPerformsNoWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  std::atomic<bool> abort {true};  // already cancelled before the call
  a1::ApplyPolicy p = basePolicy();
  p.abort_flag = &abort;

  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);

  EXPECT_EQ(r.scene_state, "INTERRUPTED");
  EXPECT_EQ(r.outcome_code, "E_INTERRUPTED");
  // A run cancelled up front must not touch the simulation at all.
  EXPECT_EQ(t.write_calls, 0);
  EXPECT_FALSE(r.write_attempted);
}

TEST(A1ServiceFaults, CancelAfterWriteKeepsTheRecordAndDoesNotSucceed)
{
  FakeClock c;
  FakeTransport t(&c);
  std::atomic<bool> abort {false};
  a1::ApplyPolicy p = basePolicy();
  p.abort_flag = &abort;
  c.on_sleep = [&abort](int count) {
    if (count >= 3) {
      abort.store(true);
    }
  };

  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);

  EXPECT_EQ(r.scene_state, "INTERRUPTED");
  EXPECT_EQ(r.outcome_code, "E_INTERRUPTED");
  // The write already happened and stands: exactly one, never a second.
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_TRUE(r.write_attempted);
}

// Reproduced defect: a read that blocks past the settling budget was still
// followed by a STABLE verdict, because the deadline was only checked after the
// success branch. The read latency below is larger than the budget.
TEST(A1ServiceFaults, ReadBlockingPastTheBudgetIsNeverReportedAsSuccess)
{
  FakeClock c;
  FakeTransport t(&c);
  a1::ApplyPolicy p = basePolicy();
  p.stability_timeout_sec = 0.30;
  p.service_call_timeout_sec = 0.10;
  t.read_latency_sec = 0.12;  // each read overruns by more than the poll period

  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);

  EXPECT_NE(r.scene_state, "SCENE_READY_UNVALIDATED");
  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_SCENE_SETTLE_TIMEOUT");
  EXPECT_EQ(t.write_calls, 1);
}

TEST(A1ServiceFaults, HealthQueryFailureDuringSettlingIsNotSuccess)
{
  FakeClock c;
  FakeTransport t(&c);
  t.sim_info_fail_at_call = 2;  // first in-loop health query fails

  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_HEALTH_QUERY_FAILED");
  EXPECT_EQ(t.write_calls, 1);
  EXPECT_FALSE(r.health_confirmed);
}

TEST(A1ServiceFaults, ModelBecomingInvalidDuringSettlingIsNotSuccess)
{
  FakeClock c;
  FakeTransport t(&c);
  t.model_invalid_at_sim_info_call = 2;

  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_HEALTH_MODEL_INVALID");
  EXPECT_FALSE(r.health_confirmed);
}

TEST(A1ServiceFaults, ModelPathChangingDuringSettlingIsNotSuccess)
{
  FakeClock c;
  FakeTransport t(&c);
  t.model_path_change_at_sim_info_call = 2;

  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_HEALTH_MODEL_PATH_CHANGED");
  EXPECT_FALSE(r.health_confirmed);
}

// With health checking effectively disabled in-loop, the only remaining query is
// the final confirmation taken just before success is reported. Failing it must
// still prevent success.
TEST(A1ServiceFaults, FailingFinalHealthConfirmationIsNotSuccess)
{
  FakeClock c;
  FakeTransport t(&c);
  a1::ApplyPolicy p = basePolicy();
  p.sim_info_check_every = 1000;  // no in-loop health queries
  t.sim_info_fail_at_call = 2;    // 1st = pre-write, 2nd = final confirmation

  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_HEALTH_UNCONFIRMED");
  EXPECT_FALSE(r.health_confirmed);
  EXPECT_EQ(t.write_calls, 1);
}

TEST(A1ServiceFaults, SuccessfulRunConfirmsHealth)
{
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), 0, t, c);
  EXPECT_EQ(r.scene_state, "SCENE_READY_UNVALIDATED");
  EXPECT_TRUE(r.health_confirmed);
  EXPECT_TRUE(r.sim_info_final.ok);
}

// ---------------------------------------------------------------------------
// Defects reproduced by the planner: parameter and policy validation
// ---------------------------------------------------------------------------

TEST(A1ServiceFaults, PolicyRejectsHealthCheckingDisabled)
{
  a1::ApplyPolicy p = basePolicy();
  p.sim_info_check_every = 0;
  const a1::ValidationResult v = a1::validateApplyParameters(p, baseCriteria());
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_POLICY_BAD_RANGE");
}

TEST(A1ServiceFaults, PolicyRejectsMoreThanOneWriteAttempt)
{
  a1::ApplyPolicy p = basePolicy();
  p.max_write_attempts_per_run = 3;
  const a1::ValidationResult v = a1::validateApplyParameters(p, baseCriteria());
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_POLICY_BAD_WRITE_ATTEMPTS");
}

TEST(A1ServiceFaults, PolicyRejectsCallTimeoutLargerThanTheBudget)
{
  a1::ApplyPolicy p = basePolicy();
  p.service_call_timeout_sec = 5.0;  // > stability_timeout_sec
  const a1::ValidationResult v = a1::validateApplyParameters(p, baseCriteria());
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_POLICY_BAD_RANGE");
}

TEST(A1ServiceFaults, PolicyRejectsNonFiniteAndNonPositiveValues)
{
  {
    a1::ApplyPolicy p = basePolicy();
    p.poll_rate_hz = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(a1::validateApplyParameters(p, baseCriteria()).ok);
  }
  {
    a1::ApplyPolicy p = basePolicy();
    p.stability_timeout_sec = 0.0;
    EXPECT_FALSE(a1::validateApplyParameters(p, baseCriteria()).ok);
  }
  {
    a1::ApplyPolicy p = basePolicy();
    a1::StabilityCriteria c = baseCriteria();
    c.min_sim_window_sec = -1.0;
    EXPECT_FALSE(a1::validateApplyParameters(p, c).ok);
  }
  {
    a1::ApplyPolicy p = basePolicy();
    a1::StabilityCriteria c = baseCriteria();
    c.max_window_pos_tol = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(a1::validateApplyParameters(p, c).ok);
  }
  {
    a1::ApplyPolicy p = basePolicy();
    p.max_window_samples = 2;
    EXPECT_FALSE(a1::validateApplyParameters(p, baseCriteria()).ok);
  }
}

TEST(A1ServiceFaults, InvalidPolicyStopsBeforeAnyWrite)
{
  FakeClock c;
  FakeTransport t(&c);
  a1::ApplyPolicy p = basePolicy();
  p.max_write_attempts_per_run = 2;
  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);
  EXPECT_EQ(r.scene_state, "NOT_RUN");
  EXPECT_EQ(r.outcome_code, "E_POLICY_BAD_WRITE_ATTEMPTS");
  EXPECT_EQ(t.write_calls, 0);
}

TEST(A1ServiceFaults, NonUnitCandidateQuaternionIsRejectedWithoutWriting)
{
  a1::Candidate bad;
  ASSERT_TRUE(a1::makeExplicitCandidate(baseConfig(), 0, 0.5, 0.0, 0.0, bad));
  bad.quat = {0.0, 0.0, 0.0, 2.0};  // finite, but not a unit quaternion
  FakeClock c;
  FakeTransport t(&c);
  const a1::RunResult r =
      a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), basePolicy(), bad, t, c);
  EXPECT_EQ(r.outcome_code, "E_BAD_CANDIDATE");
  EXPECT_EQ(t.write_calls, 0);
}

TEST(A1ServiceFaults, RunIdValidation)
{
  EXPECT_TRUE(a1::validateRunId("a1_p1a_pose_yaw000").ok);
  EXPECT_TRUE(a1::validateRunId("a1-2026.10.11_x").ok);

  EXPECT_EQ(a1::validateRunId("").reason_code, "E_RUN_ID_EMPTY");
  EXPECT_EQ(a1::validateRunId(".").reason_code, "E_RUN_ID_DANGEROUS");
  EXPECT_EQ(a1::validateRunId("..").reason_code, "E_RUN_ID_DANGEROUS");
  EXPECT_EQ(a1::validateRunId("../escape").reason_code, "E_RUN_ID_SEPARATOR");
  EXPECT_EQ(a1::validateRunId("a/b").reason_code, "E_RUN_ID_SEPARATOR");
  EXPECT_EQ(a1::validateRunId("a\\b").reason_code, "E_RUN_ID_SEPARATOR");
  EXPECT_EQ(a1::validateRunId("a b").reason_code, "E_RUN_ID_CHARSET");
  EXPECT_EQ(a1::validateRunId("a$b").reason_code, "E_RUN_ID_CHARSET");
  EXPECT_EQ(a1::validateRunId(std::string(200, 'a')).reason_code, "E_RUN_ID_TOO_LONG");
}

// ---------------------------------------------------------------------------
// The final health query is a blocking call too: cancelling during it, or
// returning past the budget, must not yield a success verdict. The already
// applied pose is kept in the record and nothing is retried.
// ---------------------------------------------------------------------------

TEST(A1ServiceFaults, CancelDuringFinalHealthQueryIsNotSuccess)
{
  FakeClock c;
  FakeTransport t(&c);
  std::atomic<bool> abort {false};
  a1::ApplyPolicy p = basePolicy();
  p.abort_flag = &abort;
  p.sim_info_check_every = 1000;  // only the pre-write and final queries happen
  t.on_sim_info = [&abort](int call) {
    if (call >= 2) {           // the final confirmation
      abort.store(true);
    }
  };

  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);

  EXPECT_EQ(r.scene_state, "INTERRUPTED");
  EXPECT_EQ(r.outcome_code, "E_INTERRUPTED");
  EXPECT_FALSE(r.health_confirmed);
  EXPECT_EQ(t.write_calls, 1);  // the applied pose stands, and is not retried
  EXPECT_FALSE(r.samples.empty());
}

TEST(A1ServiceFaults, FinalHealthQueryReturningPastTheBudgetIsNotSuccess)
{
  FakeClock c;
  FakeTransport t(&c);
  a1::ApplyPolicy p = basePolicy();
  p.stability_timeout_sec = 0.30;
  p.service_call_timeout_sec = 0.10;
  p.sim_info_check_every = 1000;  // only the pre-write and final queries happen
  // The final query blocks long enough to overrun the settling budget.
  t.sim_info_latency_from_call = 2;
  t.sim_info_latency_sec = 5.0;

  const a1::RunResult r = a1::applyCandidateAndSettle(baseConfig(), baseCriteria(), p, 0, t, c);

  EXPECT_EQ(r.scene_state, "SCENE_SETTLE_FAILED");
  EXPECT_EQ(r.outcome_code, "E_SCENE_SETTLE_TIMEOUT");
  EXPECT_FALSE(r.health_confirmed);
  EXPECT_EQ(t.write_calls, 1);
}
