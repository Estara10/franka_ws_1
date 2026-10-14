// A1 P3a-2a: tests for the gripper feedback handoff and the stable-arrival
// window. These exercise the same header the action server uses, so what is
// verified here is what runs in the simulator.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <franka_hardware/sim/gripper_shared_state.hpp>
#include <franka_hardware/sim/gripper_command_policy.hpp>

namespace {

using franka_gripper::GripperFeedback;
using franka_gripper::GripperSharedState;
using franka_gripper::StabilityWindow;
using franka_gripper::BilateralContactHold;
using franka_gripper::publishFeedback;
using franka_gripper::readFeedback;
using franka_gripper::writeCommand;
using franka_gripper::contactSuccessAllowed;
using franka_gripper::GripperCompletionPolicy;
using franka_gripper::gripperCommandCompletionPolicy;
using franka_gripper::explicitGraspCompletionPolicy;

GripperFeedback makeFeedback(double q1, double q2, unsigned long long seq);
GripperFeedback contactFeedback(double q1, double q2, unsigned long long seq,
                                bool left, bool right);

TEST(GripperCommandPolicy, OpeningAtMaximumGapNeverSelectsContactFromFeedbackDirection) {
  // A measured gap slightly above the 0.08 m target is still a plain position
  // command. The relation between target and feedback does not select Grasp.
  EXPECT_EQ(gripperCommandCompletionPolicy(0.08, 0.0801),
            GripperCompletionPolicy::kPositionTarget);
}

TEST(GripperCommandPolicy, OrdinaryClosingStillUsesFreshPositionArrivalPolicy) {
  constexpr auto policy = gripperCommandCompletionPolicy(0.02, 0.08);
  EXPECT_EQ(policy, GripperCompletionPolicy::kPositionTarget);

  StabilityWindow window(3, 0.002, 2e-4);
  auto first = makeFeedback(0.010, 0.010, 10);
  EXPECT_FALSE(window.update(first, 0.02));
  EXPECT_FALSE(window.update(first, 0.02));  // cached sample cannot satisfy arrival
  EXPECT_FALSE(window.update(makeFeedback(0.010, 0.010, 11), 0.02));
  EXPECT_TRUE(window.update(makeFeedback(0.010, 0.010, 12), 0.02));
}

TEST(GripperCommandPolicy, ExplicitGraspAloneSelectsBilateralContactPolicy) {
  constexpr auto policy = explicitGraspCompletionPolicy();
  EXPECT_EQ(policy, GripperCompletionPolicy::kBilateralContact);

  BilateralContactHold hold(2, 2e-4);
  const auto no_contact = hold.update(contactFeedback(0.01, 0.01, 1, false, false),
                                      0.02, 0.0, 0.001);
  EXPECT_FALSE(no_contact.confirmed);
  const auto first_contact = hold.update(contactFeedback(0.01, 0.01, 2, true, true),
                                         no_contact.command_gap_m, 0.0, 0.001);
  EXPECT_FALSE(first_contact.confirmed);
  const auto second_contact = hold.update(contactFeedback(0.01, 0.01, 3, true, true),
                                          first_contact.command_gap_m, 0.0, 0.001);
  EXPECT_TRUE(second_contact.confirmed);
}

GripperFeedback makeFeedback(double q1, double q2, unsigned long long seq) {
  GripperFeedback fb;
  fb.command_per_finger = 0.0;
  fb.finger_q1 = q1;
  fb.finger_q2 = q2;
  fb.feedback_seq = seq;
  fb.valid = seq > 0ULL && std::isfinite(q1) && std::isfinite(q2);
  return fb;
}

TEST(GripperSharedState, TotalGapSumsBothFingers) {
  // Asymmetric fingers: the total is q1 + q2, never one finger doubled.
  const GripperFeedback fb = makeFeedback(0.030, 0.010, 7);
  EXPECT_DOUBLE_EQ(fb.totalGap(), 0.040);
  EXPECT_NE(fb.totalGap(), 0.060);  // 2 * q1 would have given this
}

TEST(GripperSharedState, UnpublishedFeedbackIsInvalid) {
  auto state = std::make_shared<GripperSharedState>();
  const GripperFeedback fb = readFeedback(state);
  EXPECT_FALSE(fb.valid);
  EXPECT_EQ(fb.feedback_seq, 0ULL);
}

TEST(GripperSharedState, PublishBumpsSequenceAndSumsFingers) {
  auto state = std::make_shared<GripperSharedState>();
  publishFeedback(state, 0.02, 0.02);
  const GripperFeedback first = readFeedback(state);
  EXPECT_TRUE(first.valid);
  EXPECT_EQ(first.feedback_seq, 1ULL);
  EXPECT_DOUBLE_EQ(first.totalGap(), 0.04);

  publishFeedback(state, 0.03, 0.03);
  const GripperFeedback second = readFeedback(state);
  EXPECT_EQ(second.feedback_seq, 2ULL);
  EXPECT_DOUBLE_EQ(second.totalGap(), 0.06);
}

TEST(GripperSharedState, ContactIdentityAndSequenceShareOneSnapshot) {
  auto state = std::make_shared<GripperSharedState>();
  publishFeedback(state, 0.018, 0.017, true, false);
  const GripperFeedback one_side = readFeedback(state);
  EXPECT_EQ(one_side.feedback_seq, 1ULL);
  EXPECT_TRUE(one_side.left_finger_rod_contact);
  EXPECT_FALSE(one_side.right_finger_rod_contact);

  publishFeedback(state, 0.018, 0.017, true, true);
  const GripperFeedback bilateral = readFeedback(state);
  EXPECT_EQ(bilateral.feedback_seq, 2ULL);
  EXPECT_TRUE(bilateral.left_finger_rod_contact);
  EXPECT_TRUE(bilateral.right_finger_rod_contact);
}

TEST(GripperSharedState, NonFiniteCommandIsRejectedNotZeroed) {
  auto state = std::make_shared<GripperSharedState>();
  ASSERT_TRUE(writeCommand(state, 0.02));
  // NaN must not become 0.0, which in this unit contract is a full close.
  EXPECT_FALSE(writeCommand(state, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(writeCommand(state, std::numeric_limits<double>::infinity()));
  EXPECT_DOUBLE_EQ(readFeedback(state).command_per_finger, 0.02);
}

TEST(GripperSharedState, NonFiniteFingerFeedbackIsInvalid) {
  auto state = std::make_shared<GripperSharedState>();
  publishFeedback(state, 0.02, 0.02);
  ASSERT_TRUE(readFeedback(state).valid);
  publishFeedback(state, std::numeric_limits<double>::quiet_NaN(), 0.02);
  EXPECT_FALSE(readFeedback(state).valid);
}

// --- StabilityWindow -------------------------------------------------------

constexpr int kSamples = 10;
constexpr double kTol = 0.002;
constexpr double kDelta = 2e-4;

TEST(StabilityWindow, RepeatedReadsOfOneSampleNeverSatisfyIt) {
  // The defect this replaces: a counter that added one per loop iteration, so
  // re-reading the same physics value ten times looked like ten fresh samples.
  StabilityWindow window(kSamples, kTol, kDelta);
  const GripperFeedback fb = makeFeedback(0.04, 0.04, 42);
  for (int i = 0; i < 50; ++i) {
    EXPECT_FALSE(window.update(fb, 0.08)) << "repeated read at iteration " << i;
  }
  EXPECT_EQ(window.count(), 1);
}

TEST(StabilityWindow, FreshSamplesReachTheWindow) {
  StabilityWindow window(kSamples, kTol, kDelta);
  bool satisfied = false;
  for (int i = 0; i < kSamples; ++i) {
    satisfied = window.update(makeFeedback(0.04, 0.04, static_cast<unsigned long long>(i + 1)),
                              0.08);
  }
  EXPECT_TRUE(satisfied);
  EXPECT_EQ(window.count(), kSamples);
}

TEST(StabilityWindow, SampleOutsideToleranceEmptiesTheWindow) {
  StabilityWindow window(kSamples, kTol, kDelta);
  for (int i = 0; i < kSamples - 1; ++i) {
    window.update(makeFeedback(0.04, 0.04, static_cast<unsigned long long>(i + 1)), 0.08);
  }
  EXPECT_EQ(window.count(), kSamples - 1);
  // One sample 5 mm off target, then the count has to start again.
  EXPECT_FALSE(window.update(makeFeedback(0.0375, 0.0375, 100ULL), 0.08));
  EXPECT_EQ(window.count(), 0);
  EXPECT_FALSE(window.update(makeFeedback(0.04, 0.04, 101ULL), 0.08));
  EXPECT_EQ(window.count(), 1);
}

TEST(StabilityWindow, SlowDriftFailsEvenThoughEachStepIsTiny) {
  // Each consecutive step is 1e-4, i.e. half the adjacent-step threshold the old
  // check used, but the window walks 1e-3 in total -- outside kDelta.
  StabilityWindow window(kSamples, kTol, kDelta);
  bool ever_satisfied = false;
  for (int i = 0; i < kSamples; ++i) {
    const double gap = 0.0800 + 0.0001 * i;
    if (window.update(makeFeedback(gap / 2.0, gap / 2.0, static_cast<unsigned long long>(i + 1)),
                      0.08)) {
      ever_satisfied = true;
    }
  }
  EXPECT_FALSE(ever_satisfied);
  EXPECT_GT(window.spread(), kDelta);
}

TEST(StabilityWindow, SmallRealNoiseInsideTheSpreadIsAccepted) {
  StabilityWindow window(kSamples, kTol, kDelta);
  const double jitter[3] = {0.0, 5e-5, -5e-5};
  bool satisfied = false;
  for (int i = 0; i < kSamples; ++i) {
    const double gap = 0.08 + jitter[i % 3];
    satisfied = window.update(makeFeedback(gap / 2.0, gap / 2.0,
                                           static_cast<unsigned long long>(i + 1)),
                              0.08);
  }
  EXPECT_TRUE(satisfied);
  EXPECT_LE(window.spread(), kDelta);
}

TEST(StabilityWindow, SnapshotIsTheSatisfyingSampleNotTheTarget) {
  StabilityWindow window(kSamples, kTol, kDelta);
  GripperFeedback last;
  for (int i = 0; i < kSamples; ++i) {
    // Deliberately asymmetric fingers so the snapshot proves q1 + q2.
    last = makeFeedback(0.0400, 0.0399, static_cast<unsigned long long>(i + 1));
    window.update(last, 0.08);
  }
  EXPECT_DOUBLE_EQ(window.snapshot().totalGap(), last.totalGap());
  EXPECT_EQ(window.snapshot().feedback_seq, last.feedback_seq);
  EXPECT_DOUBLE_EQ(window.snapshot().finger_q1, 0.0400);
  EXPECT_DOUBLE_EQ(window.snapshot().finger_q2, 0.0399);
}

TEST(StabilityWindow, AsymmetricFingersStillFormTheGapAndSnapshot) {
  // q1 and q2 differ by more than the window spread. If either the gap were
  // taken from one finger doubled, or the snapshot were rebuilt from a single
  // number, this would not settle.
  StabilityWindow window(kSamples, kTol, kDelta);
  bool satisfied = false;
  for (int i = 0; i < kSamples; ++i) {
    satisfied = window.update(makeFeedback(0.045, 0.035, static_cast<unsigned long long>(i + 1)),
                              0.08);
  }
  EXPECT_TRUE(satisfied);
  EXPECT_DOUBLE_EQ(window.snapshot().totalGap(), 0.08);
}

// --- Explicit bilateral-contact hold (the Grasp server uses this class) ---

GripperFeedback contactFeedback(double q1, double q2, unsigned long long seq,
                                bool left, bool right) {
  auto fb = makeFeedback(q1, q2, seq);
  fb.left_finger_rod_contact = left;
  fb.right_finger_rod_contact = right;
  return fb;
}

TEST(BilateralContactHold, BilateralContactLatchesOneHalfMillimeterPreload) {
  BilateralContactHold hold(4, 2e-4);
  auto approaching = hold.update(makeFeedback(0.024, 0.024, 1), 0.050, 0.0, 0.001);
  ASSERT_EQ(approaching.stage, BilateralContactHold::Stage::APPROACH);
  EXPECT_DOUBLE_EQ(approaching.command_gap_m, 0.049);

  const auto contact = hold.update(contactFeedback(0.021, 0.020, 2, true, true),
                                   approaching.command_gap_m, 0.0, 0.001);
  ASSERT_TRUE(contact.entered_hold);
  EXPECT_EQ(contact.stage, BilateralContactHold::Stage::HOLD);
  EXPECT_DOUBLE_EQ(hold.latchedContactGap(), 0.041);
  EXPECT_DOUBLE_EQ(contact.hold_target_gap_m, 0.0405);
  EXPECT_DOUBLE_EQ(contact.command_gap_m, 0.048);
  EXPECT_LE(std::abs(contact.command_gap_m - approaching.command_gap_m), 0.001 + 1e-12);

  const auto held = hold.update(contactFeedback(0.0205, 0.0205, 3, true, true),
                                contact.command_gap_m, 0.0, 0.001);
  EXPECT_DOUBLE_EQ(held.hold_target_gap_m, 0.0405);
  EXPECT_DOUBLE_EQ(held.command_gap_m, 0.047);
  EXPECT_LE(std::abs(held.command_gap_m - contact.command_gap_m), 0.001 + 1e-12);
  EXPECT_EQ(held.stage, BilateralContactHold::Stage::HOLD);
}

TEST(BilateralContactHold, LaterContactCannotRelatchOrAddPreload) {
  BilateralContactHold hold(4, 2e-4);
  auto first = hold.update(contactFeedback(0.021, 0.020, 1, true, true), 0.043, 0.0, 0.001);
  ASSERT_TRUE(first.entered_hold);
  const double target = first.hold_target_gap_m;
  const auto later = hold.update(contactFeedback(0.024, 0.023, 2, true, true),
                                 first.command_gap_m, 0.0, 0.001);
  EXPECT_FALSE(later.entered_hold);
  EXPECT_DOUBLE_EQ(later.hold_target_gap_m, target);
  EXPECT_DOUBLE_EQ(hold.latchedContactGap(), 0.041);
}

TEST(BilateralContactHold, SingleFingerContactDoesNotEnterHoldOrConfirm) {
  BilateralContactHold hold(3, 2e-4);
  auto one = hold.update(contactFeedback(0.02, 0.02, 1, true, false), 0.05, 0.0, 0.001);
  EXPECT_FALSE(one.entered_hold);
  EXPECT_FALSE(one.confirmed);
  EXPECT_EQ(one.stage, BilateralContactHold::Stage::APPROACH);
  EXPECT_LT(one.command_gap_m, 0.05);
}

TEST(BilateralContactHold, ContactLossResetsWindowWithoutFurtherInwardMotion) {
  BilateralContactHold hold(4, 2e-4);
  auto first = hold.update(contactFeedback(0.021, 0.020, 1, true, true), 0.043, 0.0, 0.001);
  ASSERT_TRUE(first.entered_hold);
  auto second = hold.update(contactFeedback(0.021, 0.020, 2, true, true),
                            first.command_gap_m, 0.0, 0.001);
  ASSERT_EQ(second.window_samples, 2);
  auto lost = hold.update(contactFeedback(0.021, 0.020, 3, true, false),
                          second.command_gap_m, 0.0, 0.001);
  EXPECT_TRUE(lost.contact_lost);
  EXPECT_EQ(lost.window_samples, 0);
  EXPECT_FALSE(lost.confirmed);
  EXPECT_DOUBLE_EQ(lost.hold_target_gap_m, first.hold_target_gap_m);
  EXPECT_DOUBLE_EQ(lost.command_gap_m, second.command_gap_m);
  EXPECT_TRUE(lost.fresh_sample);
}

TEST(BilateralContactHold, OldFeedbackCannotFillWindowOrConfirm) {
  BilateralContactHold hold(3, 2e-4);
  auto first = hold.update(contactFeedback(0.02, 0.02, 10, true, true), 0.04, 0.0, 0.001);
  ASSERT_TRUE(first.entered_hold);
  EXPECT_EQ(first.window_samples, 1);
  for (int i = 0; i < 10; ++i) {
    const auto old = hold.update(contactFeedback(0.02, 0.02, 10, true, true),
                                 first.command_gap_m, 0.0, 0.001);
    EXPECT_FALSE(old.fresh_sample);
    EXPECT_FALSE(old.confirmed);
  }
  EXPECT_EQ(hold.windowSamples(), 1);
}

TEST(BilateralContactHold, StableFreshBilateralContactConfirms) {
  BilateralContactHold hold(4, 2e-4);
  auto update = hold.update(contactFeedback(0.020, 0.020, 1, true, true), 0.04, 0.0, 0.001);
  ASSERT_TRUE(update.entered_hold);
  for (unsigned long long seq = 2; seq <= 4; ++seq) {
    update = hold.update(contactFeedback(0.020 + (seq % 2) * 1e-5,
                                         0.020 - (seq % 2) * 1e-5,
                                         seq, true, true),
                         update.command_gap_m, 0.0, 0.001);
  }
  EXPECT_TRUE(update.confirmed);
  EXPECT_LE(update.window_spread_m, 2e-4);
}

TEST(BilateralContactHold, ContactWindowCannotOverrideCancelOrDeadline) {
  EXPECT_TRUE(contactSuccessAllowed(true, false, false));
  EXPECT_FALSE(contactSuccessAllowed(true, true, false));
  EXPECT_FALSE(contactSuccessAllowed(true, false, true));
  EXPECT_FALSE(contactSuccessAllowed(false, false, false));
}

TEST(StabilityWindow, InvalidFeedbackResetsTheWindow) {
  StabilityWindow window(kSamples, kTol, kDelta);
  for (int i = 0; i < kSamples - 1; ++i) {
    window.update(makeFeedback(0.04, 0.04, static_cast<unsigned long long>(i + 1)), 0.08);
  }
  EXPECT_FALSE(window.update(makeFeedback(std::numeric_limits<double>::quiet_NaN(), 0.04, 50ULL),
                             0.08));
  EXPECT_EQ(window.count(), 0);
}

TEST(StabilityWindow, SlidingWindowForgetsTheApproachTransient) {
  // Regression from a real run: the gripper held a dead-flat 0.08000 m for its
  // last 200 cycles, but a window that accumulated min/max from the start of
  // the move kept the approach transient in view (spread 1.4 mm) and never
  // reported arrival. Only the LAST kSamples may count.
  StabilityWindow window(kSamples, kTol, kDelta);
  unsigned long long seq = 0;
  // Approach: swings 3 mm around the target before settling (inside kTol).
  const double approach[4] = {0.0785, 0.0810, 0.0790, 0.0815};
  for (int i = 0; i < 40; ++i) {
    const double gap = approach[i % 4];
    EXPECT_FALSE(window.update(makeFeedback(gap / 2.0, gap / 2.0, ++seq), 0.08))
        << "reported arrival during the approach at sample " << i;
  }
  // Settled: 10 dead-flat samples must now satisfy the window.
  bool satisfied = false;
  for (int i = 0; i < kSamples; ++i) {
    satisfied = window.update(makeFeedback(0.04, 0.04, ++seq), 0.08);
  }
  EXPECT_TRUE(satisfied);
  EXPECT_DOUBLE_EQ(window.spread(), 0.0);
}

TEST(StabilityWindow, WindowKeepsSlidingAndStillCatchesLaterDrift) {
  StabilityWindow window(kSamples, kTol, kDelta);
  unsigned long long seq = 0;
  for (int i = 0; i < kSamples - 1; ++i) {
    EXPECT_FALSE(window.update(makeFeedback(0.04, 0.04, ++seq), 0.08));
  }
  ASSERT_TRUE(window.update(makeFeedback(0.04, 0.04, ++seq), 0.08));
  // Then the gripper creeps 1 mm off target over the next window. However flat
  // each individual step looks, the window has to stop reporting arrival.
  bool satisfied_again = false;
  for (int i = 1; i <= kSamples; ++i) {
    const double gap = 0.08 + 0.0001 * i;
    satisfied_again = window.update(makeFeedback(gap / 2.0, gap / 2.0, ++seq), 0.08);
  }
  EXPECT_FALSE(satisfied_again);
  EXPECT_GT(window.spread(), kDelta);
}

}  // namespace
