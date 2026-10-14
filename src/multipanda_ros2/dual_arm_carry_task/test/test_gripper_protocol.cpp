// A1 P3a-2a offline tests for the gripper action protocol decisions.
//
// These drive the pure classification directly. They are fault-injection /
// protocol tests, NOT measurements of a live ROS action client.

#include "dual_arm_carry_task/gripper_protocol.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

using a1g::ActionCode;
using a1g::LateAcceptance;

// ---------------------------------------------------------------------------
// Result classification
// ---------------------------------------------------------------------------

TEST(GripperProtocol, SucceededIsSuccess)
{
  const auto r = a1g::classifyResult(ActionCode::SUCCEEDED, false, true, 0.08, 0.08);
  EXPECT_TRUE(r.ok) << r.reason_code;
  EXPECT_EQ(r.reason_code, "E_OK");
  EXPECT_FALSE(r.contact_unproven);
}

TEST(GripperProtocol, SucceededWhileClosingWithReachedGoalIsClean)
{
  const auto r = a1g::classifyResult(ActionCode::SUCCEEDED, true, true, 0.04, 0.04);
  EXPECT_TRUE(r.ok);
  EXPECT_FALSE(r.contact_unproven);
}

// A successful close that did not reach the commanded gap stopped early; the
// reason is recorded, but it is not turned into a grasp claim.
TEST(GripperProtocol, SucceededCloseWithoutReachingTheGoalIsFlaggedUnproven)
{
  const auto r = a1g::classifyResult(ActionCode::SUCCEEDED, true, false, 0.05, 0.02);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(r.contact_unproven);
}

TEST(GripperContactWindow, NoContactAndSingleSideNeverConfirm)
{
  a1g::BilateralContactWindow no_contact(3, 2e-4);
  for (unsigned long long seq = 1; seq <= 5; ++seq)
    EXPECT_FALSE(no_contact.update(seq, 0.02, 0.02, false, false));
  EXPECT_EQ(no_contact.count(), 0U);

  a1g::BilateralContactWindow one_side(3, 2e-4);
  for (unsigned long long seq = 1; seq <= 5; ++seq)
    EXPECT_FALSE(one_side.update(seq, 0.02, 0.02, true, false));
  EXPECT_FALSE(one_side.confirmed());
}

TEST(GripperContactWindow, RequiresFreshBilateralStableSamples)
{
  a1g::BilateralContactWindow window(3, 2e-4);
  EXPECT_FALSE(window.update(1, 0.0200, 0.0200, true, true));
  EXPECT_FALSE(window.update(1, 0.0200, 0.0200, true, true));  // old feedback
  EXPECT_FALSE(window.confirmed());
  EXPECT_FALSE(window.update(2, 0.0201, 0.0199, true, true));
  EXPECT_TRUE(window.update(3, 0.0200, 0.0200, true, true));
  EXPECT_TRUE(window.confirmed());
  EXPECT_NEAR(window.gap(), 0.0400, 1e-12);
}

TEST(GripperContactWindow, GapStabilityAloneDoesNotConfirm)
{
  a1g::BilateralContactWindow window(3, 2e-4);
  for (unsigned long long seq = 1; seq <= 5; ++seq)
    EXPECT_FALSE(window.update(seq, 0.02, 0.02, false, false));
  EXPECT_FALSE(window.confirmed());
}

// THE regression this round exists for: an aborted close used to be waved
// through as success. It must never be.
TEST(GripperProtocol, AbortedWhileClosingIsNeverSuccess)
{
  const auto r = a1g::classifyResult(ActionCode::ABORTED, true, false, 0.02, 0.02);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_GRIPPER_ABORTED_CLOSING");
  EXPECT_TRUE(r.contact_unproven);
}

// Even when the measured gap sits exactly on the command, an abort stays a
// failure -- a near-target position must not rewrite an action failure.
TEST(GripperProtocol, AbortIsNotSuccessEvenWhenTheGapLooksOnTarget)
{
  const auto r = a1g::classifyResult(ActionCode::ABORTED, true, true, 0.02, 0.02);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_GRIPPER_ABORTED_CLOSING");

  const auto opening = a1g::classifyResult(ActionCode::ABORTED, false, true, 0.08, 0.08);
  EXPECT_FALSE(opening.ok);
  EXPECT_EQ(opening.reason_code, "E_GRIPPER_ABORTED");
  EXPECT_FALSE(opening.contact_unproven);  // opening makes no contact claim
}

TEST(GripperProtocol, CanceledAndUnknownAreFailures)
{
  const auto canceled = a1g::classifyResult(ActionCode::CANCELED, false, false, 0.01, 0.08);
  EXPECT_FALSE(canceled.ok);
  EXPECT_EQ(canceled.reason_code, "E_GRIPPER_CANCELED");

  const auto unknown = a1g::classifyResult(ActionCode::UNKNOWN, false, false, 0.0, 0.08);
  EXPECT_FALSE(unknown.ok);
  EXPECT_EQ(unknown.reason_code, "E_GRIPPER_UNKNOWN_CODE");
}

// rclcpp_action::ResultCode aliases the GoalStatus values in this distro, so
// SUCCEEDED is 4 -- not 1. Getting this wrong made a real successful open look
// like an unknown code during the first empty-load run.
TEST(GripperProtocol, DecodesTheRealGoalStatusAliasedResultCodes)
{
  EXPECT_EQ(a1g::actionCodeFromInt(4), ActionCode::SUCCEEDED);  // STATUS_SUCCEEDED
  EXPECT_EQ(a1g::actionCodeFromInt(5), ActionCode::CANCELED);   // STATUS_CANCELED
  EXPECT_EQ(a1g::actionCodeFromInt(6), ActionCode::ABORTED);    // STATUS_ABORTED

  // Non-terminal statuses and anything unrecognised are not success.
  EXPECT_EQ(a1g::actionCodeFromInt(0), ActionCode::UNKNOWN);  // STATUS_UNKNOWN
  EXPECT_EQ(a1g::actionCodeFromInt(1), ActionCode::UNKNOWN);  // ACCEPTED
  EXPECT_EQ(a1g::actionCodeFromInt(2), ActionCode::UNKNOWN);  // EXECUTING
  EXPECT_EQ(a1g::actionCodeFromInt(3), ActionCode::UNKNOWN);  // CANCELING
  EXPECT_EQ(a1g::actionCodeFromInt(-1), ActionCode::UNKNOWN);
  EXPECT_EQ(a1g::actionCodeFromInt(99), ActionCode::UNKNOWN);

  // The old wrong mapping must not read a success as unknown.
  EXPECT_NE(a1g::actionCodeFromInt(4), ActionCode::UNKNOWN);
}

// Abnormal feedback must be rejected even on SUCCEEDED: the previous version
// accepted any SUCCEEDED, so a negative gap counted as a completed move.
TEST(GripperProtocol, RejectsAbnormalGapEvenWhenSucceeded)
{
  const auto neg = a1g::classifyResult(ActionCode::SUCCEEDED, false, true, -0.0142, 0.08);
  EXPECT_FALSE(neg.ok);
  EXPECT_EQ(neg.reason_code, "E_GRIPPER_GAP_NEGATIVE");

  const auto nan = a1g::classifyResult(ActionCode::SUCCEEDED, false, true, std::nan(""), 0.08);
  EXPECT_FALSE(nan.ok);
  EXPECT_EQ(nan.reason_code, "E_GRIPPER_GAP_NOT_FINITE");

  const auto inf = a1g::classifyResult(ActionCode::SUCCEEDED, false, true,
                                       std::numeric_limits<double>::infinity(), 0.08);
  EXPECT_FALSE(inf.ok);
  EXPECT_EQ(inf.reason_code, "E_GRIPPER_GAP_NOT_FINITE");

  // Beyond the model maximum is refused, not silently clamped to it.
  const auto big = a1g::classifyResult(ActionCode::SUCCEEDED, false, true, 0.0876, 0.08, 0.08);
  EXPECT_FALSE(big.ok);
  EXPECT_EQ(big.reason_code, "E_GRIPPER_GAP_OUT_OF_RANGE");

  // Boundaries that are physically fine still pass.
  EXPECT_TRUE(a1g::classifyResult(ActionCode::SUCCEEDED, false, true, 0.0, 0.0, 0.08).ok);
  EXPECT_TRUE(a1g::classifyResult(ActionCode::SUCCEEDED, false, true, 0.08, 0.08, 0.08).ok);
}

// ---------------------------------------------------------------------------
// Late goal acceptance
// ---------------------------------------------------------------------------

TEST(GripperProtocolLate, AcceptsAnInBudgetResponse)
{
  EXPECT_EQ(a1g::decideLateAcceptance(true, false, true), LateAcceptance::PROCEED);
  EXPECT_EQ(a1g::decideLateAcceptance(true, false, false), LateAcceptance::REJECTED);
}

// A goal accepted only after the budget expired must not let the call proceed:
// the goal was accepted, so it is cancelled and the call fails.
TEST(GripperProtocolLate, AResponseOnlyInTheGraceWindowIsCancelledNotUsed)
{
  EXPECT_EQ(a1g::decideLateAcceptance(false, true, true), LateAcceptance::CANCEL_LATE);
}

// Nothing at all arrived, so whether the server accepted the goal is unknown;
// the round must stop rather than continue with another attempt.
TEST(GripperProtocolLate, NoResponseAtAllIsAnUnknownState)
{
  EXPECT_EQ(a1g::decideLateAcceptance(false, false, false), LateAcceptance::UNKNOWN_STOP);
  EXPECT_EQ(a1g::decideLateAcceptance(false, true, false), LateAcceptance::UNKNOWN_STOP);
}

// ---------------------------------------------------------------------------
// Budget accounting
// ---------------------------------------------------------------------------

TEST(GripperProtocolBudget, RemainingBudgetNeverGoesNegativeAndHandlesBadInput)
{
  EXPECT_DOUBLE_EQ(a1g::remainingBudget(2.0, 20.0), 18.0);
  EXPECT_DOUBLE_EQ(a1g::remainingBudget(25.0, 20.0), 0.0);
  EXPECT_DOUBLE_EQ(a1g::remainingBudget(std::nan(""), 20.0), 0.0);
  EXPECT_DOUBLE_EQ(a1g::remainingBudget(1.0, std::nan("")), 0.0);
}

TEST(GripperProtocolBudget, StageBudgetClampsToTheRemainingTotalBudget)
{
  EXPECT_TRUE(a1g::stageWithinBudget(1.0, 20.0));
  EXPECT_FALSE(a1g::stageWithinBudget(20.0, 20.0));   // exhausted, not merely large
  EXPECT_FALSE(a1g::stageWithinBudget(21.0, 20.0));
  EXPECT_FALSE(a1g::stageWithinBudget(std::nan(""), 20.0));
  // Two serial 8 s stage budgets do not fit inside a 10 s total. The helper is
  // what stops the second stage being given its full budget and the pair being
  // presented as a single 16 s timeout: with 8 s already spent only 2 s remain,
  // so an 8 s stage must be clamped to 2 s.
  const double total = 10.0;
  const double elapsed_after_first = 8.0;
  EXPECT_DOUBLE_EQ(a1g::remainingBudget(elapsed_after_first, total), 2.0);
  const double requested_stage = 8.0;
  EXPECT_DOUBLE_EQ(std::min(requested_stage, a1g::remainingBudget(elapsed_after_first, total)), 2.0);
  // And the overall call refuses to start another stage once the total is gone.
  EXPECT_FALSE(a1g::stageWithinBudget(total, total));
}
