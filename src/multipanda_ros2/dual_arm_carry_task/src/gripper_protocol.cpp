// A1 P3a-2a: gripper action protocol semantics. See the header.

#include "dual_arm_carry_task/gripper_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace a1g {
namespace {

std::string gapText(double achieved_m, double commanded_m)
{
  return "gap " + std::to_string(achieved_m) + " m vs commanded " + std::to_string(commanded_m) +
         " m";
}

}  // namespace

ActionCode actionCodeFromInt(int code)
{
  // rclcpp_action::ResultCode aliases the action_msgs GoalStatus values in this
  // distribution: UNKNOWN=0, ACCEPTED=1, EXECUTING=2, CANCELING=3,
  // SUCCEEDED=4, CANCELED=5, ABORTED=6. Mapping these as 0/1/2/3 would classify
  // a real success as UNKNOWN, so the correct values are used here and the
  // non-terminal statuses deliberately fall through to UNKNOWN.
  switch (code) {
    case 4: return ActionCode::SUCCEEDED;
    case 5: return ActionCode::CANCELED;
    case 6: return ActionCode::ABORTED;
    default: return ActionCode::UNKNOWN;
  }
}

ResultClassification classifyResult(ActionCode code,
                                    bool closing,
                                    bool reached_goal,
                                    double achieved_gap_m,
                                    double commanded_gap_m,
                                    double max_gap_m,
                                    double soft_limit_tol_m)
{
  ResultClassification r;

  // Feedback validity is checked for EVERY code, including SUCCEEDED: the
  // previous version accepted any SUCCEEDED, so a negative or out-of-range gap
  // was reported as a completed move.
  if (!std::isfinite(achieved_gap_m)) {
    r.ok = false;
    r.reason_code = "E_GRIPPER_GAP_NOT_FINITE";
    r.detail = "reported gap is not finite";
    return r;
  }
  if (achieved_gap_m < 0.0) {
    r.ok = false;
    r.reason_code = "E_GRIPPER_GAP_NEGATIVE";
    r.detail = "reported gap " + std::to_string(achieved_gap_m) +
               " m is negative; a physical opening cannot be negative";
    return r;
  }
  const double allowed = soft_limit_tol_m > 0.0 ? soft_limit_tol_m : 0.0;
  if (std::isfinite(max_gap_m) && max_gap_m > 0.0 && achieved_gap_m > max_gap_m + allowed) {
    r.ok = false;
    r.reason_code = "E_GRIPPER_GAP_OUT_OF_RANGE";
    r.detail = "reported gap " + std::to_string(achieved_gap_m) + " m exceeds the model maximum " +
               std::to_string(max_gap_m) + " m (soft-limit tolerance " +
               std::to_string(allowed) + " m)";
    return r;
  }
  switch (code) {
    case ActionCode::SUCCEEDED:
      r.ok = true;
      r.reason_code = "E_OK";
      r.detail = "succeeded; " + gapText(achieved_gap_m, commanded_gap_m) +
                 (reached_goal ? "; reached_goal=true" : "; reached_goal=false");
      // A close that reports success without reaching the commanded gap is
      // reported as unproven contact, but it is still not turned into a failure
      // here: the server's open-loop behaviour and anything contact-related
      // belong to the object-validation phase.
      if (closing && !reached_goal) {
        r.contact_unproven = true;
      }
      return r;
    case ActionCode::ABORTED:
      r.ok = false;
      // Aborted while closing means the gripper stopped before the commanded
      // gap was reached. That is exactly the case the old rule waved through;
      // it is a failure here regardless of how close the measured gap looks.
      r.contact_unproven = closing;
      r.reason_code = closing ? "E_GRIPPER_ABORTED_CLOSING" : "E_GRIPPER_ABORTED";
      r.detail = std::string("aborted") + (closing ? " while closing" : "") + "; " +
                 gapText(achieved_gap_m, commanded_gap_m) +
                 (closing ? " (contact NOT confirmed; not treated as success)" : "");
      return r;
    case ActionCode::CANCELED:
      r.ok = false;
      r.reason_code = "E_GRIPPER_CANCELED";
      r.detail = "canceled; " + gapText(achieved_gap_m, commanded_gap_m);
      return r;
    case ActionCode::UNKNOWN:
      break;
  }
  r.ok = false;
  r.reason_code = "E_GRIPPER_UNKNOWN_CODE";
  r.detail = "unrecognised action result code";
  return r;
}

LateAcceptance decideLateAcceptance(bool ready_within_budget,
                                    bool ready_within_grace,
                                    bool handle_valid)
{
  if (ready_within_budget) {
    return handle_valid ? LateAcceptance::PROCEED : LateAcceptance::REJECTED;
  }
  if (ready_within_grace && handle_valid) {
    // Accepted, but only after the stage budget expired. The call must not
    // proceed; the goal is cancelled and the call fails.
    return LateAcceptance::CANCEL_LATE;
  }
  // Nothing arrived, so whether the server accepted the goal is unknown.
  return LateAcceptance::UNKNOWN_STOP;
}

bool stageWithinBudget(double elapsed_total_sec, double budget_sec)
{
  if (!std::isfinite(elapsed_total_sec) || !std::isfinite(budget_sec)) {
    return false;
  }
  return elapsed_total_sec < budget_sec;
}

double remainingBudget(double elapsed_total_sec, double total_budget_sec)
{
  if (!std::isfinite(elapsed_total_sec) || !std::isfinite(total_budget_sec)) {
    return 0.0;
  }
  return std::max(0.0, total_budget_sec - elapsed_total_sec);
}

BilateralContactWindow::BilateralContactWindow(std::size_t required_samples,
                                               double max_gap_spread_m)
    : required_(std::max<std::size_t>(1, required_samples)),
      max_gap_spread_m_(std::max(0.0, max_gap_spread_m))
{
}

bool BilateralContactWindow::update(unsigned long long seq, double q1_m, double q2_m,
                                    bool left_contact, bool right_contact)
{
  if (seq == 0 || seq <= last_seq_) return false;
  last_seq_ = seq;
  if (!std::isfinite(q1_m) || !std::isfinite(q2_m) || q1_m < 0.0 || q2_m < 0.0 ||
      !left_contact || !right_contact) {
    gaps_.clear();
    return false;
  }
  gaps_.push_back(q1_m + q2_m);
  if (gaps_.size() > required_) gaps_.erase(gaps_.begin());
  return confirmed();
}

bool BilateralContactWindow::confirmed() const
{
  return gaps_.size() == required_ && spread() <= max_gap_spread_m_;
}

double BilateralContactWindow::spread() const
{
  if (gaps_.empty()) return 0.0;
  const auto bounds = std::minmax_element(gaps_.begin(), gaps_.end());
  return *bounds.second - *bounds.first;
}

}  // namespace a1g
