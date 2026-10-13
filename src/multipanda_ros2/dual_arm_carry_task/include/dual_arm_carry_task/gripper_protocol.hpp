// A1 P3a-2a: gripper action protocol semantics, kept free of ROS so the
// decisions can be driven directly by tests.
//
// Units, as implemented by this workspace's server
// (franka_hardware/src/sim/gripper_sim_action_server.cpp):
//   * command.position is the PER-FINGER displacement: the server computes the
//     target gap as `2 * command.position`.
//   * result.position is the TOTAL gap in metres (control_msgs/GripperCommand
//     documents it as "the current gripper gap size").
//   * result.stalled is never set by this server, so it carries no information.
// The two are NOT interchangeable, and neither is the rod width.

#pragma once

#include <string>

namespace a1g {

// rclcpp_action::ResultCode values, kept as a plain enum so this module needs
// no ROS headers.
// These mirror rclcpp_action::ResultCode, which aliases the action_msgs
// GoalStatus values in this ROS distribution. They are NOT 0/1/2/3.
enum class ActionCode { SUCCEEDED = 4, CANCELED = 5, ABORTED = 6, UNKNOWN = 0 };

// Maps a raw action result code. Unrecognised and non-terminal statuses map to
// UNKNOWN, which is treated as a failure.
ActionCode actionCodeFromInt(int code);

struct ResultClassification {
  bool ok {false};
  // The gripper stopped without the commanded gap being reached and without
  // contact being confirmed. A closing move that ends this way is NOT a grasp.
  bool contact_unproven {false};
  std::string reason_code;
  std::string detail;
};

// Interprets one gripper action result.
//
// ABORTED is never success -- not even while closing, and not even when the
// measured gap happens to sit near the target. An aborted close means the
// gripper stopped early; whether that was object contact is unproven here and
// belongs to the next phase, where it can be checked against the object.
// `max_gap_m` is the model's maximum opening. A SUCCEEDED result whose reported
// gap is non-finite, negative or beyond that maximum is rejected rather than
// accepted: an abnormal value must not be dressed up as success, and the value
// is never silently truncated.
ResultClassification classifyResult(ActionCode code,
                                    bool closing,
                                    bool reached_goal,
                                    double achieved_gap_m,
                                    double commanded_gap_m,
                                    double max_gap_m = 0.08);

// What to do when the goal-acceptance response arrives late.
enum class LateAcceptance { PROCEED, REJECTED, CANCEL_LATE, UNKNOWN_STOP };

// `ready_within_budget` is whether the goal response arrived inside the stage
// budget; `ready_within_grace` whether it arrived during the extra grace window
// afterwards; `handle_valid` whether an accepted goal handle is present.
//
// A response that only arrives during the grace window must NOT let this call
// proceed: the goal was accepted, so it is cancelled and the call fails. If it
// never arrives at all the state is unknown and the caller must stop the round
// rather than continue with another attempt.
LateAcceptance decideLateAcceptance(bool ready_within_budget,
                                    bool ready_within_grace,
                                    bool handle_valid);

// Whether a stage bounded by `budget_sec` still has time left, given how much of
// the whole call has already been consumed. Keeps the per-stage bounds from
// silently extending the overall deadline.
bool stageWithinBudget(double elapsed_total_sec, double budget_sec);

// Total-time helper: how long a stage may still wait so that the overall call
// stays inside `total_budget_sec`. Never negative.
double remainingBudget(double elapsed_total_sec, double total_budget_sec);

}  // namespace a1g
