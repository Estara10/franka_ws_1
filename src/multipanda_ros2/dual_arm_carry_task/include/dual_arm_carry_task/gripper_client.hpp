#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <std_srvs/srv/trigger.hpp>

// A1 P3a-2a: gripper action client.
//
// The left/right action clients are created ONCE and reused; they are not
// rebuilt on every open/close. Every wait is bounded, the whole call is bounded
// by a single total budget, and an accepted goal is cancelled when the call
// fails. ABORTED is never success -- see a1g::classifyResult.
//
// This header deliberately does NOT include the task context: the client only
// needs a node and the bounds below, which is what lets the integration test
// drive the production client against a fake action server without building a
// whole task node.

/// The bounds and thresholds the client runs with. The task fills this from its
/// parameters; the test fills it directly.
struct GripperClientConfig {
    double server_timeout_sec {2.0};   // wait for the action server to appear
    double goal_timeout_sec {2.0};     // wait for the goal to be accepted
    double result_timeout_sec {8.0};   // wait for the result
    double total_timeout_sec {16.0};   // bound on the whole dual call
    double max_effort {10.0};
    double close_threshold {0.03};     // position below this counts as "closing"
    // The server names the simulator's hardware plugin publishes. Overridable so
    // the interface test can point the production client at a fake server.
    std::string left_action {"/mj_left_gripper_sim_node/gripper_action"};
    std::string right_action {"/mj_right_gripper_sim_node/gripper_action"};
};

/// Per-goal-call state shared between the calling thread and the executor
/// callbacks. Defined in gripper_client.cpp; declared here only so the
/// controller can hold the registry by shared_ptr.
struct GripperGoalContext;
/// Tracks goals whose outcome is still unknown. Defined in gripper_client.cpp.
struct GripperPendingRegistry;

/// action_msgs/srv/CancelGoal return codes. Redeclared here so the classifier
/// below needs no message headers and can be tested on its own.
enum CancelReturnCode {
    kCancelErrorNone = 0,             // request accepted, goal(s) moved to CANCELING
    kCancelErrorRejected = 1,         // request rejected outright
    kCancelErrorUnknownGoalId = 2,    // the server does not know that goal any more
    kCancelErrorGoalTerminated = 3,   // the goal is already in a terminal state
};

/// What a CancelGoal response actually proves about ONE specific goal.
enum class CancelVerdict {
    /// The server accepted the request AND named our goal as canceling. It has
    /// taken the goal over; this is not a claim that the gripper has stopped.
    ACCEPTED,
    /// The server says our goal is gone or already terminal. This is positive
    /// evidence of a terminal state.
    GOAL_TERMINAL,
    /// Nothing was proven: a refusal, an answer that does not mention our goal,
    /// an unrecognised return code, or no answer at all.
    UNCONFIRMED,
};

/// Interprets a CancelGoal response for one specific goal.
///
/// The return code alone is not enough. `ERROR_NONE` says "one or more goals
/// moved to CANCELING" -- it does not say WHICH, so an answer that never names
/// our goal proves nothing about it and must come back UNCONFIRMED. Only
/// `ERROR_GOAL_TERMINATED` and `ERROR_UNKNOWN_GOAL_ID` are terminal evidence,
/// because both are stated per requested goal.
inline CancelVerdict classifyCancelResponse(
    int return_code,
    const std::array<uint8_t, 16>& our_goal_id,
    const std::vector<std::array<uint8_t, 16>>& goals_canceling)
{
    if (return_code == kCancelErrorGoalTerminated ||
        return_code == kCancelErrorUnknownGoalId) {
        return CancelVerdict::GOAL_TERMINAL;
    }
    if (return_code != kCancelErrorNone) {
        // Includes ERROR_REJECTED and anything unrecognised.
        return CancelVerdict::UNCONFIRMED;
    }
    for (const auto& id : goals_canceling) {
        if (id == our_goal_id) {
            return CancelVerdict::ACCEPTED;
        }
    }
    // ERROR_NONE but our goal is not among the ones that moved to CANCELING.
    return CancelVerdict::UNCONFIRMED;
}

class GripperController {
    rclcpp::Node* node_;
    GripperClientConfig cfg_;

public:
    using GripperAction = control_msgs::action::GripperCommand;

    GripperController(rclcpp::Node* node, const GripperClientConfig& config);

    /// Drives both grippers to `position` (PER-FINGER displacement, metres).
    /// Returns true only when both arms report E_OK.
    bool controlDualGrippers(double position);
    /// Stops both grippers at their measured positions; used to leave the
    /// contact-only actuator mode before a failed load probe shuts down.
    bool stopDualGrippers();
    /// Explicit object-contact path. Uses the simulator's Grasp action contract;
    /// each action succeeds only after bilateral finger-to-rod feedback.
    bool controlDualGrippersContact(double force_per_finger_n = 0.0);

    /// Drives one gripper. Kept for the single-arm entry points.
    bool controlGripper(const std::string& arm_name, double position);

private:
    using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperAction>;

    struct ArmOutcome {
        std::string arm;
        bool server_ready {false};
        bool goal_accepted {false};
        bool goal_rejected {false};
        bool result_received {false};
        int result_code {3};  // UNKNOWN
        bool reached_goal {false};
        bool stalled {false};
        double achieved_gap_m {0.0};
        double effort {0.0};
        bool cancel_requested {false};
        bool cancel_confirmed {false};
        bool late_goal_unknown {false};
        bool not_attempted {false};
        bool goal_already_finished {false};
        std::string reason_code;
        std::string detail;
    };

    // Lazily create and then keep both clients.
    rclcpp_action::Client<GripperAction>::SharedPtr clientFor(bool left);
    // Drives one arm through send -> accept -> result with every stage bounded.
    ArmOutcome runArm(bool left,
                      double position,
                      bool closing,
                      double total_budget_sec,
                      std::chrono::steady_clock::time_point begin);
    void requestCancel(const GripperGoalHandle::SharedPtr& handle, ArmOutcome& outcome);
    bool logOutcomes(const char* what, const ArmOutcome& left, const ArmOutcome& right) const;

    /// Refuses a new call while any earlier goal's outcome is still unknown.
    /// Nothing is sent when it refuses.
    bool refuseIfUnknownOutstanding(const char* what);

    rclcpp_action::Client<GripperAction>::SharedPtr left_client_;
    rclcpp_action::Client<GripperAction>::SharedPtr right_client_;
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr left_contact_client_;
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr right_contact_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_stop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_stop_client_;
    std::mutex client_mutex_;   // guards lazy creation
    std::mutex call_mutex_;     // only one open/close may be in flight
    std::atomic<bool> in_flight_ {false};
    std::atomic<bool> contact_goal_unknown_ {false};
    // Goals whose response, or whose cancel answer, is still outstanding.
    std::shared_ptr<GripperPendingRegistry> pending_;
};
