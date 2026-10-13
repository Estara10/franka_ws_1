#pragma once
#include <atomic>
#include <mutex>

#include "carry_task_context.hpp"

// A1 P3a-2a: gripper action client.
//
// The left/right action clients are created ONCE and reused; they are not
// rebuilt on every open/close. Every wait is bounded, the whole call is bounded
// by a single total budget, and an accepted goal is cancelled when the call
// fails. ABORTED is never success -- see a1g::classifyResult.
class GripperController {
    DualArmCarryTask* ctx_;
    using GripperAction = control_msgs::action::GripperCommand;
    using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperAction>;

public:
    explicit GripperController(DualArmCarryTask* ctx) : ctx_(ctx) {}
    bool controlDualGrippers(double position);
    bool controlGripper(const std::string& arm_name, double position);

private:
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

    rclcpp_action::Client<GripperAction>::SharedPtr left_client_;
    rclcpp_action::Client<GripperAction>::SharedPtr right_client_;
    std::mutex client_mutex_;   // guards lazy creation
    std::mutex call_mutex_;     // only one open/close may be in flight
    std::atomic<bool> in_flight_ {false};
};
