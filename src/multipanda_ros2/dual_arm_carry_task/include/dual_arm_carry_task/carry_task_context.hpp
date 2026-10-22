#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <dual_arm_a1_msgs/msg/rod_pose_truth.hpp>
#include <dual_arm_a1_msgs/msg/gripper_contact_feedback.hpp>
#include "dual_arm_carry_task/a1_truth.hpp"
#include "dual_arm_carry_task/a1_sample_validator.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <set>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <atomic>
#include <mutex>

enum class TaskState {
    INIT,
    APPROACH,
    GRASP,
    WAITING_PHYSICS, // 等待物理引擎稳定
    LIFT,
    TRANSPORT,
    ROTATE,      // 旋转物体方向（Y轴→X轴）
    DESCEND,     // 下降准备放置
    PLACE,       // 放置物体（GRASP逆操作）
    RETREAT,     // 撤退至初始位置
    PREPARED,    // A1 P1b: 真值已冻结且规划场景已同步确认；仅准备完成
    PREGRASP_REACHED,  // A1 P3a-1: 已执行本轮校验过的预抓取轨迹并实测到位；不是搬运 DONE
    GRASP_POSE_REACHED, // A1 P3a-2b: 已下降到抓取位姿并实测到位；夹爪保持打开，不是抓取成功
    GRASP_CONTACT_CONFIRMED, // A1 P3a-2c: 双手双指对杆接触并通过新鲜稳定窗口
    CONTACT_HOLD_MECHANISM_VALIDATED, // fixed-arm hold diagnostic only; no load/lift claim
    PLAN_COMPARE_DONE, // P4/S2 phase 1: planner comparison only; not a carry claim
    GRIPPER_TEST_DONE, // A1 P3a-2a: 空载开合测试结束；机械臂未动，不是搬运 DONE
    DONE,
    ERROR
};

// Where the task takes the rod pose from.
//   static    : the original fixed A0 pose (default, unchanged behaviour)
//   vision    : the original /detected_rod_pose input (equivalent to use_vision=true)
//   a1_truth  : the A1 P1b truth message; preparation only, no motion this round
inline const char* kRodPoseSourceStatic = "static";
inline const char* kRodPoseSourceVision = "vision";
inline const char* kRodPoseSourceA1Truth = "a1_truth";

class GripperController;
class DualArmController;

class DualArmCarryTask : public rclcpp::Node {
public:
    DualArmCarryTask();
    
    std::atomic<TaskState> current_state_;
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> dual_arm_;
    
    std::string left_arm_group_;
    std::string right_arm_group_;
    std::string dual_arm_group_;
    std::string control_mode_;
    double approach_height_;
    double grasp_height_;
    double lift_height_;
    bool enable_rotate_;
    double gripper_close_pos_;
    double place_release_hand_z_ref_ {0.0};
    bool place_release_hand_z_ref_valid_ {false};
    double rod_center_ground_z_ {0.02};
    double rod_half_height_ {0.02};

    // Mode B: vision-driven dynamic grasping
    geometry_msgs::msg::Pose detected_rod_pose_;
    bool rod_pose_received_ {false};
    bool use_vision_ {false};
    double rod_target_x_ {0.40};
    double rod_target_y_ {0.0};
    double rod_target_yaw_rad_ {20.0 * M_PI / 180.0};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rod_pose_sub_;

    // A1 P1b: rod pose source selection and the frozen truth reference.
    // `detected_rod_pose_` above is deliberately NOT reused for truth: its
    // received flag latches forever and it carries no run identity or validity.
    std::string rod_pose_source_ {kRodPoseSourceStatic};
    std::string expected_run_id_;
    bool prepare_only_ {false};
    std::string a1_truth_topic_ {"/a1_rod_pose_truth"};
    double a1_truth_timeout_sec_ {20.0};
    double rod_truth_size_min_ {0.005};
    double rod_truth_size_max_ {1.0};
    a1t::TruthLatch a1_truth_latch_;
    rclcpp::Subscription<dual_arm_a1_msgs::msg::RodPoseTruth>::SharedPtr a1_truth_sub_;

    // A1 P2a: static sample validity checks on the frozen truth. Runs inside the
    // PREPARED boundary and never moves the robot.
    bool p2a_check_ {false};
    // Configured research envelope for the complete rotated rod footprint. On
    // this scene the support surface is an unbounded MuJoCo `plane`, so these
    // bounds are a chosen research region, NOT a physical table edge. The
    // source string is recorded verbatim in the result.
    double p2a_workspace_x_min_ {0.30};
    double p2a_workspace_x_max_ {0.70};
    double p2a_workspace_y_min_ {-0.30};
    double p2a_workspace_y_max_ {0.30};
    std::string p2a_workspace_source_;
    double p2a_support_z_ {0.0};
    double p2a_resting_z_tol_ {0.003};
    double p2a_max_tilt_deg_ {2.0};
    // The MuJoCo floor never enters MoveIt, so the support surface is added to
    // the VERIFICATION scene only. `p2a_ground_negative` injects a deliberate
    // below-floor state to prove the check can reject a link passing through it.
    bool p2a_support_plane_ {true};
    std::string p2a_support_object_id_ {"a1_support_plane"};
    bool p2a_ground_negative_ {true};
    // Replace ONLY the two hand links' collision geometry with the conservative
    // AABB of the simulation's hand_c mesh, in an independently built model. The
    // original URDF result is always reported alongside as a reference.
    // Depth of the verification support box below the support plane. A thin
    // slab would leave deep paths unchecked, so it is a box, not a plate.
    double p2a_support_depth_m_ {2.0};
    bool p2a_hand_aabb_substitution_ {true};
    // Inject an obstacle coincident with the substituted hand box to prove the
    // replacement geometry still participates in collision checking.
    bool p2a_hand_negative_ {true};

    // A1 P2b: plan the joint pre-grasp path from the ACTUAL start state and
    // re-verify it on the P2a criterion model. Planning only -- never executes.
    bool p2b_check_ {false};
    std::string p2b_planner_id_ {"PTP"};
    double p2b_planning_time_sec_ {10.0};
    int p2b_planning_attempts_ {5};
    // Max single-joint step used when subdividing each trajectory segment for
    // collision checking. Recorded, not claimed as a continuous guarantee.
    double p2b_max_joint_step_rad_ {0.05};
    bool p2b_midpath_negative_ {true};

    // A1 P3a-1: execute the SAME joint pre-grasp plan that P2b just verified, in
    // this same simulation instance, then stop. Default off. It never enters the
    // legacy FSM, never descends and never touches the gripper/weld.
    bool pregrasp_execute_ {false};
    double pregrasp_vel_scale_ {0.1};
    double pregrasp_acc_scale_ {0.1};
    double pregrasp_arrival_pos_tol_m_ {0.005};
    double pregrasp_arrival_angle_tol_deg_ {2.0};
    double pregrasp_stable_window_sec_ {0.3};
    double pregrasp_stable_tol_rad_ {0.002};
    double pregrasp_exec_timeout_sec_ {90.0};

    // A1 P3a-2b: the descent from the reached pre-grasp pose to the grasp pose.
    // Off by default; enabling it never unlocks the legacy carry FSM.
    bool grasp_descent_ {false};
    bool grasp_contact_close_ {false};
    std::string grasp_model_ {"none"};
    bool contact_hold_diagnostic_only_ {false};
    bool contact_hover_only_ {false};
    std::array<unsigned long long, 2> contact_action_completion_seq_{{0, 0}};
    double contact_hover_height_m_ {0.05};
    double contact_hover_hold_seconds_ {3.0};
    std::string contact_hover_control_mode_ {"jtc"};
    // Optional complete-carry comparison group: A/B use JTC, C/D use MMC.
    // "none" preserves the established contact_probe entry point.
    std::string contact_abcd_group_ {"none"};
    // P4/S2 phase 1: compare the current object-space transport plan with a
    // MoveIt CHOMP plan to the same joint target. Planning only unless the
    // CHOMP plan also passes the closed-chain/collision/limit checks.
    bool contact_plan_compare_ {false};
    // S2: build the transport segment from a reconstructed nominal state and
    // compare the baseline path with the closed-chain projected path. Planning
    // only -- never closes the gripper, never sends a motion target.
    bool contact_transport_plan_only_ {false};
    double contact_transport_max_distance_m_ {0.20};
    std::atomic<bool> weld_state_received_ {false};
    std::atomic<bool> weld_state_active_ {false};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr weld_state_sub_;
    // Number of SEGMENTS in the Cartesian descent, so the path holds steps+1
    // targets including both ends.
    int descent_steps_ {20};
    // How far the ACHIEVED end-effector poses may depart from a straight
    // vertical descent before the path is rejected. The descent is only called
    // "along world Z" to the extent this measurement shows it is.
    double descent_max_xy_dev_m_ {0.002};
    double descent_max_angle_dev_deg_ {1.0};
    // A1 P3a-2b: physical rod truth, read from MuJoCo before and after the
    // descent so the physics measurement is reported separately from the
    // planning-scene object.
    std::string mujoco_get_body_state_service_ {"/get_body_state"};
    std::string mujoco_rod_body_name_ {"obj_rod_01"};
    std::string mujoco_admin_hash_ {""};
    double mujoco_truth_timeout_sec_ {2.0};
    // Tolerance for "the rod the arm is about to descend onto is still the rod
    // this round froze". Reuses the same pose-consistency bound the pre-grasp
    // stage already applies to the planning scene (5 mm / 5 deg).
    double descent_truth_pos_tol_m_ {0.005};
    double descent_truth_angle_tol_deg_ {5.0};
    // Test-only controlled injections for the P3a-2b negative examples. Both are
    // empty by default; when set they deliberately damage one thing so the
    // rejection path can be shown to fire and to dispatch nothing.
    //   descent_negative_final: "" | "position" | "collision"
    //   descent_negative_truth: "" | "unavailable" | "offset"
    std::string descent_negative_final_;
    std::string descent_negative_truth_;

    // A1 P3a-2b: fresh joint feedback for the arrival window.
    //
    // `getCurrentState()` returns whatever the monitor last assembled; calling
    // it twice in a row can hand back the same sample, so it cannot by itself
    // prove that new feedback arrived. This subscription carries the joint
    // values together with a monotonically increasing sequence number and the
    // moment the message arrived, which is what the arrival window counts.
    struct JointFeedback {
        unsigned long long seq {0};
        double recv_steady_sec {0.0};
        rclcpp::Time stamp;
        bool stamp_valid {false};
        std::map<std::string, double> positions;
    };
    std::string joint_states_topic_ {"/joint_states"};
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    mutable std::mutex joint_feedback_mutex_;
    JointFeedback latest_joint_feedback_;

    struct GripperContactFeedback {
        std::string arm;
        double q1 {0.0};
        double q2 {0.0};
        double command_per_finger_m {0.0};
        double closing_force_limit_per_finger_n {0.0};
        std::uint32_t rod_pad_contact_count {0};
        double normal_left_n {0.0};
        double normal_right_n {0.0};
        double tangent_left_n {0.0};
        double tangent_right_n {0.0};
        double rod_force_x_n {0.0};
        double rod_force_y_n {0.0};
        double rod_force_z_n {0.0};
        double simulation_time {0.0};
        double hold_force_per_finger_n {0.0};
        bool force_hold_supported {false};
        bool weld_left_active {false};
        bool weld_right_active {false};
        bool rod_constraint_active {false};

        bool left_contact {false};
        bool right_contact {false};
        unsigned long long seq {0};
        std::int64_t stamp_ns {0};
        double recv_steady_sec {0.0};
    };
    rclcpp::Subscription<dual_arm_a1_msgs::msg::GripperContactFeedback>::SharedPtr
        gripper_contact_sub_;
    mutable std::mutex gripper_contact_mutex_;
    std::map<std::string, GripperContactFeedback> gripper_contact_feedback_;
    void receiveGripperContact(const dual_arm_a1_msgs::msg::GripperContactFeedback& msg) {
        std::lock_guard<std::mutex> guard(gripper_contact_mutex_);
        auto& sample = gripper_contact_feedback_[msg.arm];
        sample.arm = msg.arm;
        sample.q1 = msg.finger_q1;
        sample.q2 = msg.finger_q2;
        sample.command_per_finger_m = msg.command_per_finger_m;
        sample.closing_force_limit_per_finger_n = msg.closing_force_limit_per_finger_n;
        sample.normal_left_n = msg.normal_left_n;
        sample.normal_right_n = msg.normal_right_n;
        sample.tangent_left_n = msg.tangent_left_n;
        sample.tangent_right_n = msg.tangent_right_n;
        sample.rod_force_x_n = msg.rod_force_x_n;
        sample.rod_force_y_n = msg.rod_force_y_n;
        sample.rod_force_z_n = msg.rod_force_z_n;
        sample.simulation_time = msg.simulation_time;
        sample.rod_pad_contact_count = msg.rod_pad_contact_count;
        sample.hold_force_per_finger_n = msg.hold_force_per_finger_n;
        sample.force_hold_supported = msg.force_hold_supported;
        sample.weld_left_active = msg.weld_left_active;
        sample.weld_right_active = msg.weld_right_active;
        sample.rod_constraint_active = msg.rod_constraint_active;

        sample.left_contact = msg.left_finger_rod_contact;
        sample.right_contact = msg.right_finger_rod_contact;
        sample.seq = msg.update_seq;
        sample.stamp_ns = static_cast<std::int64_t>(msg.header.stamp.sec) * 1000000000LL +
                          static_cast<std::int64_t>(msg.header.stamp.nanosec);
        sample.recv_steady_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    bool latestGripperContact(const std::string& arm, GripperContactFeedback& out) const {
        std::lock_guard<std::mutex> guard(gripper_contact_mutex_);
        const auto it = gripper_contact_feedback_.find(arm);
        if (it == gripper_contact_feedback_.end() || it->second.seq == 0) return false;
        out = it->second;
        return true;
    }

    /// Copies the newest feedback under the lock. Returns false when nothing has
    /// arrived yet, so "no feedback" can never be read as a sample.
    bool latestJointFeedback(JointFeedback& out) const
    {
        std::lock_guard<std::mutex> guard(joint_feedback_mutex_);
        if (latest_joint_feedback_.seq == 0) {
            return false;
        }
        out = latest_joint_feedback_;
        return true;
    }

    // A1 P3a-2a: gripper action interface bounds. The stage budgets are
    // per-stage and are additionally clamped by the single total budget, so two
    // serial stage timeouts are never presented as one total timeout.
    double gripper_total_timeout_sec_ {20.0};
    double gripper_server_timeout_sec_ {10.0};
    double gripper_goal_timeout_sec_ {5.0};
    double gripper_result_timeout_sec_ {14.0};
    double gripper_max_effort_ {100.0};
    // command.position is a per-finger displacement; below this the call counts
    // as a closing move for result interpretation.
    double gripper_close_threshold_ {0.03};
    // A1 P3a-2a: minimal explicit empty-load cycle entry (open -> close -> open).
    bool gripper_selftest_ {false};
    // Open command for the empty-load cycle (per-finger). The closing command
    // reuses the pre-existing `gripper_close_position` / gripper_close_pos_ so
    // the selftest and the task cannot drift apart.
    double gripper_open_position_ {0.04};
    // Grasp offset along the rod's long axis; A0 uses 0.15.
    double p2a_grasp_offset_m_ {0.15};
    // Yaw term of A0's setRPY(pi, 0, pi/4).
    double p2a_eef_base_yaw_deg_ {45.0};
    int p2a_ik_attempts_ {5};
    double p2a_ik_timeout_sec_ {0.05};
    double p2a_ik_pos_tol_m_ {0.001};
    double p2a_ik_angle_tol_deg_ {0.5};
    double p2a_max_joint_jump_rad_ {2.0};

    // True when this run takes its rod pose from the A1 truth message.
    bool usesA1Truth() const
    {
        return rod_pose_source_ == kRodPoseSourceA1Truth;
    }

    // Frozen truth reference, valid only when the latch is frozen.
    bool getFrozenRodTruth(a1t::TruthPayload& out) const
    {
        if (!a1_truth_latch_.frozen()) {
            return false;
        }
        out = a1_truth_latch_.value();
        return true;
    }
    
    std::vector<double> initial_left_joints_;
    std::vector<double> initial_right_joints_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr weld_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_stage_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;
    rclcpp::TimerBase::SharedPtr wait_timer_;
    
    std::shared_ptr<GripperController> grippers_;
    std::shared_ptr<DualArmController> arms_;
    
    void executeTask();

    TaskState getCurrentState() const
    {
        return current_state_.load(std::memory_order_acquire);
    }

    void setCurrentState(TaskState state)
    {
        current_state_.store(state, std::memory_order_release);
    }

    void setDetectedRodPose(const geometry_msgs::msg::Pose& pose)
    {
        std::lock_guard<std::mutex> lock(rod_pose_mutex_);
        detected_rod_pose_ = pose;
        rod_pose_received_ = true;
    }

    bool getDetectedRodPose(geometry_msgs::msg::Pose& pose) const
    {
        std::lock_guard<std::mutex> lock(rod_pose_mutex_);
        if (!rod_pose_received_) {
            return false;
        }
        pose = detected_rod_pose_;
        return true;
    }

    bool hasDetectedRodPose() const
    {
        std::lock_guard<std::mutex> lock(rod_pose_mutex_);
        return rod_pose_received_;
    }

private:
    mutable std::mutex rod_pose_mutex_;
};
