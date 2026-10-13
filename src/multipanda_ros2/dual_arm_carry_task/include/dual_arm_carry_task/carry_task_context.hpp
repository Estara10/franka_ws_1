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
#include "dual_arm_carry_task/a1_truth.hpp"
#include "dual_arm_carry_task/a1_sample_validator.hpp"
#include <memory>
#include <thread>
#include <chrono>
#include <set>
#include <map>
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
    double rod_target_y_ {-0.20};
    double rod_target_yaw_rad_ {M_PI / 2.0};
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

    // A1 P3a-2a: gripper action interface bounds. The stage budgets are
    // per-stage and are additionally clamped by the single total budget, so two
    // serial stage timeouts are never presented as one total timeout.
    double gripper_total_timeout_sec_ {20.0};
    double gripper_server_timeout_sec_ {10.0};
    double gripper_goal_timeout_sec_ {5.0};
    double gripper_result_timeout_sec_ {8.0};
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
