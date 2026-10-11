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
