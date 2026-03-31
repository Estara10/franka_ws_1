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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <memory>
#include <thread>
#include <chrono>
#include <set>
#include <map>

enum class TaskState {
    INIT,
    APPROACH,
    GRASP,
    LIFT,
    TRANSPORT,
    ROTATE,      // 旋转物体方向（Y轴→X轴）
    DESCEND,     // 下降准备放置
    PLACE,       // 放置物体（GRASP逆操作）
    RETREAT,     // 撤退至初始位置
    DONE,
    ERROR
};

class GripperController;
class DualArmController;

class DualArmCarryTask : public rclcpp::Node {
public:
    DualArmCarryTask();
    
    TaskState current_state_;
    
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> dual_arm_;
    
    std::string left_arm_group_;
    std::string right_arm_group_;
    std::string dual_arm_group_;
    double approach_height_;
    double grasp_height_;
    double lift_height_;
    bool enable_rotate_;
    double gripper_close_pos_;
    
    std::vector<double> initial_left_joints_;
    std::vector<double> initial_right_joints_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr weld_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;
    
    std::shared_ptr<GripperController> grippers_;
    std::shared_ptr<DualArmController> arms_;
    
    void executeTask();
};
