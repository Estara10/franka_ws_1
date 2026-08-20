#ifndef DUAL_ARM_GRIPPER_CONTROLLER_HPP
#define DUAL_ARM_GRIPPER_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

class DualArmGripperController
{
public:
    DualArmGripperController(const std::shared_ptr<rclcpp::Node>& node);

    /**
     * @brief Open both grippers using named target "open"
     */
    bool open_grippers();

    /**
     * @brief Close both grippers using named target "close"
     */
    bool close_grippers();

    /**
     * @brief Add the long bar object to the planning scene
     */
    void add_collision_object();

    /**
     * @brief Attach the existing object to the robot hands
     *        This connects it to panda_1_link8 and ignores collisions with both hands
     */
    void attach_object_to_hands();

    /**
     * @brief Detach the object and place it back into the world
     */
    void detach_object();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_1_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_2_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    
    // Constants
    const std::string OBJECT_ID = "long_bar";
};

#endif // DUAL_ARM_GRIPPER_CONTROLLER_HPP
