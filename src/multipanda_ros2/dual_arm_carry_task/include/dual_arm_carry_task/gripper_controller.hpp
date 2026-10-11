#pragma once
#include "carry_task_context.hpp"

class GripperController {
    DualArmCarryTask* ctx_;
    using GripperAction = control_msgs::action::GripperCommand;
    using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperAction>;
public:
    GripperController(DualArmCarryTask* ctx) : ctx_(ctx) {}
    bool controlDualGrippers(double position);
    bool controlGripper(const std::string& arm_name, double position);
};
