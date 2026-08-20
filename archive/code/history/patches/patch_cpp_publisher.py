import re

with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "r") as f:
    cpp_code = f.read()

# 1. Add publisher definition
pub_def = "    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;"
if "task_status_pub_" not in cpp_code:
    # insert before control_mode_
    cpp_code = cpp_code.replace("    std::string control_mode_;", pub_def + "\n    std::string control_mode_;")

# 2. Add publisher initialization in constructor
pub_init = '        task_status_pub_ = this->create_publisher<std_msgs::msg::String>("/task_status", 10);'
if "/task_status" not in cpp_code:
    # insert inside constructor, around line 74 "gripper_close_position"
    # Actually just insert it after the declare_parameters
    cpp_code = cpp_code.replace('        dual_arm_group_ = this->get_parameter("dual_arm_group").as_string();', pub_init + '\n        dual_arm_group_ = this->get_parameter("dual_arm_group").as_string();')

# 3. Add publish DONE at the end of executeRetreat or executeTask
if "DONE" not in cpp_code and '"DONE"' not in cpp_code:
    publish_str = '''
        std_msgs::msg::String msg;
        msg.data = "DONE";
        task_status_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "已发布任务完成状态(DONE)。");
'''
    # We will put it at the very end of executeTask
    old_end = 'RCLCPP_INFO(this->get_logger(), "=== 搬运任务已全部执行完成 ===");'
    new_end = old_end + publish_str
    cpp_code = cpp_code.replace(old_end, new_end)


with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "w") as f:
    f.write(cpp_code)
