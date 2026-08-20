with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "r") as f:
    cpp_code = f.read()

old_str = 'RCLCPP_INFO(this->get_logger(), "    ✓ 任务完成！");'
new_str = old_str + '''
            std_msgs::msg::String msg;
            msg.data = "DONE";
            task_status_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "已发布任务完成状态(DONE)。");
'''

if 'msg.data = "DONE";' not in cpp_code:
    cpp_code = cpp_code.replace(old_str, new_str)

with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "w") as f:
    f.write(cpp_code)
