import re

with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "r") as f:
    cpp_code = f.read()

if "rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;" not in cpp_code:
    cpp_code = cpp_code.replace("private:", "private:\n    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;")

with open("src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp", "w") as f:
    f.write(cpp_code)
