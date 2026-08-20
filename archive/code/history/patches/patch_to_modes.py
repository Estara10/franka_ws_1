import re

with open('src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp', 'r') as f:
    cpp_content = f.read()

# Update admittance condition
cpp_content = cpp_content.replace(
    'if (control_mode_ == "rigid" || control_mode_ != "compliant") {',
    'if (control_mode_ != "compliant" && control_mode_ != "compliant_chomp") {'
)

# Update CHOMP Approach
old_chomp_approach = """            // [显式开启 CHOMP 优化] 
            // 此处应用了 CHOMP 算法进行时间-能量综合优化，平滑 OMPL 生成的初始轨迹
            dual_arm_->setPlanningPipelineId("ompl_chomp");
            dual_arm_->setPlanningTime(15.0); // 延长计算时间以确保 CHOMP 充分收敛"""
            
new_chomp = """            // [动态规划管道分配] 根据控制模式选择是否开启 CHOMP
            if (control_mode_ == "chomp_only" || control_mode_ == "compliant_chomp") {
                // 此处应用了 CHOMP 算法进行时间-能量综合优化，平滑 OMPL 生成的初始轨迹
                dual_arm_->setPlanningPipelineId("ompl_chomp");
                dual_arm_->setPlanningTime(15.0); // 延长计算时间以确保 CHOMP 充分收敛
                RCLCPP_INFO(this->get_logger(), "  [规划配置] 已激活 CHOMP 优化管道");
            } else {
                // 仅使用标准 OMPL 进行基础避障找路
                dual_arm_->setPlanningPipelineId("ompl");
                dual_arm_->setPlanningTime(10.0);
                RCLCPP_INFO(this->get_logger(), "  [规划配置] 使用基础 OMPL 管道 (无 CHOMP)");
            }"""

cpp_content = cpp_content.replace(old_chomp_approach, new_chomp)

with open('src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp', 'w') as f:
    f.write(cpp_content)
