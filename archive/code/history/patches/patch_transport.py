with open('src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp', 'r') as f:
    text = f.read()

target = 'RCLCPP_INFO(this->get_logger(), "[状态: TRANSPORT] 平移物体...");'

new_text = target + '''
        
        // [配置恢复] 重置为默认的 OMPL 管道和标准规划时间，以免 CHOMP 干扰之前写好的密集航点（Waypoint）TOTG 插值逻辑
        dual_arm_->setPlanningPipelineId("ompl");
        dual_arm_->setPlanningTime(10.0);
        left_arm_->setPlanningPipelineId("ompl");
        right_arm_->setPlanningPipelineId("ompl");'''

text = text.replace(target, new_text)

with open('src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp', 'w') as f:
    f.write(text)
