import re

with open('start_dual_arm_task.sh', 'r') as f:
    sh_content = f.read()

# Update script to show the new modes
old_script_prompt = """echo "请选择要运行的实验模式:"
echo "  1) rigid      - 刚性对照模式 (无任何柔顺补偿)"
echo "  2) compliant  - 主从导纳柔顺模式 (当前有效算法)"
echo "  3) opt_1      - 预留规划优化算法1 (暂同刚性)"
echo "  4) opt_2      - 预留规划优化算法2 (暂同刚性)"
read -p "请输入模式编号 [默认: 2]: " MODE_SELECTION

case "$MODE_SELECTION" in
    1) CONTROL_MODE="rigid" ;;
    2) CONTROL_MODE="compliant" ;;
    3) CONTROL_MODE="opt_1" ;;
    4) CONTROL_MODE="opt_2" ;;
    *) CONTROL_MODE="compliant" ;; # 默认选2
esac"""

new_script_prompt = """echo "请选择要运行的实验模式:"
echo "  1) rigid             - 刚性对照模式 (无导纳柔顺, 无CHOMP)"
echo "  2) compliant         - 柔顺控制模式 (强导纳柔顺, 无CHOMP)"
echo "  3) chomp_only        - 仅限优化模式 (无导纳柔顺, 强CHOMP顺滑)"
echo "  4) compliant_chomp   - 全功能模式   (强导纳柔顺 + 强CHOMP顺滑) [默认]"
read -p "请输入模式编号 [默认: 4]: " MODE_SELECTION

case "$MODE_SELECTION" in
    1) CONTROL_MODE="rigid" ;;
    2) CONTROL_MODE="compliant" ;;
    3) CONTROL_MODE="chomp_only" ;;
    4) CONTROL_MODE="compliant_chomp" ;;
    *) CONTROL_MODE="compliant_chomp" ;; # 默认选4
esac"""

sh_content = sh_content.replace(old_script_prompt, new_script_prompt)

with open('start_dual_arm_task.sh', 'w') as f:
    f.write(sh_content)


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

# Update CHOMP Retreat
            // 此处应用了 CHOMP 算法进行时间-能量综合优化，平滑 OMPL 生成的初始轨迹
            dual_arm_->setPlanningPipelineId("ompl_chomp");
            dual_arm_->setPlanningTime(15.0); // 延长规划时间，确保 CHOMP 处理平移或者可能有碰撞的情况"""

cpp_content = cpp_content.replace(old_chomp_retreat, new_chomp)

with open('src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp', 'w') as f:
    f.write(cpp_content)


with open('sensor_dashboard.py', 'r') as f:
    py_content = f.read()

py_content = py_content.replace('def __init__(self, mode="compliant"):', 'def __init__(self, mode="compliant_chomp"):')
py_content = py_content.replace('mode = "compliant"', 'mode = "compliant_chomp"')

with open('sensor_dashboard.py', 'w') as f:
    f.write(py_content)
