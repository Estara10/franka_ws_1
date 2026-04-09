#!/bin/bash

# 双臂搬运任务 - 完整启动脚本
# 功能: 生成SRDF → 启动 MoveIt2 (自动启动MuJoCo) → 等待初始化 → 启动任务节点
# 更新: 2026-01-31 - 增强SRDF诊断，添加ACM验证

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${WORKSPACE_DIR}/launch_logs"
RESULTS_DIR="${WORKSPACE_DIR}/experiment_results"
RESULT_DATA_DIR="${RESULTS_DIR}/data"
RESULT_REPORT_DIR="${RESULTS_DIR}/reports"
RESULT_PLOT_DIR="${RESULTS_DIR}/plots"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DIAG_LOG="${LOG_DIR}/diagnosis_${TIMESTAMP}.log"

# 保存 PID 到文件以便停止
PID_FILE="${WORKSPACE_DIR}/.running_pids.txt"

# 创建日志目录
mkdir -p "$LOG_DIR"
mkdir -p "$RESULT_DATA_DIR" "$RESULT_REPORT_DIR" "$RESULT_PLOT_DIR"

# 清理函数
cleanup_processes() {
    echo ""
    echo "正在清理所有进程..."
    if [ -f "$PID_FILE" ]; then
        while read pid; do
            if ps -p $pid > /dev/null 2>&1; then
                kill -9 $pid 2>/dev/null || true
            fi
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi
    pkill -9 -f "ros2 launch franka_moveit_config" 2>/dev/null || true
    pkill -9 -f "ros2 launch dual_arm_carry_task" 2>/dev/null || true
    pkill -9 -f "mujoco_ros" 2>/dev/null || true
    pkill -9 -f "mujoco_node" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "move_group" 2>/dev/null || true
    pkill -9 -f "dual_arm_carry_task" 2>/dev/null || true
    sleep 2
    echo "✓ 清理完成"
}

# 检查 MoveIt + MuJoCo 栈是否真正就绪（不仅 launch 进程存活）
wait_for_sim_stack_ready() {
    local timeout_sec="${1:-12}"
    local elapsed=0

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        local nodes
        local services
        nodes=$(timeout 2s ros2 node list 2>/dev/null || true)
        services=$(timeout 2s ros2 service list 2>/dev/null || true)

        # 关键判据：
        # 1) move_group 已就绪
        # 2) MuJoCo 主节点或 ros2_control 子节点存在
        # 3) controller_manager 服务可用（比节点名更稳定）
        local has_move_group=0
        local has_mujoco=0
        local has_controller_service=0

        if echo "$nodes" | grep -q "/move_group"; then
            has_move_group=1
        fi
        if echo "$nodes" | grep -q "/mujoco_node" || \
           echo "$nodes" | grep -q "/mujoco_ros2_control"; then
            has_mujoco=1
        fi
        if echo "$services" | grep -q "/controller_manager/list_controllers"; then
            has_controller_service=1
        fi

        if [ "$has_move_group" -eq 1 ] && \
           [ "$has_mujoco" -eq 1 ] && \
           [ "$has_controller_service" -eq 1 ]; then
            return 0
        fi

        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 1
}

# 监控任务终态（DONE / ERROR / 超时 / 进程退出）
wait_for_task_terminal_state() {
    local task_log="$1"
    local task_pid="$2"
    local timeout_sec="${3:-240}"
    local elapsed=0

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        if [ -f "$task_log" ]; then
            if grep -q "✓ 任务完成！" "$task_log"; then
                return 0
            fi
            if grep -q "✗ 任务失败！" "$task_log"; then
                return 2
            fi
        fi

        if ! ps -p "$task_pid" > /dev/null 2>&1; then
            return 3
        fi

        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 1
}

# 解析可用的双臂阻抗参数服务。返回0并输出服务名表示成功。
resolve_adaptive_impedance_service() {
    local requested_service="$1"
    local services
    local detected

    services=$(timeout 3s ros2 service list 2>/dev/null || true)
    if [ -z "$services" ]; then
        return 1
    fi

    if [ "$requested_service" != "auto" ]; then
        if echo "$services" | grep -Fxq "$requested_service"; then
            echo "$requested_service"
            return 0
        fi
        return 2
    fi

    detected=$(echo "$services" | grep -E '/[[:alnum:]_]+_and_[[:alnum:]_]+/dual_cartesian_impedance_controller/parameters$' | head -1 || true)
    if [ -z "$detected" ]; then
        detected=$(echo "$services" | grep -E '/left_and_right/dual_cartesian_impedance_controller/parameters$' | head -1 || true)
    fi
    if [ -z "$detected" ]; then
        detected=$(echo "$services" | grep -E '/.*/dual_cartesian_impedance_controller/parameters$' | head -1 || true)
    fi

    if [ -n "$detected" ]; then
        echo "$detected"
        return 0
    fi

    return 1
}

# 捕获 Ctrl+C 信号
# 注意：必须返回 130（被中断）而不是 0，避免外层批处理脚本误判为“本轮成功”并继续下一轮。
trap 'echo ""; echo "检测到 Ctrl+C，正在停止所有进程..."; cleanup_processes; exit 130' INT TERM

echo "========================================"
echo "  双臂搬运任务 - 完整启动"
echo "========================================"
echo ""

# ----------------- 模式选择逻辑重构 (添加视觉模组接口) -----------------
echo "请选择要运行的大模式 (A 或 B):"
echo "  A) 经典固态搬运模式 (固定测试坐标，现有算法保底闭环)"
echo "  B) 视觉动态抓取模式 (引入视觉感知，由用户指定初始与目标位置)"
read -p "请输入大模式代号 [默认: A]: " MAIN_MODE

if [[ "$MAIN_MODE" == "B" || "$MAIN_MODE" == "b" ]]; then
    export TASK_MAIN_MODE="B"
    echo ""
    echo "========================================"
    echo "  [模式 B] 视觉动态感知与搬运"
    echo "========================================"
    echo "机器人双臂协作存在运动学约束，请输入物体的空间坐标(单位:m)。"
    echo "【推荐公共工作空间范围】"
    echo "  ▶ 前后(X)范围: [0.35, 0.65]"
    echo "  ▶ 左右(Y)范围: [-0.25, 0.25]"
    echo ""
    
    # 定义输入验证函数，确保数据在物理合理范围内
    get_valid_input() {
        local prompt="$1"
        local default_val="$2"
        local min_val="$3"
        local max_val="$4"
        local var_name="$5"
        local input_val
        
        while true; do
            read -p "$prompt [默认: $default_val]: " input_val
            input_val=${input_val:-$default_val}
            
            # 使用 awk 验证是否为数字且在范围内
            if ! awk -v val="$input_val" -v min="$min_val" -v max="$max_val" 'BEGIN {
                if (val !~ /^[-+]?[0-9]*\.?[0-9]+$/) { exit 1 }
                if (val < min || val > max) { exit 2 }
                exit 0
            }'; then
                local exit_code=$?
                if [ $exit_code -eq 1 ]; then
                    echo "  ❌ 错误: '$input_val' 不是有效的数字！"
                elif [ $exit_code -eq 2 ]; then
                    echo "  ❌ 错误: '$input_val' 超出合理工作空间 [$min_val, $max_val]，会导致无逆解或干涉，请重新输入。"
                fi
                continue
            fi
            
            export $var_name="$input_val"
            break
        done
    }

    # 通过环境变量传参，实施严格边界管控
    get_valid_input "1. 设定铝棒初始 X 坐标" "0.50" "0.30" "0.70" "ROD_INIT_X"
    get_valid_input "2. 设定铝棒初始 Y 坐标" "0.00" "-0.25" "0.25" "ROD_INIT_Y"
    get_valid_input "3. 设定铝棒初始偏航角(Yaw) (度, -90~90)" "0" "-90" "90" "ROD_INIT_YAW"
    echo ""
    get_valid_input "4. 设定放置目标 X 坐标" "0.40" "0.30" "0.70" "ROD_TARGET_X"
    get_valid_input "5. 设定放置目标 Y 坐标" "-0.20" "-0.25" "0.25" "ROD_TARGET_Y"
    get_valid_input "6. 设定放置目标偏航角(Yaw) (度, -90~90)" "90" "-90" "90" "ROD_TARGET_YAW"

    echo ""
    echo "✅ 动态位姿已锁定 -> 初始位置:(X:${ROD_INIT_X}, Y:${ROD_INIT_Y}, 角度:${ROD_INIT_YAW}°) | 目标位置:(X:${ROD_TARGET_X}, Y:${ROD_TARGET_Y}, 角度:${ROD_TARGET_YAW}°)"
    
    # 模式B直接默认使用最好的自适应架构
    CONTROL_MODE="vision_compliant_chomp"

else
    export TASK_MAIN_MODE="A"
    echo ""
    echo "========================================"
    echo "  [模式 A] 经典固态搬运 (作为基础对照与保底)"
    echo "========================================"
    echo "请选择内部的测试子模式:"
    echo "  1) rigid             - 刚性对照模式 (无柔顺, 无CHOMP)"
    echo "  2) compliant         - 柔顺控制模式 (内部柔顺, 无CHOMP)"
    echo "  3) chomp_only        - 仅限优化模式 (无柔顺, CHOMP顺滑)"
    echo "  4) compliant_chomp   - 组合优化模式 (内部柔顺 + CHOMP顺滑) [默认]"
    read -p "请输入模式编号 [默认: 4]: " MODE_SELECTION

    case "$MODE_SELECTION" in
        1) CONTROL_MODE="rigid" ;;
        2) CONTROL_MODE="compliant" ;;
        3) CONTROL_MODE="chomp_only" ;;
        4) CONTROL_MODE="compliant_chomp" ;;
        *) CONTROL_MODE="compliant_chomp" ;; # 默认选4
    esac
fi

echo ""
echo "✅ 最终设定的运行配置: ===[ 大模式: ${TASK_MAIN_MODE} | 核心策略: ${CONTROL_MODE} ]==="
echo ""

APPROACH_HEIGHT="0.28"
GRASP_HEIGHT="0.13"
LIFT_HEIGHT="0.40"

# MuJoCo 硬件层内部柔顺开关：用于保证四模式对比真实隔离
#   ENABLE_INTERNAL_COMPLIANCE=auto|true|false
ENABLE_INTERNAL_COMPLIANCE="${ENABLE_INTERNAL_COMPLIANCE:-auto}"
if [[ "$ENABLE_INTERNAL_COMPLIANCE" == "auto" ]]; then
    case "$CONTROL_MODE" in
        rigid|chomp_only)
            INTERNAL_COMPLIANCE_ENABLED="false"
            ;;
        *)
            INTERNAL_COMPLIANCE_ENABLED="true"
            ;;
    esac
else
    if [[ "$ENABLE_INTERNAL_COMPLIANCE" == "1" || "$ENABLE_INTERNAL_COMPLIANCE" == "true" || "$ENABLE_INTERNAL_COMPLIANCE" == "yes" ]]; then
        INTERNAL_COMPLIANCE_ENABLED="true"
    else
        INTERNAL_COMPLIANCE_ENABLED="false"
    fi
fi
export MJ_INTERNAL_COMPLIANCE_ENABLED="${INTERNAL_COMPLIANCE_ENABLED}"

MJ_INTERNAL_COMPLIANCE_STRESS_THRESHOLD="2.0"
MJ_INTERNAL_COMPLIANCE_K_GAIN="0.08"
MJ_INTERNAL_COMPLIANCE_D_GAIN="0.15"
MJ_INTERNAL_COMPLIANCE_MIN_K="0.45"
MJ_INTERNAL_COMPLIANCE_MAX_D="1.80"

ADAPTIVE_NOMINAL_STIFFNESS="400.0"
ADAPTIVE_MIN_STIFFNESS="180.0"
ADAPTIVE_FORCE_GAIN="1.6"
ADAPTIVE_IMBALANCE_GAIN="6.0"
ADAPTIVE_ACTIVE_STAGES="TRANSPORT,ROTATE,DESCEND"

case "$CONTROL_MODE" in
    compliant_chomp)
        MJ_INTERNAL_COMPLIANCE_STRESS_THRESHOLD="3.0"
        MJ_INTERNAL_COMPLIANCE_K_GAIN="0.03"
        MJ_INTERNAL_COMPLIANCE_D_GAIN="0.06"
        MJ_INTERNAL_COMPLIANCE_MIN_K="0.72"
        MJ_INTERNAL_COMPLIANCE_MAX_D="1.20"
        ADAPTIVE_NOMINAL_STIFFNESS="420.0"
        ADAPTIVE_MIN_STIFFNESS="300.0"
        ADAPTIVE_FORCE_GAIN="0.7"
        ADAPTIVE_IMBALANCE_GAIN="2.0"
        ADAPTIVE_ACTIVE_STAGES="DESCEND"
        ;;
    rigid|chomp_only)
        ADAPTIVE_NOMINAL_STIFFNESS="400.0"
        ADAPTIVE_MIN_STIFFNESS="400.0"
        ADAPTIVE_FORCE_GAIN="0.0"
        ADAPTIVE_IMBALANCE_GAIN="0.0"
        ADAPTIVE_ACTIVE_STAGES="DESCEND"
        ;;
esac

export MJ_INTERNAL_COMPLIANCE_STRESS_THRESHOLD
export MJ_INTERNAL_COMPLIANCE_K_GAIN
export MJ_INTERNAL_COMPLIANCE_D_GAIN
export MJ_INTERNAL_COMPLIANCE_MIN_K
export MJ_INTERNAL_COMPLIANCE_MAX_D

# 自适应柔顺闭环开关：
#   在当前经典搬运流程中，MoveIt 依赖 dual_panda_arm_controller（位置轨迹控制），
#   而 adaptive supervisor 依赖 sim_multi_mode_controller/dual_cartesian_impedance_controller。
#   这两套控制器无法同时处于 active，因此 auto 策略默认关闭，避免 mode4 出现“名义启用，实际冲突”的假配置。
#   如需研究多模态阻抗链路，可显式设 ENABLE_ADAPTIVE_COMPLIANCE=true 手动测试。
#   ENABLE_ADAPTIVE_COMPLIANCE=auto|true|false
ENABLE_ADAPTIVE_COMPLIANCE="${ENABLE_ADAPTIVE_COMPLIANCE:-auto}"
if [[ "$ENABLE_ADAPTIVE_COMPLIANCE" == "auto" ]]; then
    ADAPTIVE_COMPLIANCE_ENABLED="false"
else
    if [[ "$ENABLE_ADAPTIVE_COMPLIANCE" == "1" || "$ENABLE_ADAPTIVE_COMPLIANCE" == "true" || "$ENABLE_ADAPTIVE_COMPLIANCE" == "yes" ]]; then
        ADAPTIVE_COMPLIANCE_ENABLED="true"
    else
        ADAPTIVE_COMPLIANCE_ENABLED="false"
    fi
fi
ADAPTIVE_IMPEDANCE_SERVICE="${ADAPTIVE_IMPEDANCE_SERVICE:-auto}"
ADAPTIVE_MMC_FORCE_SWITCH="${ADAPTIVE_MMC_FORCE_SWITCH:-false}"
echo "MuJoCo内部柔顺: ${INTERNAL_COMPLIANCE_ENABLED} (ENABLE_INTERNAL_COMPLIANCE=${ENABLE_INTERNAL_COMPLIANCE})"
echo "内部柔顺参数: threshold=${MJ_INTERNAL_COMPLIANCE_STRESS_THRESHOLD} K_gain=${MJ_INTERNAL_COMPLIANCE_K_GAIN} D_gain=${MJ_INTERNAL_COMPLIANCE_D_GAIN} min_K=${MJ_INTERNAL_COMPLIANCE_MIN_K} max_D=${MJ_INTERNAL_COMPLIANCE_MAX_D}"
echo "自适应柔顺闭环: ${ADAPTIVE_COMPLIANCE_ENABLED} (ENABLE_ADAPTIVE_COMPLIANCE=${ENABLE_ADAPTIVE_COMPLIANCE})"
if [[ "$ENABLE_ADAPTIVE_COMPLIANCE" == "auto" ]]; then
    echo "自适应柔顺说明: 经典搬运流程默认关闭外部MMC阻抗监督，避免与MoveIt轨迹控制冲突"
fi
echo "自适应阻抗服务策略: ${ADAPTIVE_IMPEDANCE_SERVICE}"
echo "自适应MMC强制切换: ${ADAPTIVE_MMC_FORCE_SWITCH}"
echo "监督式柔顺参数: nominal=${ADAPTIVE_NOMINAL_STIFFNESS} min=${ADAPTIVE_MIN_STIFFNESS} force_gain=${ADAPTIVE_FORCE_GAIN} imbalance_gain=${ADAPTIVE_IMBALANCE_GAIN} stages=${ADAPTIVE_ACTIVE_STAGES}"

# 旋转阶段开关（默认启用）。可通过环境变量覆盖：ENABLE_ROTATE=false ./start_dual_arm_task.sh
ENABLE_ROTATE="${ENABLE_ROTATE:-true}"
echo "旋转阶段开关: ${ENABLE_ROTATE}"

echo "日志目录: $LOG_DIR"
echo "诊断日志: $DIAG_LOG"
echo "结果目录: $RESULTS_DIR"
echo "  - 数据CSV: $RESULT_DATA_DIR"
echo "  - 报告CSV/MD: $RESULT_REPORT_DIR"
echo "  - 图表PNG: $RESULT_PLOT_DIR"
echo ""

# 0. Source 工作空间
echo "[0/4] 加载ROS2工作空间环境..."
source "${WORKSPACE_DIR}/install/setup.bash"
echo "✓ 环境已加载"
echo ""

# 0.5 防呆：若源码比 install 二进制新，自动重编译，避免“改了代码但仍跑旧版本”
echo "[0.5/4] 检查 dual_arm_carry_task / franka_hardware 是否需要重编译..."
TASK_NODE_BIN="${WORKSPACE_DIR}/install/dual_arm_carry_task/lib/dual_arm_carry_task/dual_arm_carry_task_node"
TASK_SRC_DIR="${WORKSPACE_DIR}/src/multipanda_ros2/dual_arm_carry_task"
FRANKA_HW_BIN="${WORKSPACE_DIR}/install/franka_hardware/lib/libfranka_mj_hardware.so"
FRANKA_HW_SRC_DIR="${WORKSPACE_DIR}/src/multipanda_ros2/franka_hardware"

needs_rebuild=0
if [ ! -f "$TASK_NODE_BIN" ]; then
    needs_rebuild=1
else
    if find "${TASK_SRC_DIR}/src" "${TASK_SRC_DIR}/include" "${TASK_SRC_DIR}/launch" \
        -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.py" -o -name "CMakeLists.txt" -o -name "package.xml" \) \
        -newer "$TASK_NODE_BIN" | head -1 | grep -q .; then
        needs_rebuild=1
    fi
fi

needs_hw_rebuild=0
if [ ! -f "$FRANKA_HW_BIN" ]; then
    needs_hw_rebuild=1
else
    if find "${FRANKA_HW_SRC_DIR}/src" "${FRANKA_HW_SRC_DIR}/include" \
        -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "CMakeLists.txt" -o -name "package.xml" \) \
        -newer "$FRANKA_HW_BIN" | head -1 | grep -q .; then
        needs_hw_rebuild=1
    fi
fi

if [ "$needs_rebuild" -eq 1 ] || [ "$needs_hw_rebuild" -eq 1 ]; then
    BUILD_PACKAGES=""
    if [ "$needs_hw_rebuild" -eq 1 ]; then
        BUILD_PACKAGES="${BUILD_PACKAGES} franka_hardware"
    fi
    if [ "$needs_rebuild" -eq 1 ]; then
        BUILD_PACKAGES="${BUILD_PACKAGES} dual_arm_carry_task"
    fi
    echo "⚠ 检测到源码更新，自动执行增量编译:${BUILD_PACKAGES}"
    (
        cd "$WORKSPACE_DIR"
        colcon build --packages-select ${BUILD_PACKAGES} --event-handlers console_direct+
    )
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo "✓ 相关包已重编译并重新加载环境"
else
    echo "✓ dual_arm_carry_task / franka_hardware 二进制为最新，无需重编译"
fi
echo ""

# 0.6 防呆：franka_moveit_config 的 launch/config 若更新，自动重编译并同步 install
echo "[0.6/4] 检查 franka_moveit_config 是否需要重编译..."
MOVEIT_INSTALL_LAUNCH="${WORKSPACE_DIR}/install/franka_moveit_config/share/franka_moveit_config/launch/sim_dual_moveit.launch.py"
MOVEIT_SRC_DIR="${WORKSPACE_DIR}/src/multipanda_ros2/franka_moveit_config"

needs_moveit_rebuild=0
if [ ! -f "$MOVEIT_INSTALL_LAUNCH" ]; then
    needs_moveit_rebuild=1
else
    if find "${MOVEIT_SRC_DIR}/launch" "${MOVEIT_SRC_DIR}/config" "${MOVEIT_SRC_DIR}/srdf" \
        -type f \( -name "*.py" -o -name "*.yaml" -o -name "*.yml" -o -name "*.srdf" -o -name "*.xacro" -o -name "CMakeLists.txt" -o -name "package.xml" \) \
        -newer "$MOVEIT_INSTALL_LAUNCH" | head -1 | grep -q .; then
        needs_moveit_rebuild=1
    fi
fi

if [ "$needs_moveit_rebuild" -eq 1 ]; then
    echo "⚠ 检测到 franka_moveit_config 源码更新（launch/config/srdf），自动执行增量编译..."
    (
        cd "$WORKSPACE_DIR"
        colcon build --packages-select franka_moveit_config --event-handlers console_direct+
    )
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo "✓ franka_moveit_config 已重编译并重新加载环境"
else
    echo "✓ franka_moveit_config 安装文件为最新，无需重编译"
fi
echo ""

# 1. 清理旧进程
echo "[1/4] 清理旧进程..."
cleanup_processes
echo ""

# 2. 生成/更新静态SRDF文件（确保碰撞规则正确）
echo "[2/4] 生成静态SRDF文件（确保碰撞规则正确加载）..."
SRDF_XACRO="${WORKSPACE_DIR}/install/franka_moveit_config/share/franka_moveit_config/srdf/dual_panda.srdf.xacro"
SRDF_OUTPUT="${WORKSPACE_DIR}/install/franka_moveit_config/share/franka_moveit_config/srdf/dual_panda_generated.srdf"
SRDF_SRC_OUTPUT="${WORKSPACE_DIR}/src/multipanda_ros2/franka_moveit_config/srdf/dual_panda_generated.srdf"

if [ -f "$SRDF_XACRO" ]; then
    xacro "$SRDF_XACRO" arm_id_1:=mj_left arm_id_2:=mj_right hand_1:=true hand_2:=true > "$SRDF_OUTPUT" 2>/dev/null
    cp "$SRDF_OUTPUT" "$SRDF_SRC_OUTPUT" 2>/dev/null || true
    
    COLLISION_RULES=$(grep -c "disable_collisions" "$SRDF_OUTPUT" 2>/dev/null || echo "0")
    echo "✓ SRDF已生成: $SRDF_OUTPUT"
    echo "  碰撞禁用规则数量: $COLLISION_RULES"
    
    # 记录诊断日志
    echo "=== SRDF 诊断 [$(date)] ===" > "$DIAG_LOG"
    echo "规则数量: $COLLISION_RULES" >> "$DIAG_LOG"
    
    # 验证关键规则并记录
    MISSING_RULES=0
    echo "关键规则验证:" >> "$DIAG_LOG"
    
    # 检查相邻关节规则
    for arm in mj_left mj_right; do
        for i in 0 1 2 3 4 5 6 7; do
            j=$((i+1))
            if [ $j -le 8 ]; then
                if grep -qE "${arm}_link${i}.*${arm}_link${j}|${arm}_link${j}.*${arm}_link${i}" "$SRDF_OUTPUT"; then
                    echo "  ✓ ${arm}_link${i} <-> ${arm}_link${j}" >> "$DIAG_LOG"
                else
                    echo "  ✗ ${arm}_link${i} <-> ${arm}_link${j} 缺失!" >> "$DIAG_LOG"
                    MISSING_RULES=$((MISSING_RULES+1))
                fi
            fi
        done
    done
    
    # base_link 规则
    for arm in mj_left mj_right; do
        if grep -qE "base_link.*${arm}_link0|${arm}_link0.*base_link" "$SRDF_OUTPUT"; then
            echo "  ✓ base_link <-> ${arm}_link0" >> "$DIAG_LOG"
        else
            echo "  ✗ base_link <-> ${arm}_link0 缺失!" >> "$DIAG_LOG"
            MISSING_RULES=$((MISSING_RULES+1))
        fi
    done
    
    # 终端输出
    if grep -q "mj_right_link0.*mj_right_link1\|mj_right_link1.*mj_right_link0" "$SRDF_OUTPUT"; then
        echo "  ✓ 关键碰撞规则 (link0↔link1) 已包含"
    else
        echo "  ⚠ 警告: 未找到 link0↔link1 碰撞规则!"
    fi
    
    if grep -q "base_link.*mj_left_link0\|mj_left_link0.*base_link" "$SRDF_OUTPUT"; then
        echo "  ✓ base_link 碰撞规则已包含"
    else
        echo "  ⚠ 警告: 未找到 base_link 碰撞规则!"
    fi
    
    if [ $MISSING_RULES -gt 0 ]; then
        echo "  ⚠ 缺失 $MISSING_RULES 条关键规则，详见: $DIAG_LOG"
    fi
else
    echo "✗ 错误: 找不到SRDF xacro文件: $SRDF_XACRO"
    exit 1
fi
echo ""

# 2.5 动态更新模型物体位置
if [[ "$TASK_MAIN_MODE" == "B" ]]; then
    echo "[2.5] (模式 B) 动态更新 MuJoCo 物体初始状态..."
    ROD_YAW_RAD=$(awk "BEGIN {print $ROD_INIT_YAW * 3.14159265 / 180}")
    OBJECTS_XML="${WORKSPACE_DIR}/src/multipanda_ros2/franka_description/mujoco/franka/objects.xml"
    INSTALL_OBJECTS_XML="${WORKSPACE_DIR}/install/franka_description/share/franka_description/mujoco/franka/objects.xml"
    
    # 动态匹配并替换 obj_rod_01 标签属性
    if [ -f "$OBJECTS_XML" ]; then
        sed -i -E "s/<body name=\"obj_rod_01\"[^>]*>/<body name=\"obj_rod_01\" pos=\"${ROD_INIT_X} ${ROD_INIT_Y} 0.02\" euler=\"0 0 ${ROD_YAW_RAD}\">/g" "$OBJECTS_XML"
        cp "$OBJECTS_XML" "$INSTALL_OBJECTS_XML" 2>/dev/null || true
        echo "  ✓ 铝棒生成位置已动态修改 -> pos: ${ROD_INIT_X} ${ROD_INIT_Y} 0.02, euler: 0 0 ${ROD_YAW_RAD}"
    fi
    echo ""
fi

# 3. 启动 MoveIt2 (自动启动 MuJoCo)
echo "[3/4] 启动 MoveIt2 和 MuJoCo..."
echo "注意: MoveIt2 会自动启动 MuJoCo 仿真"

MAX_SIM_RETRIES=2
SIM_READY=0
MOVEIT_PID=""
MOVEIT_LOG=""

for attempt in $(seq 1 $MAX_SIM_RETRIES); do
    MOVEIT_LOG="${LOG_DIR}/moveit_mujoco_${TIMESTAMP}_try${attempt}.log"
    echo "日志: $MOVEIT_LOG"
    echo "启动尝试: ${attempt}/${MAX_SIM_RETRIES}"

    ros2 launch franka_moveit_config sim_dual_moveit.launch.py > "$MOVEIT_LOG" 2>&1 &
    MOVEIT_PID=$!
    echo "进程 PID: $MOVEIT_PID"
    echo $MOVEIT_PID > "$PID_FILE"

    # 等待 MoveIt2 和 MuJoCo 完全启动
    echo "等待 MoveIt2 和 MuJoCo 完全启动 (10秒)..."
    for i in {1..10}; do
        sleep 1
        printf "\r  进度: %d/10 秒" $i
    done
    echo ""

    # 先检查 launch 进程是否还在运行
    if ! ps -p $MOVEIT_PID > /dev/null; then
        echo "✗ MoveIt2 启动失败，请检查日志: $MOVEIT_LOG"
        echo "最后20行日志:"
        tail -20 "$MOVEIT_LOG"
        if [ "$attempt" -lt "$MAX_SIM_RETRIES" ]; then
            echo "正在重试启动..."
            cleanup_processes
            source "${WORKSPACE_DIR}/install/setup.bash"
            continue
        fi
        exit 1
    fi

    # 再检查关键栈（防止 mujoco_node 崩溃但 launch 仍存活）
    echo "正在检查关键栈就绪状态（move_group + mujoco + controller_manager service）..."
    if wait_for_sim_stack_ready 12; then
        SIM_READY=1
        break
    fi

    echo "✗ 关键栈未就绪（/move_group + /mujoco_node(or /mujoco_ros2_control) + /controller_manager/list_controllers）"
    if grep -q "spin_some() called while already spinning" "$MOVEIT_LOG" 2>/dev/null; then
        echo "  检测到 MuJoCo 启动期崩溃: spin_some() called while already spinning"
    fi
    if grep -q "\[ERROR\] \[mujoco_node" "$MOVEIT_LOG" 2>/dev/null; then
        echo "  检测到 mujoco_node 进程异常退出"
    fi

    if [ "$attempt" -lt "$MAX_SIM_RETRIES" ]; then
        echo "正在自动重试启动 MuJoCo..."
        cleanup_processes
        source "${WORKSPACE_DIR}/install/setup.bash"
    fi
done

if [ "$SIM_READY" -ne 1 ]; then
    echo "✗ MoveIt2/MuJoCo 启动失败（已重试 ${MAX_SIM_RETRIES} 次）"
    echo "请检查日志: $MOVEIT_LOG"
    tail -40 "$MOVEIT_LOG"
    exit 1
fi

# 检查是否有碰撞错误
if grep -q "Start state appears to be in collision" "$MOVEIT_LOG" 2>/dev/null; then
    echo "⚠ 警告: 检测到碰撞状态问题，但继续执行..."
    echo "" >> "$DIAG_LOG"
    echo "=== 碰撞警告 ===" >> "$DIAG_LOG"
    grep -A2 "Start state appears to be in collision\|collision between" "$MOVEIT_LOG" >> "$DIAG_LOG" 2>/dev/null || true
fi

# 验证 MoveIt 是否正确加载了 SRDF
echo "验证 robot_description_semantic 参数..."
echo "" >> "$DIAG_LOG"
echo "=== 运行时验证 ===" >> "$DIAG_LOG"

# 等待 move_group 节点完全启动（最多等待10秒）
MOVE_GROUP_READY=0
for attempt in {1..10}; do
    if timeout 2s ros2 node list 2>/dev/null | grep -q "/move_group"; then
        MOVE_GROUP_READY=1
        break
    fi
    sleep 1
done

if [ "$MOVE_GROUP_READY" -eq 0 ]; then
    echo "  ⚠ move_group 节点未在10秒内就绪，跳过参数验证（不阻塞启动）"
    echo "robot_description_semantic: 跳过验证（move_group未就绪）" >> "$DIAG_LOG"
else
    SRDF_OK=0
    SRDF_PARAM=""

    # 参数服务可能刚启动，做短超时重试，避免卡住脚本
    for attempt in {1..5}; do
        SRDF_PARAM=$(timeout 2s ros2 param get /move_group robot_description_semantic 2>/dev/null | head -1 || true)
        if echo "$SRDF_PARAM" | grep -q "String value"; then
            SRDF_OK=1
            break
        fi
        sleep 1
    done

    if [ "$SRDF_OK" -eq 1 ]; then
        RUNTIME_RULES=$(timeout 2s ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -c "disable_collisions" || echo "0")
        echo "  ✓ robot_description_semantic 已加载 ($RUNTIME_RULES 条规则)"
        echo "robot_description_semantic: 已加载 ($RUNTIME_RULES 条规则)" >> "$DIAG_LOG"
    else
        echo "  ⚠ 无法验证 robot_description_semantic 参数（参数服务未就绪或响应超时）"
        echo "robot_description_semantic: 验证失败（服务超时）" >> "$DIAG_LOG"
    fi
fi

echo ""
echo "✓ MoveIt2 和 MuJoCo 已启动（关键节点检测通过）"
echo ""

if [ "$ADAPTIVE_COMPLIANCE_ENABLED" = "true" ]; then
    echo "[3.5/4] 检查自适应阻抗服务可用性..."
    RESOLVED_ADAPTIVE_SERVICE="$(resolve_adaptive_impedance_service "$ADAPTIVE_IMPEDANCE_SERVICE" || true)"

    if [ -z "$RESOLVED_ADAPTIVE_SERVICE" ]; then
        MMC_SPAWNER_LOG="${LOG_DIR}/sim_multi_mode_spawner_${TIMESTAMP}.log"
        echo "⚠ 未检测到阻抗参数服务，尝试加载 sim_multi_mode_controller..."
        if timeout 30s ros2 run controller_manager spawner sim_multi_mode_controller \
            --controller-manager-timeout 20 --service-call-timeout 20 > "$MMC_SPAWNER_LOG" 2>&1; then
            sleep 1
            RESOLVED_ADAPTIVE_SERVICE="$(resolve_adaptive_impedance_service "$ADAPTIVE_IMPEDANCE_SERVICE" || true)"
            if [ -n "$RESOLVED_ADAPTIVE_SERVICE" ]; then
                echo "✓ sim_multi_mode_controller 加载成功，并检测到阻抗参数服务"
            else
                echo "⚠ sim_multi_mode_controller 已加载，但仍未发现阻抗参数服务"
                echo "  spawner日志: $MMC_SPAWNER_LOG"
            fi
        else
            echo "⚠ sim_multi_mode_controller 加载失败，无法启用自适应阻抗服务"
            echo "  spawner日志: $MMC_SPAWNER_LOG"
            tail -20 "$MMC_SPAWNER_LOG" 2>/dev/null || true
        fi
    fi

    if [ -z "$RESOLVED_ADAPTIVE_SERVICE" ]; then
        CONTROLLER_SNAPSHOT=""
        for _ in 1 2 3; do
            CONTROLLER_SNAPSHOT="$(timeout 4s ros2 control list_controllers 2>/dev/null || true)"
            if [ -n "$CONTROLLER_SNAPSHOT" ]; then
                break
            fi
            sleep 1
        done
        MMC_INACTIVE=0
        TRAJ_ACTIVE=0

        if echo "$CONTROLLER_SNAPSHOT" | grep -Eq 'sim_multi_mode_controller[[:space:]].*inactive'; then
            MMC_INACTIVE=1
        fi
        if echo "$CONTROLLER_SNAPSHOT" | grep -Eq 'dual_panda_arm_controller[[:space:]].*active'; then
            TRAJ_ACTIVE=1
        fi

        if [ "$MMC_INACTIVE" -eq 1 ] && [ "$TRAJ_ACTIVE" -eq 1 ]; then
            echo "⚠ 诊断: sim_multi_mode_controller 已加载但未激活，dual_panda_arm_controller 仍在 active。"
            echo "⚠ 根因: MuJoCo 硬件层禁止在未先停掉位置控制器时直接切到 effort 控制模式。"
            echo "  手动切换参考: ros2 control switch_controllers --deactivate dual_panda_arm_controller --activate sim_multi_mode_controller"

            if [[ "$ADAPTIVE_MMC_FORCE_SWITCH" == "1" || "$ADAPTIVE_MMC_FORCE_SWITCH" == "true" || "$ADAPTIVE_MMC_FORCE_SWITCH" == "yes" ]]; then
                echo "⚠ ADAPTIVE_MMC_FORCE_SWITCH 已开启，尝试强制切换到 sim_multi_mode_controller..."
                if timeout 20s ros2 control switch_controllers --deactivate dual_panda_arm_controller --activate sim_multi_mode_controller; then
                    sleep 1
                    RESOLVED_ADAPTIVE_SERVICE="$(resolve_adaptive_impedance_service "$ADAPTIVE_IMPEDANCE_SERVICE" || true)"
                    if [ -n "$RESOLVED_ADAPTIVE_SERVICE" ]; then
                        echo "✓ 强制切换成功，自适应阻抗服务已上线: ${RESOLVED_ADAPTIVE_SERVICE}"
                        echo "⚠ 注意: dual_panda_arm_controller 已被停用，MoveIt 轨迹执行可能失败。"
                    fi
                else
                    echo "⚠ 强制切换失败，保持降级策略"
                fi
            fi
        fi
    fi

    if [ -n "$RESOLVED_ADAPTIVE_SERVICE" ]; then
        ADAPTIVE_IMPEDANCE_SERVICE="$RESOLVED_ADAPTIVE_SERVICE"
        echo "✓ 自适应阻抗服务已就绪: ${ADAPTIVE_IMPEDANCE_SERVICE}"
    else
        if [ "$ADAPTIVE_IMPEDANCE_SERVICE" = "auto" ]; then
            echo "⚠ 未检测到 dual_cartesian_impedance_controller 参数服务"
        else
            echo "⚠ 指定的阻抗服务不可用: ${ADAPTIVE_IMPEDANCE_SERVICE}"
        fi
        echo "⚠ 已明确降级: 关闭自适应柔顺闭环，避免 mode4 名义启用但实际失效"
        ADAPTIVE_COMPLIANCE_ENABLED="false"
        ADAPTIVE_IMPEDANCE_SERVICE="/mj_left_and_mj_right/dual_cartesian_impedance_controller/parameters"
    fi
else
    if [ "$ADAPTIVE_IMPEDANCE_SERVICE" = "auto" ]; then
        ADAPTIVE_IMPEDANCE_SERVICE="/mj_left_and_mj_right/dual_cartesian_impedance_controller/parameters"
    fi
fi
echo "自适应柔顺最终状态: ${ADAPTIVE_COMPLIANCE_ENABLED}"
echo "自适应阻抗服务: ${ADAPTIVE_IMPEDANCE_SERVICE}"
echo ""

if [[ "$TASK_MAIN_MODE" == "A" ]]; then
    ENABLE_SENSOR_GUI="${ENABLE_SENSOR_GUI:-false}"
    ENABLE_EXPERIMENT_RECORDER="${ENABLE_EXPERIMENT_RECORDER:-false}"

    # 4. 启动任务节点
    echo "[4/5] 启动双臂搬运任务节点 ($CONTROL_MODE 模式)..."
    sleep 5

    TASK_LOG="${LOG_DIR}/dual_arm_task_${TIMESTAMP}.log"
    echo "日志: $TASK_LOG"

    ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
        control_mode:=${CONTROL_MODE} \
        approach_height:=${APPROACH_HEIGHT} \
        grasp_height:=${GRASP_HEIGHT} \
        lift_height:=${LIFT_HEIGHT} \
        enable_rotate:=${ENABLE_ROTATE} \
        enable_latency_monitor:=true \
        latency_topic:=/joint_states \
        latency_task_status_topic:=/task_status \
        latency_target_ms:=100.0 \
        latency_result_dir:=${RESULT_REPORT_DIR} \
        latency_result_prefix:=comm_latency_${TIMESTAMP} \
        enable_task_metrics_monitor:=true \
        metrics_joint_state_topic:=/joint_states \
        metrics_task_stage_topic:=/task_stage \
        metrics_task_status_topic:=/task_status \
        metrics_base_frame:=base_link \
        metrics_left_frame:=mj_left_hand \
        metrics_right_frame:=mj_right_hand \
        metrics_sync_target_mm:=5.0 \
        metrics_ee_target_mm:=2.0 \
        metrics_enable_rotate:=${ENABLE_ROTATE} \
        metrics_descend_place_z:=${GRASP_HEIGHT} \
        metrics_descend_shift_x:=999.0 \
        metrics_result_dir:=${RESULT_REPORT_DIR} \
        metrics_result_prefix:=task_metrics_${TIMESTAMP}_${CONTROL_MODE} \
        enable_adaptive_compliance_supervisor:=${ADAPTIVE_COMPLIANCE_ENABLED} \
        adaptive_left_wrench_topic:=/force_torque_sensor_broadcaster_left/wrench \
        adaptive_right_wrench_topic:=/force_torque_sensor_broadcaster_right/wrench \
        adaptive_task_stage_topic:=/task_stage \
        adaptive_task_status_topic:=/task_status \
        adaptive_impedance_service:=${ADAPTIVE_IMPEDANCE_SERVICE} \
        adaptive_params_topic:=/adaptive_impedance_params \
        adaptive_update_rate_hz:=20.0 \
        adaptive_nominal_stiffness:=${ADAPTIVE_NOMINAL_STIFFNESS} \
        adaptive_min_stiffness:=${ADAPTIVE_MIN_STIFFNESS} \
        adaptive_force_gain:=${ADAPTIVE_FORCE_GAIN} \
        adaptive_imbalance_gain:=${ADAPTIVE_IMBALANCE_GAIN} \
        adaptive_active_stages:=${ADAPTIVE_ACTIVE_STAGES} > "$TASK_LOG" 2>&1 &
    TASK_PID=$!
    echo "进程 PID: $TASK_PID"
    echo $TASK_PID >> "$PID_FILE"

    sleep 3

    # 检查任务节点是否启动
    if ! ps -p $TASK_PID > /dev/null; then
        echo "✗ 任务节点启动失败，请检查日志: $TASK_LOG"
        exit 1
    fi

    echo ""

    # 5. 启动实验数据记录器（默认开启，支持无GUI）
    RECORDER_PID=""
    if [ "$ENABLE_EXPERIMENT_RECORDER" = "true" ]; then
        echo "[5/6] 启动实验数据记录器..."
        EXPERIMENT_DATA_DIR="${RESULT_DATA_DIR}" /usr/bin/python3 "${WORKSPACE_DIR}/experiment_data_recorder.py" "$CONTROL_MODE" > "${LOG_DIR}/experiment_recorder_${TIMESTAMP}.log" 2>&1 &
        RECORDER_PID=$!
        echo "进程 PID: $RECORDER_PID"
        echo $RECORDER_PID >> "$PID_FILE"
    else
        echo "[5/6] 已跳过实验数据记录器（ENABLE_EXPERIMENT_RECORDER=${ENABLE_EXPERIMENT_RECORDER}）"
        echo "      如需自动生成 experiment_results/data CSV，请使用: ENABLE_EXPERIMENT_RECORDER=true ./start_dual_arm_task.sh"
    fi

    echo ""

    # 6. 启动传感器监控及控制参数 GUI（可选）
    GUI_PID=""
    if [ "$ENABLE_SENSOR_GUI" = "true" ]; then
        echo "[6/6] 启动传感器可视化与参数调优 GUI..."
        EXPERIMENT_DATA_DIR="${RESULT_DATA_DIR}" python3 "${WORKSPACE_DIR}/sensor_dashboard.py" "$CONTROL_MODE" > "${LOG_DIR}/sensor_gui_${TIMESTAMP}.log" 2>&1 &
        GUI_PID=$!
        echo "进程 PID: $GUI_PID"
        echo $GUI_PID >> "$PID_FILE"
    else
        echo "[6/6] 已跳过传感器 GUI（ENABLE_SENSOR_GUI=${ENABLE_SENSOR_GUI}）"
        echo "      如需启用可视化，请使用: ENABLE_SENSOR_GUI=true ./start_dual_arm_task.sh"
    fi

    echo ""
    echo "========================================"
    echo "  ✓ 所有固定搬运组件 (Mode A) 及 GUI 已启动"
    echo "========================================"
    echo ""
    echo "组件状态:"
    echo "  • MoveIt2 + MuJoCo: PID $MOVEIT_PID"
    echo "  • 任务节点: PID $TASK_PID"
    if [ -n "$RECORDER_PID" ]; then
        echo "  • 数据记录器: PID $RECORDER_PID"
    else
        echo "  • 数据记录器: 已跳过"
    fi
    if [ -n "$GUI_PID" ]; then
        echo "  • 可视化终端: PID $GUI_PID"
    else
        echo "  • 可视化终端: 已跳过"
    fi
    echo ""
    echo "日志位置:"
    echo "  • MoveIt/MuJoCo: $MOVEIT_LOG"
    echo "  • 任务节点: $TASK_LOG"
    echo "  • 数据记录器: ${LOG_DIR}/experiment_recorder_${TIMESTAMP}.log"
    echo "  • 通信时延报告: ${RESULT_REPORT_DIR}/comm_latency_${TIMESTAMP}_*_summary.md"
    echo "  • 指标报告: ${RESULT_REPORT_DIR}/task_metrics_${TIMESTAMP}_${CONTROL_MODE}_*_summary.md"
    echo "  • 图表输出目录: ${RESULT_PLOT_DIR}"
    echo "  • 诊断日志: $DIAG_LOG"
    echo ""
    echo "所有组件已顺利启动！机器开始全自动执行固定预编任务流程..."
    echo "========================================"
    echo ""

    # 监控 GUI 进程健康状态（不作为任务完成判据）
    if [ -n "$RECORDER_PID" ] && ! ps -p $RECORDER_PID > /dev/null 2>&1; then
        echo "⚠ 实验数据记录器进程启动后立即退出（请检查日志）"
    fi

    if [ -n "$GUI_PID" ] && ! ps -p $GUI_PID > /dev/null 2>&1; then
        echo "⚠ 传感器GUI进程启动后立即退出（不影响主任务执行）"
    fi

    echo "等待任务终态（DONE / ERROR），最长240秒..."
    wait_for_task_terminal_state "$TASK_LOG" "$TASK_PID" 240
    TASK_STATE_CODE=$?

    if [ "$TASK_STATE_CODE" -eq 0 ]; then
        echo "✅ 检测到任务完成状态(DONE)"
    elif [ "$TASK_STATE_CODE" -eq 2 ]; then
        echo "❌ 检测到任务失败状态(ERROR)"
    elif [ "$TASK_STATE_CODE" -eq 3 ]; then
        echo "❌ 任务节点进程提前退出，请检查日志: $TASK_LOG"
    else
        echo "⚠ 240秒内未检测到任务终态，可能仍在执行中"
    fi

    echo ""
    echo "========================================"
    echo "🎓 任务流程结束，开始读取整周期CSV数据并绘制高精度分析图..."
    echo "========================================"
    python3 "${WORKSPACE_DIR}/plot_experiment_results.py" \
        --data-dir "${RESULT_DATA_DIR}" \
        --plot-dir "${RESULT_PLOT_DIR}" || true

    LATENCY_SUMMARY_MD=$(ls -t "${RESULT_REPORT_DIR}/comm_latency_${TIMESTAMP}_"*_summary.md 2>/dev/null | head -1)
    LATENCY_SUMMARY_CSV=$(ls -t "${RESULT_REPORT_DIR}/comm_latency_${TIMESTAMP}_"*_summary.csv 2>/dev/null | head -1)
    METRICS_SUMMARY_MD=$(ls -t "${RESULT_REPORT_DIR}/task_metrics_${TIMESTAMP}_${CONTROL_MODE}_"*_summary.md 2>/dev/null | head -1)
    METRICS_SUMMARY_CSV=$(ls -t "${RESULT_REPORT_DIR}/task_metrics_${TIMESTAMP}_${CONTROL_MODE}_"*_summary.csv 2>/dev/null | head -1)
    if [ -n "$LATENCY_SUMMARY_MD" ]; then
        echo ""
        echo "通信时延报告已生成:"
        echo "  • 表格报告: $LATENCY_SUMMARY_MD"
        if [ -n "$LATENCY_SUMMARY_CSV" ]; then
            echo "  • 数据汇总: $LATENCY_SUMMARY_CSV"
        fi
    else
        echo ""
        echo "⚠ 未检测到通信时延报告，请检查任务日志: $TASK_LOG"
    fi

    if [ -n "$METRICS_SUMMARY_MD" ]; then
        echo ""
        echo "任务量化指标报告已生成:"
        echo "  • 表格报告: $METRICS_SUMMARY_MD"
        if [ -n "$METRICS_SUMMARY_CSV" ]; then
            echo "  • 数据汇总: $METRICS_SUMMARY_CSV"
        fi
    else
        echo ""
        echo "⚠ 未检测到任务指标报告，请检查任务日志: $TASK_LOG"
    fi

    echo ""
    echo "⭐⭐图表计算/输出步骤结束。⭐⭐"
    echo "请前往 ${RESULTS_DIR} 文件夹查看本轮数据与图表。"

else
    echo "[4/4] 视觉感知与动态搬运模式 (Mode B) 环境就绪..."
    echo "注意: 模式B有独立的研究路线，旧有的大屏绘图仪表盘(GUI)及固定C++工作流任务已被隔离。"
    
    # 还可以顺便唤起你刚刚创建的视觉桥接核心
    echo "正在拉起自定义顶置相机与视觉处理沙盒(vision_core.py)..."
    python3 "${WORKSPACE_DIR}/vision_core.py" > "${LOG_DIR}/vision_core_${TIMESTAMP}.log" 2>&1 &
    VISION_PID=$!
    echo $VISION_PID >> "$PID_FILE"

    echo ""
    echo "========================================"
    echo "  ✓ 基础平台(MuJoCo/RViz/MoveIt2) 和 相机节点已挂载"
    echo "========================================"
    echo "当前启动节点:"
    echo "  • MoveIt2 + MuJoCo: PID $MOVEIT_PID"
    echo "  • 视觉流桥接节点: PID $VISION_PID"
    echo ""
    echo "  >> 请关注 /overhead_camera/image_raw ROS话题获取实时上帝视角。"
    echo "  >> C++硬编排节点已挂起，此时可以通过新架构或新节点发布机械臂目标轨迹。"
    echo "========================================"
fi

echo ""
echo "==> 现在，你可以安全地按 Ctrl+C 彻底关闭所有后台 ROS 仿真进程了！ <=="
echo ""

# 继续等待所有进程(Mujoco/Moveit等)直到用户手动中断
wait
