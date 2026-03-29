#!/bin/bash

# 双臂搬运任务 - 完整启动脚本
# 功能: 生成SRDF → 启动 MoveIt2 (自动启动MuJoCo) → 等待初始化 → 启动任务节点
# 更新: 2026-01-31 - 增强SRDF诊断，添加ACM验证

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${WORKSPACE_DIR}/launch_logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DIAG_LOG="${LOG_DIR}/diagnosis_${TIMESTAMP}.log"

# 保存 PID 到文件以便停止
PID_FILE="${WORKSPACE_DIR}/.running_pids.txt"

# 创建日志目录
mkdir -p "$LOG_DIR"

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

# 捕获 Ctrl+C 信号
trap 'echo ""; echo "检测到 Ctrl+C，正在停止所有进程..."; cleanup_processes; exit 0' INT TERM

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
    esac
fi

echo ""
echo "✅ 最终设定的运行配置: ===[ 大模式: ${TASK_MAIN_MODE} | 核心策略: ${CONTROL_MODE} ]==="
echo ""

# 旋转阶段开关（默认启用）。可通过环境变量覆盖：ENABLE_ROTATE=false ./start_dual_arm_task.sh
ENABLE_ROTATE="${ENABLE_ROTATE:-true}"
echo "旋转阶段开关: ${ENABLE_ROTATE}"

echo "日志目录: $LOG_DIR"
echo "诊断日志: $DIAG_LOG"
echo ""

# 0. Source 工作空间
echo "[0/4] 加载ROS2工作空间环境..."
source "${WORKSPACE_DIR}/install/setup.bash"
echo "✓ 环境已加载"
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

MOVEIT_LOG="${LOG_DIR}/moveit_mujoco_${TIMESTAMP}.log"
echo "日志: $MOVEIT_LOG"

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

# 检查进程是否还在运行
if ! ps -p $MOVEIT_PID > /dev/null; then
    echo "✗ MoveIt2 启动失败，请检查日志: $MOVEIT_LOG"
    echo "最后20行日志:"
    tail -20 "$MOVEIT_LOG"
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
echo "✓ MoveIt2 和 MuJoCo 已启动"
echo ""

if [[ "$TASK_MAIN_MODE" == "A" ]]; then
    # 4. 启动任务节点
    echo "[4/5] 启动双臂搬运任务节点 ($CONTROL_MODE 模式)..."
    sleep 5

    TASK_LOG="${LOG_DIR}/dual_arm_task_${TIMESTAMP}.log"
    echo "日志: $TASK_LOG"

    ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
        control_mode:=${CONTROL_MODE} \
        approach_height:=0.28 \
        grasp_height:=0.13 \
        lift_height:=0.40 \
        enable_rotate:=${ENABLE_ROTATE} > "$TASK_LOG" 2>&1 &
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

    # 5. 启动传感器监控及控制参数 GUI
    echo "[5/5] 启动传感器可视化与参数调优 GUI..."
    python3 "${WORKSPACE_DIR}/sensor_dashboard.py" "$CONTROL_MODE" > "${LOG_DIR}/sensor_gui_${TIMESTAMP}.log" 2>&1 &
    GUI_PID=$!
    echo "进程 PID: $GUI_PID"
    echo $GUI_PID >> "$PID_FILE"

    echo ""
    echo "========================================"
    echo "  ✓ 所有固定搬运组件 (Mode A) 及 GUI 已启动"
    echo "========================================"
    echo ""
    echo "组件状态:"
    echo "  • MoveIt2 + MuJoCo: PID $MOVEIT_PID"
    echo "  • 任务节点: PID $TASK_PID"
    echo "  • 可视化终端: PID $GUI_PID"
    echo ""
    echo "日志位置:"
    echo "  • MoveIt/MuJoCo: $MOVEIT_LOG"
    echo "  • 任务节点: $TASK_LOG"
    echo "  • 诊断日志: $DIAG_LOG"
    echo ""
    echo "所有组件已顺利启动！机器开始全自动执行固定预编任务流程..."
    echo "========================================"
    echo ""

    # 监控 GUI 进程，直到其因收到任务 DONE 状态发布而自动安全退出 (耗时约几十秒)
    wait $GUI_PID

    echo ""
    echo "========================================"
    echo "🎓 监测到全流程搬运已完成，开始读取整周期CSV数据并绘制高精度分析图..."
    echo "========================================"
    python3 "${WORKSPACE_DIR}/plot_experiment_results.py"

    echo ""
    echo "⭐⭐图表计算/输出完毕！任务圆满结束。⭐⭐"
    echo "请前往 mujoco_franka/franka_ws/experiment_data 文件夹查看此轮的对比结果图。"

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
