from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
import yaml


def launch_setup(context, *args, **kwargs):
    """延迟设置以便动态加载参数"""

    def _as_bool(value: str) -> bool:
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')
    
    # 获取 MoveIt 配置
    franka_moveit_config = get_package_share_directory('franka_moveit_config')
    franka_description = get_package_share_directory('franka_description')
    
    # 生成 URDF（从 xacro 处理）
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(franka_description, 'robots', 'sim', 'dual_panda_arm_sim.urdf.xacro'),
        ' arm_id_1:=mj_left',
        ' arm_id_2:=mj_right',
        ' hand_1:=true',
        ' hand_2:=true',
        ' use_fake_hardware:=false',
        ' fake_sensor_commands:=false'
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # 生成 SRDF（从 xacro 处理）
    robot_description_semantic_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(franka_moveit_config, 'srdf', 'dual_panda.srdf.xacro'),
        ' arm_id_1:=mj_left',
        ' arm_id_2:=mj_right',
        ' hand_1:=true',
        ' hand_2:=true'
    ])
    
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }
    
    # 加载运动学配置
    kinematics_yaml_path = os.path.join(
        franka_moveit_config, 'config', 'kinematics.yaml'
    )
    
    # 读取 kinematics.yaml 内容
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_config = yaml.safe_load(file)
    
    # 加载关节限制配置（加速度限制等）
    # 修复：RobotModelLoader 需要 robot_description_planning 参数
    # 才能将 joint_limits.yaml 中的加速度限制写入 JointModel::VariableBounds，
    # 否则 TOTG 会退回默认 1 rad/s² 导致所有关节一刀切、协调性差。
    joint_limits_yaml_path = os.path.join(
        franka_moveit_config, 'config', 'joint_limits.yaml'
    )
    with open(joint_limits_yaml_path, 'r') as file:
        joint_limits_config = yaml.safe_load(file)
    joint_limits_param = {
        'robot_description_planning': joint_limits_config
    }
    
    # 任务节点
    task_node = Node(
        package='dual_arm_carry_task',
        executable='dual_arm_carry_task_node',
        name='dual_arm_carry_task',
        output='screen',
        parameters=[
            robot_description,  # 传入完整的 URDF
            robot_description_semantic,  # 传入生成的 SRDF 内容
            kinematics_config,  # 加载运动学求解器配置（已解析的字典）
            joint_limits_param,  # 关节限制（加速度等）→ RobotModelLoader 解析
            {
                'left_arm_group': LaunchConfiguration('left_arm_group'),
                'right_arm_group': LaunchConfiguration('right_arm_group'),
                'dual_arm_group': LaunchConfiguration('dual_arm_group'),
                'approach_height': LaunchConfiguration('approach_height'),
                'grasp_height': LaunchConfiguration('grasp_height'),
                'lift_height': LaunchConfiguration('lift_height'),
                'enable_rotate': LaunchConfiguration('enable_rotate'),
                'control_mode': LaunchConfiguration('control_mode'),
                'use_vision': LaunchConfiguration('use_vision'),
                'rod_target_x': LaunchConfiguration('rod_target_x'),
                'rod_target_y': LaunchConfiguration('rod_target_y'),
                'rod_target_yaw_deg': LaunchConfiguration('rod_target_yaw_deg'),
                'rod_pose_source': LaunchConfiguration('rod_pose_source'),
                'expected_run_id': LaunchConfiguration('expected_run_id'),
                'prepare_only': LaunchConfiguration('prepare_only'),
                'a1_truth_topic': LaunchConfiguration('a1_truth_topic'),
                'a1_truth_timeout_sec': LaunchConfiguration('a1_truth_timeout_sec'),
                'p2a_check': LaunchConfiguration('p2a_check'),
                'p2a_workspace_x_min': LaunchConfiguration('p2a_workspace_x_min'),
                'p2a_workspace_x_max': LaunchConfiguration('p2a_workspace_x_max'),
                'p2a_workspace_y_min': LaunchConfiguration('p2a_workspace_y_min'),
                'p2a_workspace_y_max': LaunchConfiguration('p2a_workspace_y_max'),
                'p2a_workspace_source': LaunchConfiguration('p2a_workspace_source'),
                'p2a_support_z': LaunchConfiguration('p2a_support_z'),
                'p2a_support_plane': LaunchConfiguration('p2a_support_plane'),
                'p2a_support_object_id': LaunchConfiguration('p2a_support_object_id'),
                'p2a_ground_negative': LaunchConfiguration('p2a_ground_negative'),
                'p2a_hand_aabb_substitution': LaunchConfiguration('p2a_hand_aabb_substitution'),
                'p2a_hand_negative': LaunchConfiguration('p2a_hand_negative'),
                'p2a_support_depth_m': LaunchConfiguration('p2a_support_depth_m'),
                'p2b_check': LaunchConfiguration('p2b_check'),
                'p2b_planner_id': LaunchConfiguration('p2b_planner_id'),
                'p2b_planning_time_sec': LaunchConfiguration('p2b_planning_time_sec'),
                'p2b_planning_attempts': LaunchConfiguration('p2b_planning_attempts'),
                'p2b_max_joint_step_rad': LaunchConfiguration('p2b_max_joint_step_rad'),
                'p2b_midpath_negative': LaunchConfiguration('p2b_midpath_negative'),
                'p2a_grasp_offset': LaunchConfiguration('p2a_grasp_offset'),
                'p2a_eef_base_yaw_deg': LaunchConfiguration('p2a_eef_base_yaw_deg'),
                'use_sim_time': True,
            }
        ]
    )

    monitor_enabled = _as_bool(LaunchConfiguration('enable_latency_monitor').perform(context))
    metrics_monitor_enabled = _as_bool(LaunchConfiguration('enable_task_metrics_monitor').perform(context))
    adaptive_supervisor_enabled = _as_bool(LaunchConfiguration('enable_adaptive_compliance_supervisor').perform(context))
    adaptive_active_stages = [
        stage.strip().upper()
        for stage in LaunchConfiguration('adaptive_active_stages').perform(context).split(',')
        if stage.strip()
    ]
    nodes = [task_node]

    if monitor_enabled:
        latency_monitor_node = Node(
            package='dual_arm_carry_task',
            executable='communication_latency_monitor_node',
            name='communication_latency_monitor',
            output='screen',
            parameters=[
                {
                    'latency_topic': LaunchConfiguration('latency_topic'),
                    'task_status_topic': LaunchConfiguration('latency_task_status_topic'),
                    'target_latency_ms': LaunchConfiguration('latency_target_ms'),
                    'result_dir': LaunchConfiguration('latency_result_dir'),
                    'result_prefix': LaunchConfiguration('latency_result_prefix'),
                    'stop_on_done': True,
                    'progress_log_interval_sec': 5,
                    'use_sim_time': True,
                }
            ]
        )
        nodes.append(latency_monitor_node)

    if metrics_monitor_enabled:
        task_metrics_monitor_node = Node(
            package='dual_arm_carry_task',
            executable='task_metrics_monitor_node',
            name='task_metrics_monitor',
            output='screen',
            parameters=[
                {
                    'joint_state_topic': LaunchConfiguration('metrics_joint_state_topic'),
                    'task_stage_topic': LaunchConfiguration('metrics_task_stage_topic'),
                    'task_status_topic': LaunchConfiguration('metrics_task_status_topic'),
                    'base_frame': LaunchConfiguration('metrics_base_frame'),
                    'left_frame': LaunchConfiguration('metrics_left_frame'),
                    'right_frame': LaunchConfiguration('metrics_right_frame'),
                    'sync_target_mm': LaunchConfiguration('metrics_sync_target_mm'),
                    'ee_target_mm': LaunchConfiguration('metrics_ee_target_mm'),
                    'enable_rotate': LaunchConfiguration('metrics_enable_rotate'),
                    'descend_place_z': LaunchConfiguration('metrics_descend_place_z'),
                    'descend_shift_x': LaunchConfiguration('metrics_descend_shift_x'),
                    'result_dir': LaunchConfiguration('metrics_result_dir'),
                    'result_prefix': LaunchConfiguration('metrics_result_prefix'),
                    'stop_on_done': True,
                    'sample_period_ms': 20,
                    'progress_log_interval_sec': 5,
                    'use_sim_time': True,
                }
            ]
        )
        nodes.append(task_metrics_monitor_node)

    if adaptive_supervisor_enabled:
        adaptive_supervisor_node = Node(
            package='dual_arm_carry_task',
            executable='adaptive_compliance_supervisor_node',
            name='adaptive_compliance_supervisor',
            output='screen',
            parameters=[
                {
                    'left_wrench_topic': LaunchConfiguration('adaptive_left_wrench_topic'),
                    'right_wrench_topic': LaunchConfiguration('adaptive_right_wrench_topic'),
                    'task_stage_topic': LaunchConfiguration('adaptive_task_stage_topic'),
                    'task_status_topic': LaunchConfiguration('adaptive_task_status_topic'),
                    'impedance_service': LaunchConfiguration('adaptive_impedance_service'),
                    'adaptive_params_topic': LaunchConfiguration('adaptive_params_topic'),
                    'update_rate_hz': LaunchConfiguration('adaptive_update_rate_hz'),
                    'nominal_translation_stiffness': LaunchConfiguration('adaptive_nominal_stiffness'),
                    'min_translation_stiffness': LaunchConfiguration('adaptive_min_stiffness'),
                    'force_gain': LaunchConfiguration('adaptive_force_gain'),
                    'imbalance_gain': LaunchConfiguration('adaptive_imbalance_gain'),
                    'active_stages': adaptive_active_stages,
                    'stop_on_done': True,
                    'use_sim_time': True,
                }
            ]
        )
        nodes.append(adaptive_supervisor_node)

    return nodes


def generate_launch_description():
    """生成启动描述"""
    
    # 声明参数
    left_arm_group_arg = DeclareLaunchArgument(
        'left_arm_group',
        default_value='mj_left_arm',
        description='左臂 MoveGroup 名称'
    )
    
    right_arm_group_arg = DeclareLaunchArgument(
        'right_arm_group',
        default_value='mj_right_arm',
        description='右臂 MoveGroup 名称'
    )
    
    dual_arm_group_arg = DeclareLaunchArgument(
        'dual_arm_group',
        default_value='dual_panda',
        description='双臂协同 MoveGroup 名称'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='compliant',
        description='控制模式: rigid, compliant, opt_1, etc.'
    )
    
    approach_height_arg = DeclareLaunchArgument(
        'approach_height',
        default_value='0.28',
        description='接近物体时的高度 (m)'
    )
    
    grasp_height_arg = DeclareLaunchArgument(
        'grasp_height',
        default_value='0.13',
        description='抓取物体时的高度 (m)'
    )
    
    lift_height_arg = DeclareLaunchArgument(
        'lift_height',
        default_value='0.40',
        description='抬起物体后的高度 (m)'
    )

    enable_rotate_arg = DeclareLaunchArgument(
        'enable_rotate',
        default_value='true',
        description='是否执行ROTATE阶段 (true/false)'
    )

    use_vision_arg = DeclareLaunchArgument(
        'use_vision',
        default_value='false',
        description='启用视觉动态抓取模式 (Mode B)'
    )

    rod_target_x_arg = DeclareLaunchArgument(
        'rod_target_x',
        default_value='0.40',
        description='Mode B 目标放置 X 坐标 (m)'
    )

    rod_target_y_arg = DeclareLaunchArgument(
        'rod_target_y',
        default_value='-0.20',
        description='Mode B 目标放置 Y 坐标 (m)'
    )

    rod_target_yaw_deg_arg = DeclareLaunchArgument(
        'rod_target_yaw_deg',
        default_value='90.0',
        description='Mode B 目标放置偏航角 (度)'
    )

    # --- A1 P1b: rod pose source (defaults preserve the original A0 entry) ---
    rod_pose_source_arg = DeclareLaunchArgument(
        'rod_pose_source',
        default_value='static',
        description='杆位姿来源: static(A0 固定) / vision(原视觉入口) / a1_truth(A1 真值, 仅准备)'
    )

    expected_run_id_arg = DeclareLaunchArgument(
        'expected_run_id',
        default_value='',
        description='A1 真值要求的 run_id；rod_pose_source=a1_truth 时必填且必须匹配'
    )

    prepare_only_arg = DeclareLaunchArgument(
        'prepare_only',
        default_value='false',
        description='真值冻结 + 场景同步后停在 PREPARED，不进入运动（本轮 A1 强制为 true）'
    )

    a1_truth_topic_arg = DeclareLaunchArgument(
        'a1_truth_topic',
        default_value='/a1_rod_pose_truth',
        description='A1 真值话题（与 /detected_rod_pose 分离）'
    )

    a1_truth_timeout_sec_arg = DeclareLaunchArgument(
        'a1_truth_timeout_sec',
        default_value='20.0',
        description='等待有效 A1 真值的超时（秒），超时即失败且不回退硬编码位姿'
    )

    # --- A1 P2a: static sample validity (geometry / dual-arm IK / collision) ---
    # Off by default: the P1b prepare behaviour and the A0 entry are unchanged.
    p2a_check_arg = DeclareLaunchArgument(
        'p2a_check',
        default_value='false',
        description='A1 P2a: 在 PREPARED 之前对冻结真值做几何/双臂 IK/组合碰撞静态检查（不运动）'
    )

    # The support surface in this scene is an unbounded MuJoCo `plane`
    # (dual_scene.xml `floor`, no `pos`), so these bounds are a configured
    # research envelope, NOT a physical table edge. Defaults were derived from
    # the P1a centre range in config/a1_scene_sampling.yaml inflated by the
    # largest footprint extent within its +/-15 deg yaw range, plus 80 mm.
    p2a_workspace_x_min_arg = DeclareLaunchArgument(
        'p2a_workspace_x_min', default_value='0.30',
        description='完整杆 footprint 必须位于其内的研究包络 x 下界（米；非物理桌面边缘）')
    p2a_workspace_x_max_arg = DeclareLaunchArgument(
        'p2a_workspace_x_max', default_value='0.70',
        description='完整杆 footprint 必须位于其内的研究包络 x 上界（米；非物理桌面边缘）')
    p2a_workspace_y_min_arg = DeclareLaunchArgument(
        'p2a_workspace_y_min', default_value='-0.30',
        description='完整杆 footprint 必须位于其内的研究包络 y 下界（米；非物理桌面边缘）')
    p2a_workspace_y_max_arg = DeclareLaunchArgument(
        'p2a_workspace_y_max', default_value='0.30',
        description='完整杆 footprint 必须位于其内的研究包络 y 上界（米；非物理桌面边缘）')
    p2a_workspace_source_arg = DeclareLaunchArgument(
        'p2a_workspace_source', default_value='config/a1_scene_sampling.yaml 采样范围按 15° 最大 footprint 外扩后加 80 mm 余量；MuJoCo 支撑面为无限 plane，非物理桌面边缘',
        description='上述包络的来源说明，原样写入日志与记录')
    p2a_support_z_arg = DeclareLaunchArgument(
        'p2a_support_z', default_value='0.0',
        description='支撑面高度（米）；dual_scene.xml 的 floor plane 位于 z=0')
    p2a_support_plane_arg = DeclareLaunchArgument(
        'p2a_support_plane', default_value='true',
        description='在**验证场景**中补齐支撑 plane（MuJoCo floor 不进入 MoveIt）；仅改本地验证场景')
    p2a_support_object_id_arg = DeclareLaunchArgument(
        'p2a_support_object_id', default_value='a1_support_plane',
        description='验证场景中支撑面对象的 id；若场景已存在同名对象则不重复添加')
    p2a_ground_negative_arg = DeclareLaunchArgument(
        'p2a_ground_negative', default_value='true',
        description='注入一个低于支撑面的状态，验证检查确实能拒绝连杆穿地（验证注入，非采样位姿）')
    p2a_hand_aabb_substitution_arg = DeclareLaunchArgument(
        'p2a_hand_aabb_substitution', default_value='true',
        description='在独立验证模型中，仅把两只手掌的碰撞几何替换为 hand_c 网格的保守 AABB；'
                    '原 URDF 结果仍作对照输出。不修改共享 RobotModel / 全局 URDF')
    p2a_hand_negative_arg = DeclareLaunchArgument(
        'p2a_hand_negative', default_value='true',
        description='注入一个与替换后手掌 AABB 重合的障碍，验证替换几何仍有碰撞作用（验证注入）')
    p2a_grasp_offset_arg = DeclareLaunchArgument(
        'p2a_grasp_offset', default_value='0.15',
        description='抓取点沿杆长轴的偏移（米）；A0 的 final_y_offset 为 0.15')
    p2a_eef_base_yaw_deg_arg = DeclareLaunchArgument(
        'p2a_eef_base_yaw_deg', default_value='45.0',
        description='末端基准 yaw（度），即 A0 setRPY(π,0,π/4) 的 yaw 项；实际 yaw = 杆 yaw + 该值')

    p2a_support_depth_m_arg = DeclareLaunchArgument(
        'p2a_support_depth_m', default_value='2.0',
        description='验证用支撑 BOX 向下的深度（米）；顶面仍在 support_z，深板而非 2 cm 薄板')

    # --- A1 P2b: joint pre-grasp path planning and verification (no execution) ---
    p2b_check_arg = DeclareLaunchArgument(
        'p2b_check', default_value='false',
        description='A1 P2b: 从实际起始状态规划双臂联合预抓取路径，并在 P2a 判据场景中完整复检（仅规划，不执行）')
    p2b_planner_id_arg = DeclareLaunchArgument(
        'p2b_planner_id', default_value='PTP',
        description='P2b 使用的 MoveIt 规划器 id')
    p2b_planning_time_sec_arg = DeclareLaunchArgument(
        'p2b_planning_time_sec', default_value='10.0', description='单次规划时间上限（秒）')
    p2b_planning_attempts_arg = DeclareLaunchArgument(
        'p2b_planning_attempts', default_value='5', description='规划尝试次数上限')
    p2b_max_joint_step_rad_arg = DeclareLaunchArgument(
        'p2b_max_joint_step_rad', default_value='0.05',
        description='路径段细分时的最大单关节步长（rad）；用于离散碰撞采样，非连续保证')
    p2b_midpath_negative_arg = DeclareLaunchArgument(
        'p2b_midpath_negative', default_value='true',
        description='在路径中段注入障碍，证明校验覆盖路径内部（验证注入，非规划器结果）')

    enable_latency_monitor_arg = DeclareLaunchArgument(
        'enable_latency_monitor',
        default_value='true',
        description='是否启用通信时延监测节点'
    )

    latency_topic_arg = DeclareLaunchArgument(
        'latency_topic',
        default_value='/joint_states',
        description='用于通信时延统计的带 header.stamp 话题'
    )

    latency_task_status_topic_arg = DeclareLaunchArgument(
        'latency_task_status_topic',
        default_value='/task_status',
        description='任务状态话题（收到 DONE/ERROR 时自动出报告）'
    )

    latency_target_ms_arg = DeclareLaunchArgument(
        'latency_target_ms',
        default_value='100.0',
        description='通信时延阈值（毫秒）'
    )

    latency_result_dir_arg = DeclareLaunchArgument(
        'latency_result_dir',
        default_value='',
        description='通信时延报告输出目录（为空时默认 launch_logs）'
    )

    latency_result_prefix_arg = DeclareLaunchArgument(
        'latency_result_prefix',
        default_value='comm_latency',
        description='通信时延报告输出前缀'
    )

    enable_task_metrics_monitor_arg = DeclareLaunchArgument(
        'enable_task_metrics_monitor',
        default_value='true',
        description='是否启用任务指标统计节点（同步误差/能耗代理/末端误差）'
    )

    metrics_joint_state_topic_arg = DeclareLaunchArgument(
        'metrics_joint_state_topic',
        default_value='/joint_states',
        description='任务指标统计使用的关节状态话题'
    )

    metrics_task_stage_topic_arg = DeclareLaunchArgument(
        'metrics_task_stage_topic',
        default_value='/task_stage',
        description='任务阶段话题（用于分段指标统计）'
    )

    metrics_task_status_topic_arg = DeclareLaunchArgument(
        'metrics_task_status_topic',
        default_value='/task_status',
        description='任务终态话题（DONE/ERROR触发出报告）'
    )

    metrics_base_frame_arg = DeclareLaunchArgument(
        'metrics_base_frame',
        default_value='base_link',
        description='指标计算使用的基坐标系'
    )

    metrics_left_frame_arg = DeclareLaunchArgument(
        'metrics_left_frame',
        default_value='mj_left_hand',
        description='左臂末端参考坐标系'
    )

    metrics_right_frame_arg = DeclareLaunchArgument(
        'metrics_right_frame',
        default_value='mj_right_hand',
        description='右臂末端参考坐标系'
    )

    metrics_sync_target_mm_arg = DeclareLaunchArgument(
        'metrics_sync_target_mm',
        default_value='5.0',
        description='轨迹同步误差阈值（mm）'
    )

    metrics_ee_target_mm_arg = DeclareLaunchArgument(
        'metrics_ee_target_mm',
        default_value='2.0',
        description='末端误差阈值（mm）'
    )

    metrics_enable_rotate_arg = DeclareLaunchArgument(
        'metrics_enable_rotate',
        default_value='true',
        description='是否执行旋转阶段（用于DESCEND期望位姿推断）'
    )

    metrics_descend_place_z_arg = DeclareLaunchArgument(
        'metrics_descend_place_z',
        default_value='0.0',
        description='DESCEND目标放置高度（<=0表示按enable_rotate自动选择）'
    )

    metrics_descend_shift_x_arg = DeclareLaunchArgument(
        'metrics_descend_shift_x',
        default_value='999.0',
        description='DESCEND阶段X偏移（|value|>10表示按enable_rotate自动选择）'
    )

    metrics_result_dir_arg = DeclareLaunchArgument(
        'metrics_result_dir',
        default_value='',
        description='任务指标报告输出目录（为空时默认 launch_logs）'
    )

    metrics_result_prefix_arg = DeclareLaunchArgument(
        'metrics_result_prefix',
        default_value='task_metrics',
        description='任务指标报告输出前缀'
    )

    enable_adaptive_compliance_supervisor_arg = DeclareLaunchArgument(
        'enable_adaptive_compliance_supervisor',
        default_value='false',
        description='是否启用主从+自适应阻抗闭环监督节点'
    )

    adaptive_left_wrench_topic_arg = DeclareLaunchArgument(
        'adaptive_left_wrench_topic',
        default_value='/force_torque_sensor_broadcaster_left/wrench',
        description='自适应柔顺节点左臂力传感器话题'
    )

    adaptive_right_wrench_topic_arg = DeclareLaunchArgument(
        'adaptive_right_wrench_topic',
        default_value='/force_torque_sensor_broadcaster_right/wrench',
        description='自适应柔顺节点右臂力传感器话题'
    )

    adaptive_task_stage_topic_arg = DeclareLaunchArgument(
        'adaptive_task_stage_topic',
        default_value='/task_stage',
        description='自适应柔顺节点任务阶段话题'
    )

    adaptive_task_status_topic_arg = DeclareLaunchArgument(
        'adaptive_task_status_topic',
        default_value='/task_status',
        description='自适应柔顺节点任务状态话题'
    )

    adaptive_impedance_service_arg = DeclareLaunchArgument(
        'adaptive_impedance_service',
        default_value='/mj_left_and_mj_right/dual_cartesian_impedance_controller/parameters',
        description='自适应柔顺节点阻抗参数服务名'
    )

    adaptive_params_topic_arg = DeclareLaunchArgument(
        'adaptive_params_topic',
        default_value='/adaptive_impedance_params',
        description='自适应柔顺参数发布话题'
    )

    adaptive_update_rate_hz_arg = DeclareLaunchArgument(
        'adaptive_update_rate_hz',
        default_value='20.0',
        description='自适应柔顺节点更新频率(Hz)'
    )

    adaptive_nominal_stiffness_arg = DeclareLaunchArgument(
        'adaptive_nominal_stiffness',
        default_value='400.0',
        description='自适应柔顺节点平移刚度标称值'
    )

    adaptive_min_stiffness_arg = DeclareLaunchArgument(
        'adaptive_min_stiffness',
        default_value='150.0',
        description='自适应柔顺节点平移刚度最小值'
    )

    adaptive_force_gain_arg = DeclareLaunchArgument(
        'adaptive_force_gain',
        default_value='2.0',
        description='载荷驱动柔顺增益'
    )

    adaptive_imbalance_gain_arg = DeclareLaunchArgument(
        'adaptive_imbalance_gain',
        default_value='8.0',
        description='双臂受力不均衡驱动柔顺增益'
    )

    adaptive_active_stages_arg = DeclareLaunchArgument(
        'adaptive_active_stages',
        default_value='TRANSPORT,ROTATE,DESCEND',
        description='自适应柔顺生效阶段，逗号分隔'
    )
    
    # 使用 OpaqueFunction 延迟执行以处理 xacro
    delayed_task = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=launch_setup)]
    )
    
    return LaunchDescription([
        left_arm_group_arg,
        right_arm_group_arg,
        dual_arm_group_arg,
        control_mode_arg,
        approach_height_arg,
        grasp_height_arg,
        lift_height_arg,
        enable_rotate_arg,
        use_vision_arg,
        rod_target_x_arg,
        rod_target_y_arg,
        rod_target_yaw_deg_arg,
        rod_pose_source_arg,
        expected_run_id_arg,
        prepare_only_arg,
        a1_truth_topic_arg,
        a1_truth_timeout_sec_arg,
        p2a_check_arg,
        p2a_workspace_x_min_arg,
        p2a_workspace_x_max_arg,
        p2a_workspace_y_min_arg,
        p2a_workspace_y_max_arg,
        p2a_workspace_source_arg,
        p2a_support_z_arg,
        p2a_support_plane_arg,
        p2a_support_object_id_arg,
        p2a_ground_negative_arg,
        p2a_hand_aabb_substitution_arg,
        p2a_hand_negative_arg,
        p2a_support_depth_m_arg,
        p2b_check_arg,
        p2b_planner_id_arg,
        p2b_planning_time_sec_arg,
        p2b_planning_attempts_arg,
        p2b_max_joint_step_rad_arg,
        p2b_midpath_negative_arg,
        p2a_grasp_offset_arg,
        p2a_eef_base_yaw_deg_arg,
        enable_latency_monitor_arg,
        latency_topic_arg,
        latency_task_status_topic_arg,
        latency_target_ms_arg,
        latency_result_dir_arg,
        latency_result_prefix_arg,
        enable_task_metrics_monitor_arg,
        metrics_joint_state_topic_arg,
        metrics_task_stage_topic_arg,
        metrics_task_status_topic_arg,
        metrics_base_frame_arg,
        metrics_left_frame_arg,
        metrics_right_frame_arg,
        metrics_sync_target_mm_arg,
        metrics_ee_target_mm_arg,
        metrics_enable_rotate_arg,
        metrics_descend_place_z_arg,
        metrics_descend_shift_x_arg,
        metrics_result_dir_arg,
        metrics_result_prefix_arg,
        enable_adaptive_compliance_supervisor_arg,
        adaptive_left_wrench_topic_arg,
        adaptive_right_wrench_topic_arg,
        adaptive_task_stage_topic_arg,
        adaptive_task_status_topic_arg,
        adaptive_impedance_service_arg,
        adaptive_params_topic_arg,
        adaptive_update_rate_hz_arg,
        adaptive_nominal_stiffness_arg,
        adaptive_min_stiffness_arg,
        adaptive_force_gain_arg,
        adaptive_imbalance_gain_arg,
        adaptive_active_stages_arg,
        delayed_task
    ])
