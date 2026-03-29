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
                'use_sim_time': True,
            }
        ]
    )
    
    return [task_node]


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
        delayed_task
    ])
