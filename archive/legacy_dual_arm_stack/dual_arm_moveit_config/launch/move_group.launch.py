import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. 构建 MoveIt 配置
    # [修复] 使用绝对路径解决警告
    description_path = get_package_share_path('dual_arm_description') / 'urdf' / 'dual_arm.urdf.xacro'

    moveit_config = (
        MoveItConfigsBuilder("dual_panda", package_name="dual_arm_moveit_config")
        .robot_description(file_path=str(description_path))
        .robot_description_semantic(file_path="config/dual_panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # 3. 手动定义 MoveGroup 节点 (确保参数传递)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # [关键修复] 强制使用仿真时间
            {'use_sim_time': use_sim_time},
            # 允许轨迹有一定的执行时间余量，防止超时
            {'trajectory_execution.allowed_execution_duration_scaling': 2.0},
            {'trajectory_execution.allowed_goal_duration_margin': 1.0},
            {'publish_robot_description_semantic': True}
        ],
    )

    # 4. RViz (可选，方便调试)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time}
        ],
        arguments=["-d", str(get_package_share_path('dual_arm_moveit_config') / 'config/moveit.rviz')], # 如果没有这个文件也没关系，RViz会开个空的
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='Use simulation (Gazebo) clock if true'),
        
        run_move_group_node,
        rviz_node # [修改] 取消注释
    ])
