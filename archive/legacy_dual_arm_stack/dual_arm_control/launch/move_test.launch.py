from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 加载双臂的 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("dual_panda", package_name="dual_arm_moveit_config").to_moveit_configs()

    # 2. 启动你的 C++ 控制节点，并把配置传给它
    return LaunchDescription([
        Node(
            package="dual_arm_control",
            executable="dual_arm_move_test",
            name="dual_arm_move_test",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {'use_sim_time': True}  # 强制使用仿真时间，与 Gazebo 同步
            ],
        )
    ])
