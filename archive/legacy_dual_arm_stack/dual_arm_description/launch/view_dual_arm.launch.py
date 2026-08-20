import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # 新增引用

def generate_launch_description():
    # 1. 路径配置
    pkg_share = get_package_share_directory('dual_arm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'dual_arm.urdf.xacro')

    # 2. 编译 Xacro 为 URDF 字符串
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            xacro_file, " ",
            "use_fake_hardware:=true",
        ]
    )

    # 3. 关键修正：使用 ParameterValue 包装，防止 YAML 解析错误
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 4. 节点：发布机器人状态
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 5. 节点：关节状态发布 GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{'use_sim_time': True}],
    )

    # 6. 节点：RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
