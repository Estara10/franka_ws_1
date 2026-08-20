from launch import LaunchDescription
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("dual_panda", package_name="dual_arm_moveit_config").to_moveit_configs()
    # 启动MoveIt demo
    ld = generate_demo_launch(moveit_config)
    # 获取工作空间绝对路径，确保脚本路径正确
    ws_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    script_path = os.path.join(ws_dir, "src/dual_arm_moveit_config/scripts/set_home_joint_state.py")
    # 启动自动发布home关节状态脚本
    set_home = ExecuteProcess(
        cmd=["python3", script_path],
        output="screen"
    )
    return LaunchDescription(ld.entities + [set_home])
