import re

with open('/home/wusiala/mujoco_franka/franka_ws/src/multipanda_ros2/franka_moveit_config/launch/sim_dual_moveit.launch.py', 'r') as f:
    text = f.read()

# Replace spawner format string
old_str = "cmd=['ros2 run controller_manager spawner {}'.format(controller)],"
new_str = "cmd=['ros2 run controller_manager spawner {} --controller-manager-timeout 60 --service-call-timeout 60'.format(controller)],"
text = text.replace(old_str, new_str)

with open('/home/wusiala/mujoco_franka/franka_ws/src/multipanda_ros2/franka_moveit_config/launch/sim_dual_moveit.launch.py', 'w') as f:
    f.write(text)
