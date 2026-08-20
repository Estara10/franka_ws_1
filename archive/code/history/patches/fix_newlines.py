with open('src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp', 'r') as f:
    text = f.read()

text = text.replace('任务结束\\\n"', '任务结束\\n"')

with open('src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp', 'w') as f:
    f.write(text)
