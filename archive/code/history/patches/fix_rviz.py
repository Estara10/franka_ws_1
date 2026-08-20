with open('src/multipanda_ros2/franka_moveit_config/launch/sim_dual_moveit.launch.py', 'r') as f:
    text = f.read()

text = text.replace('''        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            ompl_chomp_planning_pipeline_config,
            planning_pipelines_config,
            ompl_chomp_planning_pipeline_config,
            planning_pipelines_config,
            kinematics_yaml,
        ],''', '''        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            ompl_chomp_planning_pipeline_config,
            planning_pipelines_config,
            kinematics_yaml,
        ],''')

with open('src/multipanda_ros2/franka_moveit_config/launch/sim_dual_moveit.launch.py', 'w') as f:
    f.write(text)
