import re

with open('src/multipanda_ros2/franka_moveit_config/launch/sim_dual_moveit.launch.py', 'r') as f:
    content = f.read()

# Replace the single pipeline config with dual pipeline config
new_config = """
    # 1. 基础 OMPL 规划管道（无 CHOMP，用于 TRANSPORT 和 ROTATE 等严格插值阶段）
    ompl_planning_pipeline_config = {
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # 2. 带有 CHOMP 优化器的 OMPL 管道（用于 APPROACH 和 RETREAT）
    ompl_chomp_planning_pipeline_config = {
        'ompl_chomp': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints '
                                'default_planner_request_adapters/CHOMPOptimizerAdapter',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_chomp_planning_pipeline_config['ompl_chomp'].update(ompl_planning_yaml)
    
    # 注册所有的 Pipelines
    planning_pipelines_config = {
        'planning_pipelines': ['ompl', 'ompl_chomp'],
        'default_planning_pipeline': 'ompl'
    }
"""

content = re.sub(
    r"    ompl_planning_pipeline_config = \{\n\s+'move_group': \{.*?ompl_planning_pipeline_config\['move_group'\]\.update\(ompl_planning_yaml\)",
    new_config.strip(),
    content,
    flags=re.DOTALL
)

# Fix parameters list in move_group
move_group_params_old = """
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            chomp_planning_yaml,
        ],
"""
move_group_params_new = """
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            ompl_chomp_planning_pipeline_config,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            chomp_planning_yaml,
        ],
"""
content = content.replace(move_group_params_old, move_group_params_new)

# rviz params
rviz_params_old = """
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
"""
rviz_params_new = """
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            ompl_chomp_planning_pipeline_config,
            planning_pipelines_config,
            kinematics_yaml,
        ],
"""
content = content.replace(rviz_params_old, rviz_params_new)

with open('src/multipanda_ros2/franka_moveit_config/launch/sim_dual_moveit.launch.py', 'w') as f:
    f.write(content)

print(f"Patched successfully.")
