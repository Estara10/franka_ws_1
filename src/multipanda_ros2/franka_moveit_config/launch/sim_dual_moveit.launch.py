#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def concatenate_ns(ns1, ns2, absolute=False):
    
    if(len(ns1) == 0):
        return ns2
    if(len(ns2) == 0):
        return ns1
    
    # check for /s at the end and start
    if(ns1[0] == '/'):
        ns1 = ns1[1:]
    if(ns1[-1] == '/'):
        ns1 = ns1[:-1]
    if(ns2[0] == '/'):
        ns2 = ns2[1:]
    if(ns2[-1] == '/'):
        ns2 = ns2[:-1]
    if(absolute):
        ns1 = '/' + ns1
    return ns1 + '/' + ns2

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # Parameters as launch arguments
    arm_id_1_param = "arm_id_1"
    arm_id_2_param = "arm_id_2"
    initial_positions_1_param = 'initial_positions_1'
    initial_positions_2_param = 'initial_positions_2'
    use_rviz_param = 'use_rviz'
    headless_param = 'headless'
    no_render_param = 'no_render'

    arm_id_1 = LaunchConfiguration(arm_id_1_param)
    arm_id_2 = LaunchConfiguration(arm_id_2_param)
    initial_positions_1 = LaunchConfiguration(initial_positions_1_param)
    initial_positions_2 = LaunchConfiguration(initial_positions_2_param)
    use_rviz = LaunchConfiguration(use_rviz_param)
    headless = LaunchConfiguration(headless_param)
    no_render = LaunchConfiguration(no_render_param)
    

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    # Fixed variables
    load_gripper = True # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.
    
    if(load_gripper): # mujoco scene file must be manually adjusted since there's no way to pass parameters
        scene_file = 'dual_scene.xml'
    else:
        scene_file = 'dual_scene_ng.xml'

    # planning_context
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'sim',
                                     'dual_panda_arm_sim.urdf.xacro')
    xml_file = os.path.join(get_package_share_directory('franka_description'), 'mujoco', 'franka', scene_file)
    franka_bringup_path = get_package_share_directory('franka_bringup')


    # Robot state publisher setup
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' arm_id_1:=', arm_id_1, 
            ' arm_id_2:=', arm_id_2,
            ' hand_1:=', str(load_gripper).lower(),
            ' hand_2:=', str(load_gripper).lower(),
            ' initial_positions_1:=', initial_positions_1,
            ' initial_positions_2:=', initial_positions_2])
    robot_description = {'robot_description': robot_description_config}

    # 使用预生成的静态SRDF文件，确保碰撞规则正确加载
    # 原因：动态xacro展开在ROS2中可能导致参数传递问题
    franka_semantic_srdf_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                              'srdf',
                                              'dual_panda_generated.srdf')
    
    # 直接读取静态SRDF文件内容
    with open(franka_semantic_srdf_file, 'r') as srdf_file:
        robot_description_semantic_content = srdf_file.read()
    
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )

    # Joint limits (acceleration limits for TOTG time parameterization)
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            'franka_moveit_config', 'config/joint_limits.yaml'
        )
    }

    # Planning Functionality
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

    # 2. CHOMP 规划管道（使用 CHOMPPlanner，而非不存在的 CHOMPOptimizerAdapter）
    chomp_planning_pipeline_config = {
        'chomp': {
            'planning_plugin': 'chomp_interface/CHOMPPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    
    # 注册所有的 Pipelines
    planning_pipelines_config = {
        'planning_pipelines': ['ompl', 'chomp'],
        'default_planning_pipeline': 'ompl'
    }

    # Load CHOMP wrapper parameters
    try:
        chomp_planning_yaml = load_yaml(
            'franka_moveit_config', 'config/chomp_planning.yaml'
        )
    except Exception:
        chomp_planning_yaml = {}

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_moveit_config', 'config/sim_dual_panda_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        # [修复] 放宽轨迹起点偏差容差：LIFT后MuJoCo物理漂移可能超过0.01rad
        # 0.05 rad ≈ 2.9°，仍然安全但能容忍附着物体带来的微小关节漂移
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            chomp_planning_pipeline_config,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            chomp_planning_yaml,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'dual_moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            chomp_planning_pipeline_config,
            planning_pipelines_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'sim_dual_panda_ros_controllers.yaml',
    )

    # Mujoco ros2 server
    mujoco_ros2_node = IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(franka_bringup_path + '/launch/sim/launch_mujoco_ros_server.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_file,
                'verbose': "true",
                'unpause': "true",  # 自动开始仿真，不暂停
                'headless': headless,
                'no_render': no_render,
                'ns': '',
                'mujoco_plugin_config': ros2_controllers_path
            }.items()
        )

    # Load controllers
    load_controllers = []
    for controller in ['dual_panda_arm_controller', 'joint_state_broadcaster', 'force_torque_sensor_broadcaster_left', 'force_torque_sensor_broadcaster_right']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {} --controller-manager-timeout 60 --service-call-timeout 60'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    # Warehouse mongodb server
    db_config = LaunchConfiguration('db')
    mongodb_server_node = Node(
        package='warehouse_ros_mongo',
        executable='mongo_wrapper_ros.py',
        parameters=[
            {'warehouse_port': 33829},
            {'warehouse_host': 'localhost'},
            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'},
        ],
        output='screen',
        condition=IfCondition(db_config)
    )

    # Joint state publisher setup
    jsp_source_list = [concatenate_ns('', 'joint_states', True)]
    if(load_gripper):
        jsp_source_list.append(concatenate_ns('', 'mj_left_gripper_sim_node/joint_states/joint_states', True))
        jsp_source_list.append(concatenate_ns('', 'mj_right_gripper_sim_node/joint_states/joint_states', True))

    joint_state_publisher = Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace= '',
            parameters=[
                {'source_list': jsp_source_list,
                 'rate': 30}],
    )

    # Launch arguments    
    arm_id_1_arg = DeclareLaunchArgument(
        arm_id_1_param,
        default_value='mj_left',
        description='Unique name of robot 1.'
    )
    arm_id_2_arg = DeclareLaunchArgument(
        arm_id_2_param,
        default_value='mj_right',
        description='Unique name of robot 2.'
    )
    initial_pose_1_arg = DeclareLaunchArgument(
        initial_positions_1_param,
        default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
        description='Initial joint positions of robot 1. Must be enclosed in quotes, and in pure number.'
                    'Defaults to the "communication_test" pose.')
    initial_pose_2_arg = DeclareLaunchArgument(
        initial_positions_2_param,
        default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
        description='Initial joint positions of robot 2. Must be enclosed in quotes, and in pure number.'
                    'Defaults to the "communication_test" pose.')
    use_rviz_arg = DeclareLaunchArgument(
        use_rviz_param,
        default_value='true',
        description='Whether to start RViz.'
    )
    headless_arg = DeclareLaunchArgument(
        headless_param,
        default_value='false',
        description='Whether to run MuJoCo without an on-screen viewer.'
    )
    no_render_arg = DeclareLaunchArgument(
        no_render_param,
        default_value='false',
        description='Whether to disable MuJoCo rendering entirely.'
    )
    
    return LaunchDescription(
        [arm_id_1_arg,
         arm_id_2_arg,
         initial_pose_1_arg,
         initial_pose_2_arg,
         use_rviz_arg,
         headless_arg,
         no_render_arg,
         db_arg,
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         mujoco_ros2_node,
         mongodb_server_node,
         joint_state_publisher
         ]
        + load_controllers
    )
