cmake_minimum_required(VERSION 3.8)
project(dual_arm_carry_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(multi_mode_control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
# A1 P2a: locate the simulation's hand collision mesh via the package share dir.
find_package(ament_index_cpp REQUIRED)
# A1 P3a-1: verify which controllers are active before commanding motion.
find_package(controller_manager_msgs REQUIRED)
# A1 P1a: rod body state / simulation info services.
find_package(mujoco_ros_msgs REQUIRED)
# A1 P1b: rod pose truth message.
find_package(dual_arm_a1_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Dual arm carry task node
add_executable(dual_arm_carry_task_node
  src/carry_task_state_machine.cpp src/dual_arm_controller.cpp src/contact_load_probe.cpp
)

add_executable(communication_latency_monitor_node
  src/communication_latency_monitor.cpp
)

add_executable(task_metrics_monitor_node
  src/task_metrics_monitor.cpp
)

add_executable(adaptive_compliance_supervisor_node
  src/adaptive_compliance_supervisor.cpp
)

# A1 P1a: pure, offline-testable core (sampling, geometry, settling criterion,
# single-candidate orchestration). Deliberately has no ROS dependency.
add_library(a1_sampling src/a1_sampling.cpp)

# A1 P1b: pure, offline-testable truth handling (validation, latch-once freeze,
# frame mapping, planning-scene read-back check). No ROS dependency.
add_library(a1_truth src/a1_truth.cpp)

# A1 P3a-2a: gripper action protocol semantics (units, result classification,
# late-acceptance decision). No ROS dependency.
add_library(gripper_protocol src/gripper_protocol.cpp)
target_include_directories(gripper_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# A1 P3a-2a: the gripper action client. It takes a plain node and a bounds
# struct rather than the whole task context, so the interface test can drive
# THIS client against a fake action server.
add_library(gripper_client src/gripper_client.cpp)
target_include_directories(gripper_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gripper_client
  rclcpp
  rclcpp_action
  control_msgs
  franka_msgs
)
target_link_libraries(gripper_client gripper_protocol)

# A1 P2a: pure, offline-testable sample validity checks (rotated footprint,
# workspace/support judgement, shared pre-grasp/grasp target construction,
# joint-limit and residual helpers). No ROS dependency.
add_library(a1_sample_validator src/a1_sample_validator.cpp)
target_link_libraries(a1_sample_validator a1_truth)

ament_target_dependencies(a1_sample_validator
  dual_arm_a1_msgs
)

# A1 P1a: ROS side -- applies one sampled pose via /set_body_state and writes a
# manifest. Does not publish truth, does not start the task.
add_executable(a1_scene_runner_node
  src/a1_scene_runner_node.cpp
)

ament_target_dependencies(dual_arm_carry_task_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  moveit_visual_tools
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  std_msgs
  control_msgs
  franka_msgs
  moveit_msgs
  shape_msgs
  tf2_eigen
)

ament_target_dependencies(communication_latency_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(task_metrics_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
)

ament_target_dependencies(adaptive_compliance_supervisor_node
  rclcpp
  geometry_msgs
  std_msgs
  multi_mode_control_msgs
)

target_include_directories(a1_sampling PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(a1_truth PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(a1_sample_validator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(a1_scene_runner_node
  rclcpp
  mujoco_ros_msgs
  dual_arm_a1_msgs
  geometry_msgs
)

target_link_libraries(a1_scene_runner_node a1_sampling)

# The task node needs the truth message and the pure truth handling.
ament_target_dependencies(dual_arm_carry_task_node
  dual_arm_a1_msgs
  ament_index_cpp
  controller_manager_msgs
  # A1 P3a-2b: the task reads the rod's physical pose from MuJoCo before and
  # after the descent, so physics and the planning scene are never confused.
  mujoco_ros_msgs
)

target_link_libraries(dual_arm_carry_task_node a1_truth a1_sample_validator gripper_protocol gripper_client)

# Install targets
install(TARGETS
  dual_arm_carry_task_node
  communication_latency_monitor_node
  task_metrics_monitor_node
  adaptive_compliance_supervisor_node
  a1_scene_runner_node
  a1_sampling
  a1_truth
  a1_sample_validator
  gripper_protocol
  gripper_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install A1 P1a configuration
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_a1_sampling test/test_a1_sampling.cpp)
  target_link_libraries(test_a1_sampling a1_sampling)

  ament_add_gtest(test_a1_stability test/test_a1_stability.cpp)
  target_link_libraries(test_a1_stability a1_sampling)

  ament_add_gtest(test_a1_service_faults test/test_a1_service_faults.cpp)
  target_link_libraries(test_a1_service_faults a1_sampling)

  ament_add_gtest(test_a1_truth test/test_a1_truth.cpp)
  target_link_libraries(test_a1_truth a1_truth)

  ament_add_gtest(test_a1_sample_validator test/test_a1_sample_validator.cpp)
  target_link_libraries(test_a1_sample_validator a1_sample_validator)

  ament_add_gtest(test_s1_object_pose test/test_s1_object_pose.cpp)

  ament_add_gtest(test_s1_time_scaling test/test_s1_time_scaling.cpp)

  ament_add_gtest(test_gripper_protocol test/test_gripper_protocol.cpp)
  target_link_libraries(test_gripper_protocol gripper_protocol)

  ament_add_gtest(test_grasp_contact_acceptance test/test_grasp_contact_acceptance.cpp)

  # Interface test: the production client driven against a minimal ROS fake
  # action server. Not a helper test.
  ament_add_gtest(test_gripper_client_faults test/test_gripper_client_faults.cpp)
  ament_target_dependencies(test_gripper_client_faults rclcpp rclcpp_action control_msgs)
  target_link_libraries(test_gripper_client_faults gripper_client)
  set_tests_properties(test_gripper_client_faults PROPERTIES TIMEOUT 120)
endif()

ament_package()
