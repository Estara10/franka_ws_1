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
find_package(moveit_msgs REQUIRED)
find_package(multi_mode_control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Include directories
include_directories(
  include
)

# Dual arm carry task node
add_executable(dual_arm_carry_task_node 
  src/carry_task_state_machine.cpp src/dual_arm_controller.cpp src/gripper_controller.cpp
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

# Install targets
install(TARGETS
  dual_arm_carry_task_node
  communication_latency_monitor_node
  task_metrics_monitor_node
  adaptive_compliance_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
