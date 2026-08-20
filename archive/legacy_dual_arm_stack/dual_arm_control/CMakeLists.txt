cmake_minimum_required(VERSION 3.8)
project(dual_arm_control)

# 1. 基础编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(franka_msgs REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)

# 3. 包含目录
include_directories(include)

# 4. 编译可执行文件：双臂运动测试节点
add_executable(dual_arm_move_test 
  src/dual_arm_move_test.cpp 
  src/dual_arm_gripper_controller.cpp
)
ament_target_dependencies(dual_arm_move_test
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

add_executable(dual_arm_pick_place
  src/dual_arm_pick_place.cpp
  src/dual_arm_gripper_controller.cpp
)
ament_target_dependencies(dual_arm_pick_place
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  tf2_geometry_msgs
)

# 5. 安装规则
install(TARGETS
  dual_arm_move_test
  dual_arm_pick_place
  DESTINATION lib/${PROJECT_NAME}
)
# 安装 launch 文件夹
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
