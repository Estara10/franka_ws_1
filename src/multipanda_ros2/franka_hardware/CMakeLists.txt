cmake_minimum_required(VERSION 3.5)
project(franka_hardware)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to release
set(default_build_type "Release")
# Respect the build type from user and ide
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=unused-variable)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_ros2_control REQUIRED)
find_package(std_srvs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(dual_arm_a1_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Franka REQUIRED) # Franka_DIR will provide the path to the libfranka installation
find_package(Boost COMPONENTS thread chrono REQUIRED)

# Mujoco dependencies
# set(OpenGL_GL_PREFERENCE GLVND)
# find_package(glfw3 3.3 REQUIRED)     # GLFW3 here,
# find_package(mujoco REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)

###### Real robot ###############
add_library(franka_hardware
        SHARED
        src/real/franka_multi_hardware_interface.cpp
        src/real/robot.cpp
        src/real/franka_param_service_server.cpp
        src/real/franka_error_recovery_service_server.cpp
        src/common/franka_executor.cpp
        src/common/control_mode.cpp
        src/common/helper_functions.cpp
)

target_include_directories(
        franka_hardware
        PRIVATE
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
)

ament_target_dependencies(
        franka_hardware
        hardware_interface
        Franka
        pluginlib
        rclcpp
        rclcpp_action
        rclcpp_components
        franka_msgs
        std_srvs
)

target_link_libraries(
        franka_hardware
        Boost::thread
        Boost::chrono
        ${YAML_CPP_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface franka_hardware.xml)

########## Mujoco simulation robot ##########
add_library(franka_mj_hardware 
  SHARED
  src/sim/franka_mj_hardware_system.cpp
  src/sim/robot_sim.cpp
  src/sim/gripper_sim_action_server.cpp
  src/common/franka_executor.cpp
  src/common/control_mode.cpp
  src/common/helper_functions.cpp
  )

target_compile_features(franka_mj_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(franka_mj_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  )

ament_target_dependencies(
  franka_mj_hardware
  "mujoco_ros2_control"
  "hardware_interface"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "pluginlib"
  Franka
  franka_msgs
  dual_arm_a1_msgs
  std_srvs
  std_msgs
)

target_link_libraries(
  franka_mj_hardware
  Boost::thread
  Boost::chrono
  ${YAML_CPP_LIBRARIES}
)

pluginlib_export_plugin_description_file(mujoco_ros2_control franka_mj_hardware.xml)

# Install scripts
install(
        TARGETS franka_hardware franka_mj_hardware
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
    # A1 P3a-2a: the gripper feedback handoff and its stability window are
    # header-only and ROS-free, so they are tested directly against the same
    # code the server runs.
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_gripper_shared_state test/test_gripper_shared_state.cpp)
    target_include_directories(test_gripper_shared_state PRIVATE include)

    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        franka_hardware
        franka_mj_hardware
)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
        mujoco_ros2_control
)

ament_package()
