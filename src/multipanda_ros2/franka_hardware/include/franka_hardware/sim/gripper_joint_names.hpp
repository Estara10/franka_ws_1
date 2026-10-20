#pragma once

#include <string>
#include <vector>

namespace franka_gripper
{
inline std::vector<std::string> fingerJointNamesForRobot(const std::string& robot_name)
{
  return {robot_name + "_finger_joint1", robot_name + "_finger_joint2"};
}
}  // namespace franka_gripper
