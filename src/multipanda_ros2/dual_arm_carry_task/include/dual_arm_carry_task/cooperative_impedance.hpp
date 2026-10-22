#pragma once

#include <Eigen/Dense>
#include <array>

namespace dual_arm_carry_task {

using CooperativeWrench = Eigen::Matrix<double, 6, 1>;  // [force; torque]

inline std::array<CooperativeWrench, 2> distributeObjectWrench(
    const CooperativeWrench& object_wrench,
    const Eigen::Vector3d& object_position,
    const std::array<Eigen::Vector3d, 2>& hand_positions) {
  std::array<CooperativeWrench, 2> hands;
  const Eigen::Vector3d force = 0.5 * object_wrench.head<3>();
  for (std::size_t i = 0; i < hands.size(); ++i) {
    hands[i].head<3>() = force;
    hands[i].tail<3>() = 0.5 * object_wrench.tail<3>() -
        (hand_positions[i] - object_position).cross(force);
  }
  return hands;
}

// Internal relative wrench: equal/opposite endpoint forces plus endpoint
// couples that cancel their moment about the object origin. Pure relative
// couples are an equal/opposite pair and therefore add no object wrench.
inline std::array<CooperativeWrench, 2> internalRelativeWrench(
    const Eigen::Vector3d& force_on_right,
    const Eigen::Vector3d& torque_on_right,
    const Eigen::Vector3d& object_position,
    const std::array<Eigen::Vector3d, 2>& hand_positions) {
  std::array<CooperativeWrench, 2> hands;
  const Eigen::Vector3d r_left = hand_positions[0] - object_position;
  const Eigen::Vector3d r_right = hand_positions[1] - object_position;
  const Eigen::Vector3d couple = 0.5 * (r_left - r_right).cross(force_on_right);
  hands[0].head<3>() = -force_on_right;
  hands[1].head<3>() = force_on_right;
  hands[0].tail<3>() = couple - torque_on_right;
  hands[1].tail<3>() = couple + torque_on_right;
  return hands;
}

inline CooperativeWrench rotateWrench(const CooperativeWrench& wrench,
                                      const Eigen::Matrix3d& world_from_base) {
  CooperativeWrench local;
  local.head<3>() = world_from_base.transpose() * wrench.head<3>();
  local.tail<3>() = world_from_base.transpose() * wrench.tail<3>();
  return local;
}

inline Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rotation) {
  const Eigen::AngleAxisd aa(rotation);
  return aa.axis() * aa.angle();
}

}  // namespace dual_arm_carry_task
