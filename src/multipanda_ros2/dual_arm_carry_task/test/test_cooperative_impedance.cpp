#include <gtest/gtest.h>
#include "dual_arm_carry_task/cooperative_impedance.hpp"

namespace {
using dual_arm_carry_task::CooperativeWrench;
using dual_arm_carry_task::distributeObjectWrench;
using dual_arm_carry_task::internalRelativeWrench;
using dual_arm_carry_task::rotateWrench;

CooperativeWrench resultant(const std::array<CooperativeWrench, 2>& hands,
                            const Eigen::Vector3d& object,
                            const std::array<Eigen::Vector3d, 2>& positions) {
  CooperativeWrench total = CooperativeWrench::Zero();
  for (std::size_t i = 0; i < hands.size(); ++i) {
    total.head<3>() += hands[i].head<3>();
    total.tail<3>() += hands[i].tail<3>() +
        (positions[i] - object).cross(hands[i].head<3>());
  }
  return total;
}
}

TEST(CooperativeImpedance, ZeroObjectWrenchProducesZeroHandWrenches) {
  const Eigen::Vector3d object(0.2, -0.1, 0.4);
  const std::array<Eigen::Vector3d, 2> hands{{{0.1, 0.2, 0.3}, {0.3, -0.4, 0.5}}};
  const auto result = distributeObjectWrench(CooperativeWrench::Zero(), object, hands);
  EXPECT_DOUBLE_EQ(result[0].norm(), 0.0);
  EXPECT_DOUBLE_EQ(result[1].norm(), 0.0);
}

TEST(CooperativeImpedance, CommonWrenchIncludesMomentArmsAndPreservesPower) {
  const Eigen::Vector3d object(0.3, -0.2, 0.5);
  const std::array<Eigen::Vector3d, 2> hands{{{0.1, 0.4, 0.2}, {0.5, -0.1, 0.7}}};
  CooperativeWrench object_wrench;
  object_wrench << 7.0, -3.0, 4.0, 0.8, -1.2, 0.4;
  const auto mapped = distributeObjectWrench(object_wrench, object, hands);
  EXPECT_LT((resultant(mapped, object, hands) - object_wrench).norm(), 1e-12);

  Eigen::Vector3d v(0.2, -0.4, 0.1), omega(-0.3, 0.1, 0.5);
  double hand_power = 0.0;
  for (std::size_t i = 0; i < mapped.size(); ++i) {
    const Eigen::Vector3d vi = v + omega.cross(hands[i] - object);
    hand_power += mapped[i].head<3>().dot(vi) + mapped[i].tail<3>().dot(omega);
  }
  const double object_power = object_wrench.head<3>().dot(v) + object_wrench.tail<3>().dot(omega);
  EXPECT_NEAR(hand_power, object_power, 1e-12);
}

TEST(CooperativeImpedance, RelativeWrenchHasZeroObjectResultantAndCommonPower) {
  const Eigen::Vector3d object(-0.2, 0.3, 0.6);
  const std::array<Eigen::Vector3d, 2> hands{{{-0.1, 0.5, 0.4}, {0.25, -0.3, 0.8}}};
  const Eigen::Vector3d force(2.0, -1.5, 0.7), torque(-0.4, 0.3, 0.9);
  const auto mapped = internalRelativeWrench(force, torque, object, hands);
  EXPECT_LT(resultant(mapped, object, hands).norm(), 1e-12);

  const Eigen::Vector3d v(0.3, -0.1, 0.2), omega(0.2, 0.4, -0.3);
  double hand_power = 0.0;
  for (std::size_t i = 0; i < mapped.size(); ++i) {
    const Eigen::Vector3d vi = v + omega.cross(hands[i] - object);
    hand_power += mapped[i].head<3>().dot(vi) + mapped[i].tail<3>().dot(omega);
  }
  EXPECT_NEAR(hand_power, 0.0, 1e-12);
}

TEST(CooperativeImpedance, WrenchRotationPreservesPower) {
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, -1).normalized()).toRotationMatrix();
  CooperativeWrench wrench;
  wrench << 4.0, -2.0, 1.0, 0.5, 0.1, -0.8;
  CooperativeWrench twist;
  twist << -0.2, 0.6, 0.3, 0.4, -0.5, 0.1;
  const auto local_wrench = rotateWrench(wrench, rotation);
  CooperativeWrench local_twist;
  local_twist.head<3>() = rotation.transpose() * twist.head<3>();
  local_twist.tail<3>() = rotation.transpose() * twist.tail<3>();
  EXPECT_NEAR(wrench.dot(twist), local_wrench.dot(local_twist), 1e-12);
}

TEST(CooperativeImpedance, CommonAndRelativeGainsPreservePerArmStiffness) {
  const Eigen::Vector3d object = Eigen::Vector3d::Zero();
  const std::array<Eigen::Vector3d, 2> hands{{{0.0, 0.2, 0.0}, {0.0, -0.2, 0.0}}};
  const Eigen::Vector3d object_error(0.01, -0.02, 0.03);
  CooperativeWrench common;
  common << 800.0 * object_error, Eigen::Vector3d::Zero();
  const auto common_hands = distributeObjectWrench(common, object, hands);
  EXPECT_LT((common_hands[0].head<3>() - 400.0 * object_error).norm(), 1e-12);
  EXPECT_LT((common_hands[1].head<3>() - 400.0 * object_error).norm(), 1e-12);

  const Eigen::Vector3d relative_error(-0.02, 0.01, 0.03);
  const Eigen::Vector3d force_on_right = 200.0 * relative_error;
  const auto relative_hands = internalRelativeWrench(
      force_on_right, Eigen::Vector3d::Zero(), object, hands);
  const Eigen::Vector3d original_right_hand_error = 0.5 * relative_error;
  EXPECT_LT((relative_hands[1].head<3>() - 400.0 * original_right_hand_error).norm(), 1e-12);
  EXPECT_LT((relative_hands[0].head<3>() + 400.0 * original_right_hand_error).norm(), 1e-12);
}
