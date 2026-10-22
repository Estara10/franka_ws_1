#pragma once

#include <mutex>
#include <rclcpp/logger.hpp>
#include <Eigen/Dense>
#include <franka_semantic_components/franka_robot_model.hpp> // semantic component model 
/*
A thin wrapper class for the semantic component's franka_robot_model.
Mainly to make multi-arm control easier to manage.
*/

namespace multi_mode_controller {
  class MultiModeController;
}

namespace switch_engine {
  class SwitchEngine;
}

namespace panda_controllers {

struct RobotDataSnapshot {
  franka::RobotState state;
  Eigen::Matrix<double, 7, 1> coriolis;
  Eigen::Matrix<double, 6, 7> ee_zero_jacobian;
  Eigen::Matrix<double, 7, 7> mass;
};

class RobotData {
 public:
  RobotData() = delete;
  RobotData(RobotData&& rd);
  RobotData(const std::string& resource);
  // getters should go to public
  // Eigen::Matrix<double, 7, 1> getCoriolis();
  // Eigen::Matrix<double, 6, 7> getEeZeroJacobian();
  // Eigen::Matrix<double, 6, 7> getEeBodyJacobian();
  // Eigen::Matrix<double, 7, 7> getMass();
  // franka::RobotState getState();
  const Eigen::Matrix<double, 7, 1>& coriolis();
  const Eigen::Matrix<double, 6, 7>& eeZeroJacobian();
  const Eigen::Matrix<double, 6, 7>& eeBodyJacobian();
  const Eigen::Matrix<double, 7, 7>& mass();
  // Return a snapshot. Returning a reference after releasing update_mutex_
  // lets the controller update thread tear a 16-element O_T_EE matrix while
  // SwitchEngine serializes it into a ROS service response.
  franka::RobotState state();
  // Copy controller state and model terms under one lock. The active Cartesian
  // torque path can then use stable values for the full calculation.
  RobotDataSnapshot snapshot();

  bool init(std::string robot_model_interface_name, std::string robot_state_interface_name);
  bool initialize_model();
  bool activate_model(std::vector<hardware_interface::LoanedStateInterface>& state_interface);
  bool deactivate_model();
  void update();
 private:
  std::mutex update_mutex_;
  Eigen::Matrix<double, 7, 1> coriolis_;
  Eigen::Matrix<double, 6, 7> ee_zero_jacobian_;
  Eigen::Matrix<double, 6, 7> ee_body_jacobian_;
  Eigen::Matrix<double, 7, 7> mass_;
  const std::string resource_;
  franka::RobotState state_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> frm_;
};

}
