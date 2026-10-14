// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <atomic>
#include <boost/thread.hpp>
#include <mujoco/mujoco.h>


#include "mujoco_ros2_control/mujoco_ros2_control_system_interface.hpp"
#include "franka_hardware/common/franka_executor.hpp"
#include "franka_hardware/common/control_mode.h"
#include "franka_hardware/common/helper_functions.hpp"
#include "franka_hardware/sim/robot_sim.hpp"
#include "franka_hardware/sim/gripper_sim_action_server.hpp"
#include "franka_msgs/msg/pose_stamped_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_hardware {
  
void set_torque_control(const mjModel* m,int actuator_no,int flag);

void set_position_servo(const mjModel* m,int actuator_no,double kp);

void set_velocity_servo(const mjModel* m,int actuator_no,double kv);

struct ObjectContainer{
  std::string obj_name_;
  int obj_body_index_;
  int obj_mocap_index_;
  int obj_jnt_adr_;
  int obj_jnt_num_;
  int obj_jnt_type_;
  std::array<double, 3> obj_position_;
  std::array<double, 4> obj_orientation_;
};

struct ArmContainer {
  std::string robot_name_;
  std::shared_ptr<RobotSim> robot_;
  franka::RobotState hw_franka_robot_state_;

  std::array<double, 7> hw_commands_joint_effort_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_commands_joint_position_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_commands_joint_velocity_{0, 0, 0, 0, 0, 0, 0};

  // States
  ControlMode control_mode_ = ControlMode::None;
  std::array<double, 7> hw_positions_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_velocities_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_efforts_{0, 0, 0, 0, 0, 0, 0};
  bool switch_cm_ = false;

};

class FrankaMjHardwareSystem : public mujoco_ros2_control::MujocoRos2SystemInterface {
 public:
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  
  bool initSim(
    rclcpp_lifecycle::LifecycleNode::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    const mjModel* m,
    mjData* d,
    unsigned int & update_rate) override;

  const size_t kNumberOfJoints = 7;
  size_t robot_count_;

 private:
  std::shared_ptr<FrankaExecutor> executor_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Node::SharedPtr pub_node_;
  
  std::map<std::string, ArmContainer> arms_;
  std::map<std::string, franka::RobotState*> state_pointers_;
  std::map<std::string, ModelBase*> model_pointers_;
  std::map<std::string, std::shared_ptr<franka_gripper::GripperSimActionServer>> gripper_nodes_;
  std::map<std::string, std::shared_ptr<franka_gripper::GripperSharedState>> gripper_states_ptrs_;  // guarded handoff, see gripper_shared_state.hpp
  struct GripperContactBodyIds { int left_finger {-1}; int right_finger {-1}; int rod {-1}; };
  std::map<std::string, GripperContactBodyIds> gripper_contact_body_ids_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_mj_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Mujoco data pointer
  mjData* d_;
  const mjModel* m_;

  /// \brief controller update rate
  unsigned int * update_rate_;
  

  rclcpp::Clock clock_;
  static rclcpp::Logger getLogger();

  // ===== Grasp weld constraint (runtime activation) =====
  // 双weld: 左手+右手同时刚性绑定铝棒，消除单侧约束导致ROTATE脱落
  int weld_eq_id_ = -1;           // 左手 weld
  int weld_right_eq_id_ = -1;     // 右手 weld
  int hand_body_id_ = -1;         // mj_left_hand body id
  int right_hand_body_id_ = -1;   // mj_right_hand body id
  int rod_body_id_ = -1;
  std::atomic<bool> weld_active_desired_{false};
  bool weld_currently_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr weld_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr weld_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr adaptive_pub_;
  rclcpp::Node::SharedPtr weld_node_;
  void updateWeldRelpose();       // 计算左手→铝棒 relpose
  void updateWeldRelposeRight();  // 计算右手→铝棒 relpose

  const std::string k_robot_state_interface_name{"robot_state"};
  std::vector<double> hw_sensor_states_;
  std::map<std::string, std::pair<int, int>> ft_sensor_mj_ids_;
  const std::string k_robot_model_interface_name{"robot_model"};
};
}  // namespace franka_hardware
