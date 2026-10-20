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
#include <array>
#include <string>
#include <vector>
#include <fstream>

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
#include <cstdint>
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
  std::map<std::string, std::array<double, 2>> gripper_finger_damping_base_;
  struct GripperActuatorConfig {
    mjtByte dyntype {mjDYN_NONE};
    mjtByte gaintype {mjGAIN_FIXED};
    mjtByte biastype {mjBIAS_NONE};
    mjtByte ctrllimited {0};
    mjtByte forcelimited {0};
    std::array<mjtNum, mjNDYN> dynprm{};
    std::array<mjtNum, mjNGAIN> gainprm{};
    std::array<mjtNum, mjNBIAS> biasprm{};
    std::array<mjtNum, 6> gear{};
    std::array<mjtNum, 2> ctrlrange{};
    std::array<mjtNum, 2> forcerange{};
  };
  std::map<std::string, GripperActuatorConfig> gripper_actuator_config_base_;
  std::map<std::string, bool> gripper_direct_force_active_;
  // Optional closed-loop tendon-position correction for contact_probe. The
  // legacy hold is a fixed position target with a force cap; this bias lets an
  // explicitly enabled probe recover a small loss of measured pad normal force
  // without changing the nominal force request or model parameters.
  std::map<std::string, double> gripper_force_hold_bias_m_;
  struct GripperContactBodyIds {
    int left_finger {-1}; int right_finger {-1}; int rod {-1}; int rod_geom {-1}; int hand_link8 {-1};
  };
  std::map<std::string, GripperContactBodyIds> gripper_contact_body_ids_;
  // Optional contact-probe trace, written only by the MuJoCo physics thread.
  std::ofstream contact_probe_trace_;
  std::uint64_t contact_probe_trace_tick_ {0};
  bool contact_probe_single_pad_active_ {false};
  std::array<bool, 2> contact_probe_first_contact_logged_ {{false, false}};

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
  // One experiment-only +X pulse on the rod, scheduled by the physics thread.
  // The ROS callback only changes the request flag; all xfrc_applied writes,
  // timing and CSV events stay in write().
  bool rod_force_enabled_{false};
  int rod_force_body_id_{-1};
  std::atomic<bool> rod_force_arm_requested_{false};
  bool rod_force_armed_{false};
  bool rod_force_consumed_{false};
  bool rod_force_applied_{false};
  std::atomic<bool> rod_force_applied_status_{false};
  std::atomic<bool> rod_force_armed_status_{false};
  double rod_force_arm_sim_time_{0.0};
  double rod_force_start_sim_time_{0.0};
  // Applied duration, in simulation seconds. Default 0.2 preserves the
  // previously reported pulse; the long-pulse directional experiment sets
  // CONTACT_PROBE_ROD_FORCE_DURATION_SEC=1.0.
  double rod_force_duration_sec_{0.2};
  std::ofstream rod_force_events_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rod_force_arm_sub_;
  void updateRodForcePulse();
  void updateWeldRelpose();       // 计算左手→铝棒 relpose
  void updateWeldRelposeRight();  // 计算右手→铝棒 relpose

  const std::string k_robot_state_interface_name{"robot_state"};
  std::vector<double> hw_sensor_states_;
  std::map<std::string, std::pair<int, int>> ft_sensor_mj_ids_;
  const std::string k_robot_model_interface_name{"robot_model"};
};
}  // namespace franka_hardware
