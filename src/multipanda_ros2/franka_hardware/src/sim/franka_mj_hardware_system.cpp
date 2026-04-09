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

#include "franka_hardware/sim/franka_mj_hardware_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h> 
#include <fstream>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

bool FrankaMjHardwareSystem::initSim(
    rclcpp_lifecycle::LifecycleNode::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    const mjModel* m,
    mjData* d,
    unsigned int & update_rate){
    
    info_ = hardware_info;
    this->nh_ = model_nh;
    this->d_ = d;
    this->m_ = m;
    this->update_rate_ = &update_rate;
    std::stringstream rc_stream(info_.hardware_parameters.at("robot_count"));
    rc_stream >> robot_count_;

    if (info_.joints.size() != kNumberOfJoints * robot_count_) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                    kNumberOfJoints * robot_count_);
        return false;
    }

    for (const auto& joint : info_.joints) {

        // Check number of command interfaces
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                        joint.name.c_str(), joint.command_interfaces.size());
            return false;
        }

    //   // Check that the interfaces are named correctly
        for (const auto & cmd_interface : joint.command_interfaces){
            if (cmd_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
                cmd_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
                cmd_interface.name != hardware_interface::HW_IF_VELOCITY //&& // Joint velocity "velocity"
                ){                
                RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                            joint.name.c_str(), cmd_interface.name.c_str());
                return false;
            }
        }

    //   // Check number of state interfaces
        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return false;
        }

        for (const auto & state_interface : joint.state_interfaces){
            if (state_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
                state_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
                state_interface.name != hardware_interface::HW_IF_VELOCITY //&& // Joint velocity "velocity"
                ){                
                RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                            joint.name.c_str(), state_interface.name.c_str());
                return false;
            }
        }
    }
    RCLCPP_DEBUG(getLogger(), "Initial checks all passed");
    clock_ = rclcpp::Clock(RCL_ROS_TIME);

    // Set up the mujoco and RobotSim 
    // TODO: Clean up executor to use the one passed down from mj_env
    executor_ = std::make_shared<FrankaExecutor>();
    for(size_t i = 1; i <= robot_count_; i++){
        // Setup arm container
        std::string suffix = "_" + std::to_string(i);
        std::string mj_robot_name;
        try{
            mj_robot_name = info_.hardware_parameters.at("ns"+suffix);
        }
        catch(const std::out_of_range& ex){
            RCLCPP_FATAL(getLogger(), "Parameter 'ns%s' ! set\nMake sure all multi-robot parameters follow the format {ns/robot_ip/etc.}_n", suffix.c_str());
            return false;
        }
        if(!arms_.insert(std::make_pair(mj_robot_name, ArmContainer())).second){
            RCLCPP_FATAL(getLogger(),"The provided robot namespace %s already exists! Make sure they are unique.", mj_robot_name.c_str());
            return false;
        }
        auto &arm = arms_[mj_robot_name];
        state_pointers_.insert(std::make_pair(mj_robot_name, &arm.hw_franka_robot_state_));
        model_pointers_.insert(std::make_pair(mj_robot_name, nullptr));
        arm.robot_name_ = mj_robot_name;

        bool has_gripper = info_.hardware_parameters.at("hand" + suffix) == std::string("True") ? true : false;
        arm.robot_ = std::make_unique<RobotSim>(mj_robot_name, has_gripper);
        arm.robot_->franka_hardware_model_ = std::make_unique<ModelSim>(m_, d_);
        if(!arm.robot_->populateIndices()){
          RCLCPP_FATAL(getLogger(), "Populating indices failed!");
          return false;
        };

        // Start the gripper node if true
        if(arm.robot_->has_gripper_){
            RCLCPP_INFO_STREAM(getLogger(), "Gripper found for " << arm.robot_name_ << ". Creating a gripper server.");
            gripper_states_ptrs_.insert(std::make_pair(arm.robot_name_, std::make_shared<std::array<double,3>>()));
            gripper_nodes_.insert(std::make_pair(arm.robot_name_, std::make_shared<franka_gripper::GripperSimActionServer>(rclcpp::NodeOptions(), arm.robot_name_)));
            gripper_nodes_[arm.robot_name_]->initGripperPtrs(gripper_states_ptrs_[arm.robot_name_]);
            executor_->add_node(gripper_nodes_[arm.robot_name_]);
        }
        // Initialize with all the actuators set to "off"
        for(size_t i=0; i<kNumberOfJoints; i++){
            set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
            set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
            set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 0);
        }
        arm.hw_commands_joint_effort_.fill(0);
        arm.hw_commands_joint_velocity_.fill(0);

        // Check if there is a desired initial pose for this arm's joints
        for(size_t j = 7*(i-1); j < i*7; j++){
          // arm.default_arm_qpos_.push_back(std::stod(info_.joints[j].parameters.at("initial_position")));
          d_->qpos[arm.robot_->joint_qpos_indices_[j - (7*(i-1))]] = std::stod(info_.joints[j].parameters.at("initial_position"));
          RCLCPP_DEBUG(getLogger(), "Joint %ld (name %s) initial position: %f", j, info_.joints[j].name.c_str(), std::stod(info_.joints[j].parameters.at("initial_position")));
        }
        
    }

    // ===== Initialize grasp weld constraints (dual: left + right) =====
    // Initialize sensors
    int num_sensors = 0;
    for(const auto& s : info_.sensors) num_sensors += s.state_interfaces.size();
    hw_sensor_states_.resize(num_sensors, 0.0);
    for(const auto& s : info_.sensors) {
      int force_id = mj_name2id(m_, mjOBJ_SENSOR, (s.name + "_force").c_str());
      int torque_id = mj_name2id(m_, mjOBJ_SENSOR, (s.name + "_torque").c_str());
      ft_sensor_mj_ids_[s.name] = {force_id, torque_id};
    }
    
    weld_eq_id_ = mj_name2id(m_, mjOBJ_EQUALITY, "grasp_weld");
    if (weld_eq_id_ >= 0) {
        hand_body_id_ = mj_name2id(m_, mjOBJ_BODY, "mj_left_hand");
        rod_body_id_ = mj_name2id(m_, mjOBJ_BODY, "obj_rod_01");
        
        // Initialize right-hand weld constraint
        weld_right_eq_id_ = mj_name2id(m_, mjOBJ_EQUALITY, "grasp_weld_right");
        if (weld_right_eq_id_ >= 0) {
            right_hand_body_id_ = mj_name2id(m_, mjOBJ_BODY, "mj_right_hand");
            RCLCPP_INFO(getLogger(), "Right grasp weld ready: eq_id=%d, right_hand_body=%d",
                        weld_right_eq_id_, right_hand_body_id_);
        } else {
            RCLCPP_WARN(getLogger(), "Right grasp weld 'grasp_weld_right' not found (right gripper friction-only)");
        }
        
        // Create a dedicated node + subscriber for weld activation/deactivation
        weld_node_ = std::make_shared<rclcpp::Node>("weld_control_node");
        weld_sub_ = weld_node_->create_subscription<std_msgs::msg::Bool>(
            "/grasp_weld_active", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                weld_active_desired_.store(msg->data);
            });
        adaptive_pub_ = weld_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/adaptive_impedance_params", 10);
        executor_->add_node(weld_node_);
        
        RCLCPP_INFO(getLogger(), "Grasp weld constraint ready: eq_id=%d, hand_body=%d, rod_body=%d",
                    weld_eq_id_, hand_body_id_, rod_body_id_);
    } else {
        RCLCPP_WARN(getLogger(), "Grasp weld constraint 'grasp_weld' not found in model (grasping will rely on friction only)");
    }

    RCLCPP_INFO(getLogger(), "Robot successfully initialized!");
    return true;
  }
CallbackReturn FrankaMjHardwareSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}



std::vector<StateInterface> FrankaMjHardwareSystem::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;

  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arms_[get_ns(info_.joints[i].name)].hw_positions_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,  &arms_[get_ns(info_.joints[i].name)].hw_velocities_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,  &arms_[get_ns(info_.joints[i].name)].hw_efforts_.at(get_joint_no(info_.joints[i].name))));
  }

  for(auto arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_state_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &state_pointers_[arm_container_pair.first])));
    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_model_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &model_pointers_[arm_container_pair.first])));
  }

  int sensor_idx = 0;
  for (const auto& sensor : info_.sensors) {
    for (const auto& interface : sensor.state_interfaces) {
      if (sensor_idx < hw_sensor_states_.size()) {
        state_interfaces.emplace_back(StateInterface(
            sensor.name, interface.name, &hw_sensor_states_[sensor_idx++]));
      }
    }
  }

  return state_interfaces;
}

std::vector<CommandInterface> FrankaMjHardwareSystem::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface( // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_effort_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_position_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_velocity_.at(get_joint_no(info_.joints[i].name))));
  }

  return command_interfaces;
}

CallbackReturn FrankaMjHardwareSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.hw_commands_joint_effort_.fill(0);
    arm.control_mode_ = ControlMode::None;
    
    // Initialize gripper states from current MuJoCo position
    if(arm.robot_->has_gripper_){
      double current_finger_pos = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[0]];
      double current_gripper_width = current_finger_pos * 2.0; // symmetric gripper
      // at(0) = control value (0-255), at(1) = current position (meters)
      gripper_states_ptrs_[arm.robot_name_]->at(0) = current_gripper_width / 0.0003137;
      gripper_states_ptrs_[arm.robot_name_]->at(1) = current_finger_pos;
      RCLCPP_INFO(getLogger(), "Initialized %s gripper: finger_pos=%.4f, width=%.4f, ctrl=%.1f", 
                  arm.robot_name_.c_str(), current_finger_pos, current_gripper_width, 
                  gripper_states_ptrs_[arm.robot_name_]->at(0));
    }
  }
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMjHardwareSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.control_mode_ = ControlMode::None;
  }
  RCLCPP_INFO(getLogger(), "Stopped"); 
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaMjHardwareSystem::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {

//   mj_step1(m_, d_); // ideally, mj_step1 then mj_step2 in write
  for(auto& arm_container_pair: arms_){
    auto &arm = arm_container_pair.second;
    if (model_pointers_[arm_container_pair.first] == nullptr) {
      model_pointers_[arm_container_pair.first] = arm.robot_->getModel();
    }
    arm.hw_franka_robot_state_ = arm.robot_->populateFrankaState();
    arm.hw_positions_ = arm.hw_franka_robot_state_.q;
    arm.hw_velocities_ = arm.hw_franka_robot_state_.dq;
    arm.hw_efforts_ = arm.hw_franka_robot_state_.tau_J;
    if(arm.robot_->has_gripper_){
      gripper_states_ptrs_[arm.robot_name_]->at(1) = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[0]];
    }
  }
//   for(auto& obj_pair: mj_objs_){
//     updateObjectContainer(obj_pair.first);
//   }

  int current_idx = 0;
  for (const auto& sensor : info_.sensors) {
    auto ids = ft_sensor_mj_ids_[sensor.name];
    int force_id = ids.first;
    int torque_id = ids.second;
    
    // Copy force (3 axes) if sensor was found
    if (force_id >= 0 && m_->sensor_dim[force_id] == 3) {
      int adr = m_->sensor_adr[force_id];
      hw_sensor_states_[current_idx++] = d_->sensordata[adr];
      hw_sensor_states_[current_idx++] = d_->sensordata[adr+1];
      hw_sensor_states_[current_idx++] = d_->sensordata[adr+2];
    } else {
      current_idx += 3;
    }
    
    // Copy torque (3 axes)
    if (torque_id >= 0 && m_->sensor_dim[torque_id] == 3) {
      int adr = m_->sensor_adr[torque_id];
      hw_sensor_states_[current_idx++] = d_->sensordata[adr];
      hw_sensor_states_[current_idx++] = d_->sensordata[adr+1];
      hw_sensor_states_[current_idx++] = d_->sensordata[adr+2];
    } else {
      current_idx += 3;
    }
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMjHardwareSystem::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
//   std::lock_guard<std::mutex> guard(mj_mutex_);
  // ---------------- ADAPTIVE ADMITTANCE CALCULATION ----------------
  static int loop_cnt = 0;
  static double current_K_scale = 1.0;
  static double current_D_scale = 1.0;
  static const bool internal_adaptive_enabled = []() {
    const char* raw = std::getenv("MJ_INTERNAL_COMPLIANCE_ENABLED");
    if (raw == nullptr) {
      return true;
    }
    std::string value(raw);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return !(value == "0" || value == "false" || value == "off" || value == "no");
  }();
  static const double internal_stress_threshold = []() {
    const char* raw = std::getenv("MJ_INTERNAL_COMPLIANCE_STRESS_THRESHOLD");
    return raw ? std::max(0.0, std::atof(raw)) : 2.0;
  }();
  static const double internal_k_gain = []() {
    const char* raw = std::getenv("MJ_INTERNAL_COMPLIANCE_K_GAIN");
    return raw ? std::max(0.0, std::atof(raw)) : 0.08;
  }();
  static const double internal_d_gain = []() {
    const char* raw = std::getenv("MJ_INTERNAL_COMPLIANCE_D_GAIN");
    return raw ? std::max(0.0, std::atof(raw)) : 0.15;
  }();
  static const double internal_min_k = []() {
    const char* raw = std::getenv("MJ_INTERNAL_COMPLIANCE_MIN_K");
    return raw ? std::clamp(std::atof(raw), 0.05, 1.0) : 0.10;
  }();
  static const double internal_max_d = []() {
    const char* raw = std::getenv("MJ_INTERNAL_COMPLIANCE_MAX_D");
    return raw ? std::max(1.0, std::atof(raw)) : 2.5;
  }();
  static bool adaptive_mode_logged = false;

  if (!adaptive_mode_logged) {
    RCLCPP_INFO(getLogger(),
                "MuJoCo internal compliance: %s | threshold=%.2f K_gain=%.3f D_gain=%.3f min_K=%.2f max_D=%.2f",
                internal_adaptive_enabled ? "enabled" : "disabled",
                internal_stress_threshold, internal_k_gain, internal_d_gain,
                internal_min_k, internal_max_d);
    adaptive_mode_logged = true;
  }

  // 获取左右臂的FT传感器ID
  int l_force_id = mj_name2id(m_, mjOBJ_SENSOR, "mj_left_hand_ft_sensor_force");
  int r_force_id = mj_name2id(m_, mjOBJ_SENSOR, "mj_right_hand_ft_sensor_force");

  if (internal_adaptive_enabled && l_force_id >= 0 && r_force_id >= 0) {
      int l_adr = m_->sensor_adr[l_force_id];
      int r_adr = m_->sensor_adr[r_force_id];
      double ly = d_->sensordata[l_adr + 1]; 
      double ry = d_->sensordata[r_adr + 1];
      
      double internal_stress = std::abs(ly + ry) / 2.0;

      double target_K = 1.0;
      double target_D = 1.0;
      if (internal_stress > internal_stress_threshold) {
          const double stress_over = internal_stress - internal_stress_threshold;
          target_K = std::max(internal_min_k, 1.0 - internal_k_gain * stress_over);
          target_D = std::min(internal_max_d, 1.0 + internal_d_gain * stress_over);
      }

      current_K_scale = 0.95 * current_K_scale + 0.05 * target_K;
      current_D_scale = 0.95 * current_D_scale + 0.05 * target_D;
      
      if (loop_cnt % 10 == 0 && adaptive_pub_) {
          std_msgs::msg::Float64MultiArray msg;
          msg.data.push_back(current_K_scale * 100.0);
          msg.data.push_back(current_D_scale * 100.0);
          adaptive_pub_->publish(msg);
      }
  } else {
      current_K_scale = 1.0;
      current_D_scale = 1.0;
      if (loop_cnt % 10 == 0 && adaptive_pub_) {
          std_msgs::msg::Float64MultiArray msg;
          msg.data.push_back(100.0);
          msg.data.push_back(100.0);
          adaptive_pub_->publish(msg);
      }
  }
  loop_cnt++;
  // -----------------------------------------------------------------
  for(auto& arm_container_pair: arms_){
    auto &arm = arm_container_pair.second;
    if (std::any_of(arm.hw_commands_joint_effort_.begin(), arm.hw_commands_joint_effort_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::any_of(arm.hw_commands_joint_position_.begin(), arm.hw_commands_joint_position_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::any_of(arm.hw_commands_joint_velocity_.begin(), arm.hw_commands_joint_velocity_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }

    switch(arm.control_mode_){
      case ControlMode::JointTorque:
        for(size_t i = 0; i < kNumberOfJoints; i++){
          d_->ctrl[arm.robot_->act_trq_indices_[i]] = arm.hw_commands_joint_effort_[i];
        }
        break;
      case ControlMode::JointPosition:
      {
        static const double kp_gains[7] = {300.0, 300.0, 200.0, 200.0, 80.0, 80.0, 80.0};
        static const double kv_gains[7] = { 40.0,  40.0,  25.0,  25.0, 10.0, 10.0, 10.0};

        for(size_t i = 0; i < kNumberOfJoints; i++){
          d_->ctrl[arm.robot_->act_pos_indices_[i]] = arm.hw_commands_joint_position_[i];
          d_->ctrl[arm.robot_->act_vel_indices_[i]] = 0.0;  // velocity servo ctrl=0 → τ_d = -kv·q̇（PD的D项）
          d_->ctrl[arm.robot_->act_trq_indices_[i]] = 0.0;  // 确保力矩通道关闭

          // 根据自适应因子动态调整 PD 参数 (实现自适应导纳)
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], kp_gains[i] * current_K_scale);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], kv_gains[i] * current_D_scale);
        }
        break;
      }
      case ControlMode::JointVelocity:
        for(size_t i = 0; i < kNumberOfJoints; i++){
          d_->ctrl[arm.robot_->act_vel_indices_[i]] = arm.hw_commands_joint_velocity_[i];
        }
        break;
      default:
        break;
    }
    if(arm.robot_->has_gripper_){
      d_->ctrl[arm.robot_->gripper_act_idx_] = gripper_states_ptrs_[arm.robot_name_]->at(0);
    }
  }

  // ===== Toggle grasp weld constraints (dual: left + right) =====
  if (weld_eq_id_ >= 0) {
    bool desired = weld_active_desired_.load();
    if (desired && !weld_currently_active_) {
      // Activate left-hand weld
      updateWeldRelpose();
      d_->eq_active[weld_eq_id_] = 1;
      // Activate right-hand weld (if available)
      if (weld_right_eq_id_ >= 0) {
        updateWeldRelposeRight();
        d_->eq_active[weld_right_eq_id_] = 1;
      }
      weld_currently_active_ = true;
      RCLCPP_INFO(getLogger(), "Grasp weld ACTIVATED (left%s)",
                  weld_right_eq_id_ >= 0 ? " + right)" : " only)");
    } else if (!desired && weld_currently_active_) {
      d_->eq_active[weld_eq_id_] = 0;
      if (weld_right_eq_id_ >= 0) {
        d_->eq_active[weld_right_eq_id_] = 0;
      }
      weld_currently_active_ = false;
      RCLCPP_INFO(getLogger(), "Grasp weld DEACTIVATED (left%s)",
                  weld_right_eq_id_ >= 0 ? " + right)" : " only)");
    }
  }

//   if(mj_pose_service_node_->hasUpdates()){
//     auto new_object_poses = mj_pose_service_node_->getNewObjectPoses();
//     for(auto& new_obj : new_object_poses){
//       changeMjObjPose(new_obj.obj_name_,new_obj.x,new_obj.y,new_obj.z,new_obj.qx,new_obj.qy,new_obj.qz,new_obj.qw);
//     }
//     mj_pose_service_node_->setHasUpdatesToFalse();
//   }
//   mj_step2(m_, d_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMjHardwareSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(this->getLogger(),"Preparing command mode switch");

  bool is_effort;
  bool is_position;
  bool is_velocity;
  bool is_duplicate;
  
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //    Check if the incoming interfaces are relevant or not    //
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //              Handle the stop case first                    //
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  /* copilot answer
  #include <iostream>
#include <string>

bool startsWith(const std::string& mainStr, const std::string& toMatch) {
    if (mainStr.size() < toMatch.size()) {
        return false;
    }
    return mainStr.compare(0, toMatch.size(), toMatch) == 0;
}

int main() {
    std::string str1 = "garmi_left_base/velocity";
    std::string str2 = "left/";

    if (startsWith(str1, str2)) {
        std::cout << "The string \"" << str1 << "\" starts with \"" << str2 << "\"." << std::endl;
    } else {
        std::cout << "The string \"" << str1 << "\" does not start with \"" << str2 << "\"." << std::endl;
    }

    return 0;
}

  
  */
  // selective control mode handling
  for(auto& arm : arms_){
    std::vector<std::string> arm_stop_interfaces;
    const std::string arm_ns = arm.first + "_joint";
    // query what is the requested stop interface's arm
    for(auto& stop : stop_interfaces){
      if(startsWith(stop, arm_ns)){ // this basically has to check if the interface's first part is basically "robot_name/", else ignore
        arm_stop_interfaces.push_back(stop);
      }
    }
    int stop_type = check_command_mode_type(arm_stop_interfaces);
    if(stop_type > 0){
      // If stop_type is not empty, i.e. 1 or 2
      is_effort = all_of_element_has_string(stop_interfaces, "effort");
      is_position = all_of_element_has_string(stop_interfaces, "position");
      is_velocity = all_of_element_has_string(stop_interfaces, "velocity");
      is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
      if(!(is_effort || is_position || is_velocity)){
        RCLCPP_ERROR(this->getLogger(), "Requested stop interface is not a supported type!\n" \
                                    "Please make sure they are either effort, position or velocity.");
        return hardware_interface::return_type::ERROR;
      }
      if((is_duplicate)){
        RCLCPP_ERROR(this->getLogger(), "Requested stop interface has a confusing name!\n" \
                                    "Please make sure they are either effort, position or velocity, only.");
        return hardware_interface::return_type::ERROR;
      }
    }
    switch(stop_type){
      case -1: 
        RCLCPP_ERROR(this->getLogger(), "Requested stop interfaces do not all have the same type!\n" \
                                      "Please make sure they are either Cartesian or Joint.");
        return hardware_interface::return_type::ERROR;
      
      case 0: // if size is 0, move on to the next arm
        continue;
        break;

      case 1: // stop the joint controllers
        // Make sure there are 7
        if(arm_stop_interfaces.size() != kNumberOfJoints){
          RCLCPP_ERROR(this->getLogger(), "Requested joint stop interface's size is not 7 (got %ld)", arm_stop_interfaces.size());
          return hardware_interface::return_type::ERROR;
        }
        for(size_t i = 0 ; i < kNumberOfJoints; i++){
          if(is_effort){
            arm.second.hw_commands_joint_effort_[i] = 0;
          }
          // position command is not reset, since that can be dangerous
          else if(is_velocity){
            arm.second.hw_commands_joint_velocity_[i] = 0;
          }
        }
        arm.second.control_mode_ = ControlMode::None;
        arm.second.switch_cm_ = true;
        break;

      case 2: // Cartesian is not supported for simulation
        RCLCPP_ERROR(this->getLogger(), "Cartesian interfaces are not supported for simulation!\n" \
                                      "Please make sure they are Joint.");
        return hardware_interface::return_type::ERROR;

      default:
        break;
    }
  }

  // ////////////////////////////////////////////////////////////////
  // //              Verify that the names are valid               //
  // ////////////////////////////////////////////////////////////////

  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //              Handle the start case                         //
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  
  for(auto& arm : arms_){
    std::vector<std::string> arm_start_interfaces;
    const std::string arm_ns = arm.first + "_joint";
    // query what is the requested stop interface's arm
    for(auto& start : start_interfaces){
      if(startsWith(start, arm_ns)){
        arm_start_interfaces.push_back(start);
      }
    }
    int start_type = check_command_mode_type(arm_start_interfaces);
    // ////////////////////////////////////////////////////////////////
    // //              Verify that the names are valid               //
    // ////////////////////////////////////////////////////////////////
    if(start_type > 0){
      // If start_type is not empty, i.e. 1 or 2
      is_effort = all_of_element_has_string(start_interfaces, "effort");
      is_position = all_of_element_has_string(start_interfaces, "position");
      is_velocity = all_of_element_has_string(start_interfaces, "velocity");
      is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
      if(!(is_effort || is_position || is_velocity)){
        RCLCPP_ERROR(this->getLogger(), "Requested start interface is not a supported type!\n" \
                                    "Please make sure they are either effort, position or velocity.");
        return hardware_interface::return_type::ERROR;
      }
      if((is_duplicate)){
        RCLCPP_ERROR(this->getLogger(), "Requested start interface has a confusing name!\n" \
                                    "Please make sure they are either effort, position or velocity, only.");
        return hardware_interface::return_type::ERROR;
      }
      // Just in case: only allow new controller to be started if the current control mode is NONE
      if(arm.second.control_mode_ != ControlMode::None){
        RCLCPP_ERROR(this->getLogger(), "Switching between control modes without stopping it first is not supported.\n"\
                                        "Please stop the running controller first.");
        return hardware_interface::return_type::ERROR;
      }
    }
    switch(start_type){
      case 0: // if size is 0, move on to the next arm
        continue;
        break;

      case -1:
        RCLCPP_ERROR(this->getLogger(), "Requested start interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
        return hardware_interface::return_type::ERROR;
      case 1:
        if(arm_start_interfaces.size() != kNumberOfJoints){
          RCLCPP_ERROR(this->getLogger(), "Requested joint start interface's size is not %ld (got %ld)", kNumberOfJoints, arm_start_interfaces.size());
          return hardware_interface::return_type::ERROR;
        }
        if(is_effort){
          arm.second.control_mode_ = ControlMode::JointTorque;
          arm.second.switch_cm_ = true;
        }
        if(is_position){
          arm.second.control_mode_ = ControlMode::JointPosition;
          arm.second.switch_cm_ = true;
        }
        if(is_velocity){
          arm.second.control_mode_ = ControlMode::JointVelocity;
          arm.second.switch_cm_ = true;
        }
        break;

      case 2: // Cartesian is not supported for simulation
        RCLCPP_ERROR(this->getLogger(), "Cartesian interfaces are not supported for simulation!\n" \
                                        "Please make sure they are Joint.");
        return hardware_interface::return_type::ERROR;

      default:
        break;
    }
  }
  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FrankaMjHardwareSystem::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {

  RCLCPP_INFO(this->getLogger(),"Performing command mode switch");

  for(auto &arm_p : arms_){
    auto& arm = arm_p.second;
    RCLCPP_INFO_STREAM(this->getLogger(), "Arm " << arm.robot_name_ << 
                                          " current mode: " << arm.control_mode_);
    if(arm.switch_cm_){
      if(arm.control_mode_ == ControlMode::None){
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 0);
        }
      }
      else if(arm.control_mode_ == ControlMode::JointTorque){
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 1);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 0);
        }
      }
      else if(arm.control_mode_ == ControlMode::JointPosition){
        // PD控制器 —— 按关节分级设置增益
        // 设计原则：
        //   J1-J2（基部）：高刚度快速响应，承担支撑固定，ζ≈0.9
        //   J3-J4（中段）：减小超调和稳态误差，避免大幅震荡，ζ≈1.0-1.2
        //   J5-J7（末端）：力矩受限，保精度绝不谐振，ζ>1.3（过阻尼）
        static const double kp_gains[7] = {300.0, 300.0, 200.0, 200.0, 80.0, 80.0, 80.0};
        static const double kv_gains[7] = { 40.0,  40.0,  25.0,  25.0, 10.0, 10.0, 10.0};
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], kp_gains[i]);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], kv_gains[i]);
        }
        RCLCPP_INFO(this->getLogger(), "PD controller activated for %s: J1-2 kp=300/kv=40, J3-4 kp=200/kv=25, J5-7 kp=80/kv=10", arm.robot_name_.c_str());
      }
      else if(arm.control_mode_ == ControlMode::JointVelocity){
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 10);
        }
      }
      arm.switch_cm_ = false;
    }
  }


  return hardware_interface::return_type::OK;
}

rclcpp::Logger FrankaMjHardwareSystem::getLogger() {
  return rclcpp::get_logger("FrankaMjHardwareSystem");
}

void FrankaMjHardwareSystem::updateWeldRelpose() {
  if (hand_body_id_ < 0 || rod_body_id_ < 0) return;

  // Get world poses from MuJoCo forward kinematics
  const double* hand_pos = d_->xpos + 3 * hand_body_id_;
  const double* hand_quat = d_->xquat + 4 * hand_body_id_;
  const double* rod_pos = d_->xpos + 3 * rod_body_id_;
  const double* rod_quat = d_->xquat + 4 * rod_body_id_;

  // Compute relative position: rod position in hand's local frame
  double dp[3];
  mju_sub3(dp, rod_pos, hand_pos);        // dp = rod_pos - hand_pos (world frame)
  double hand_quat_conj[4];
  mju_negQuat(hand_quat_conj, hand_quat); // conjugate = inverse rotation
  double rel_pos[3];
  mju_rotVecQuat(rel_pos, dp, hand_quat_conj); // rotate to hand's local frame

  // Compute relative quaternion: rod orientation in hand's local frame
  double rel_quat[4];
  mju_mulQuat(rel_quat, hand_quat_conj, rod_quat);

  // Write to equality constraint data
  // Weld eq_data layout: [0:3]=anchor, [3:6]=rel_pos, [6:10]=rel_quat(w,x,y,z), [10]=torquescale
  mjModel* m_mut = const_cast<mjModel*>(m_);
  int base = mjNEQDATA * weld_eq_id_;
  // Anchor at body1 origin
  m_mut->eq_data[base + 0] = 0.0;
  m_mut->eq_data[base + 1] = 0.0;
  m_mut->eq_data[base + 2] = 0.0;
  // Relative position
  m_mut->eq_data[base + 3] = rel_pos[0];
  m_mut->eq_data[base + 4] = rel_pos[1];
  m_mut->eq_data[base + 5] = rel_pos[2];
  // Relative quaternion (MuJoCo convention: w, x, y, z)
  m_mut->eq_data[base + 6] = rel_quat[0];
  m_mut->eq_data[base + 7] = rel_quat[1];
  m_mut->eq_data[base + 8] = rel_quat[2];
  m_mut->eq_data[base + 9] = rel_quat[3];
  // Torque scale
  m_mut->eq_data[base + 10] = 1.0;

  RCLCPP_INFO(getLogger(), "Weld LEFT relpose: pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
              rel_pos[0], rel_pos[1], rel_pos[2],
              rel_quat[0], rel_quat[1], rel_quat[2], rel_quat[3]);
}

void FrankaMjHardwareSystem::updateWeldRelposeRight() {
  if (right_hand_body_id_ < 0 || rod_body_id_ < 0) return;

  // Get world poses from MuJoCo forward kinematics
  const double* hand_pos = d_->xpos + 3 * right_hand_body_id_;
  const double* hand_quat = d_->xquat + 4 * right_hand_body_id_;
  const double* rod_pos = d_->xpos + 3 * rod_body_id_;
  const double* rod_quat = d_->xquat + 4 * rod_body_id_;

  // Compute relative position: rod position in right hand's local frame
  double dp[3];
  mju_sub3(dp, rod_pos, hand_pos);
  double hand_quat_conj[4];
  mju_negQuat(hand_quat_conj, hand_quat);
  double rel_pos[3];
  mju_rotVecQuat(rel_pos, dp, hand_quat_conj);

  // Compute relative quaternion: rod orientation in right hand's local frame
  double rel_quat[4];
  mju_mulQuat(rel_quat, hand_quat_conj, rod_quat);

  // Write to equality constraint data
  mjModel* m_mut = const_cast<mjModel*>(m_);
  int base = mjNEQDATA * weld_right_eq_id_;
  m_mut->eq_data[base + 0] = 0.0;
  m_mut->eq_data[base + 1] = 0.0;
  m_mut->eq_data[base + 2] = 0.0;
  m_mut->eq_data[base + 3] = rel_pos[0];
  m_mut->eq_data[base + 4] = rel_pos[1];
  m_mut->eq_data[base + 5] = rel_pos[2];
  m_mut->eq_data[base + 6] = rel_quat[0];
  m_mut->eq_data[base + 7] = rel_quat[1];
  m_mut->eq_data[base + 8] = rel_quat[2];
  m_mut->eq_data[base + 9] = rel_quat[3];
  m_mut->eq_data[base + 10] = 1.0;

  RCLCPP_INFO(getLogger(), "Weld RIGHT relpose: pos=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
              rel_pos[0], rel_pos[1], rel_pos[2],
              rel_quat[0], rel_quat[1], rel_quat[2], rel_quat[3]);
}

void set_torque_control(const mjModel* m,int actuator_no,int flag)
{
  if (flag==0)
    m->actuator_gainprm[10*actuator_no+0]=0;
  else
    m->actuator_gainprm[10*actuator_no+0]=1;
}
void set_position_servo(const mjModel* m,int actuator_no,double kp)
{
  m->actuator_gainprm[10*actuator_no+0]=kp;
  m->actuator_biasprm[10*actuator_no+1]=-kp;
}
/******************************/

/******************************/
void set_velocity_servo(const mjModel* m,int actuator_no,double kv)
{
  m->actuator_gainprm[10*actuator_no+0]=kv;
  m->actuator_biasprm[10*actuator_no+2]=-kv;
}

}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaMjHardwareSystem,
                       mujoco_ros2_control::MujocoRos2SystemInterface)
