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
#include "franka_hardware/sim/contact_probe_contact_filter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <limits>
#include <string>
#include <thread>
#include <vector>

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

namespace {

struct ContactProbeFilterState {
  const mjModel* model {nullptr};
  int rod_geom {-1};
  std::vector<int> finger_mesh_geoms;
  mjfConFilt previous_callback {nullptr};
  std::atomic<std::uint64_t> filtered_pairs {0};
  std::atomic<bool> enabled {false};
};

ContactProbeFilterState g_contact_probe_filter_state;

int contactProbeContactFilter(const mjModel* model, mjData* data,
                              int geom1, int geom2) {
  const auto& state = g_contact_probe_filter_state;
  const bool previous_discarded = state.previous_callback != nullptr &&
      state.previous_callback != contactProbeContactFilter &&
      state.previous_callback(model, data, geom1, geom2) != 0;
  if (!model || geom1 < 0 || geom2 < 0 ||
      geom1 >= model->ngeom || geom2 >= model->ngeom) {
    return previous_discarded ? 1 : 0;
  }
  const bool same_model = state.enabled.load(std::memory_order_relaxed) &&
      model == state.model;
  const bool mesh_rule = same_model &&
      franka_gripper::isContactProbeMeshRodPair(
          geom1, geom2, state.rod_geom,
          state.finger_mesh_geoms.data(), state.finger_mesh_geoms.size());
  const bool discard = franka_gripper::shouldDiscardContactProbePair(
      previous_discarded, same_model,
      geom1, model->geom_contype[geom1], model->geom_conaffinity[geom1],
      geom2, model->geom_contype[geom2], model->geom_conaffinity[geom2],
      state.rod_geom,
      state.finger_mesh_geoms.data(), state.finger_mesh_geoms.size());
  if (!previous_discarded && mesh_rule) {
    g_contact_probe_filter_state.filtered_pairs.fetch_add(1, std::memory_order_relaxed);
  }
  return discard ? 1 : 0;
}

const char* contactGeomRole(const mjModel* model, int geom, int rod_geom) {
  if (geom == rod_geom) return "obj_rod_01";
  if (geom < 0 || geom >= model->ngeom) return "invalid_geom";
  const char* body_name = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[geom]);
  if (body_name == nullptr || std::string(body_name).find("finger") == std::string::npos) {
    return "environment_or_other";
  }
  if (model->geom_type[geom] == mjGEOM_MESH) return "finger_collision_mesh";
  if (model->geom_type[geom] == mjGEOM_BOX && model->geom_group[geom] == 3) {
    return model->geom_contype[geom] == 0 && model->geom_conaffinity[geom] == 0
        ? "disabled_aux_pad_box" : "finger_pad_box";
  }
  return "finger_other_geom";
}

}  // namespace

bool FrankaMjHardwareSystem::initSim(
    rclcpp_lifecycle::LifecycleNode::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    const mjModel* m,
    mjData* d,
    unsigned int & update_rate){
    
    info_ = hardware_info;
    this->nh_ = model_nh;
    this->m_ = m;
    this->d_ = d;
    RCLCPP_INFO(getLogger(),
        "[contact_probe] solver options: cone=%d noslip_iterations=%d "
        "noslip_tolerance=%.3e impratio=%.3f iterations=%d timestep=%.4f",
        static_cast<int>(m_->opt.cone), m_->opt.noslip_iterations,
        m_->opt.noslip_tolerance, m_->opt.impratio, m_->opt.iterations,
        m_->opt.timestep);

    if (const char* trace_path = std::getenv("CONTACT_PROBE_CONTACT_TRACE_CSV")) {
      if (*trace_path != '\0') {
        contact_probe_trace_.open(trace_path, std::ios::out | std::ios::trunc);
        if (!contact_probe_trace_) {
          RCLCPP_ERROR(getLogger(), "[contact_probe] cannot open contact trace CSV: %s", trace_path);
          return false;
        }
        contact_probe_trace_ << "sim_time,arm,contact_index,geom1_id,geom1,geom1_role,body1_id,body1,geom2_id,geom2,geom2_role,body2_id,body2,finger_side,"
            "contact_x,contact_y,contact_z,normal_x,normal_y,normal_z,"
            "contact_dim,contact_dist,contact_includemargin,friction0,friction1,friction2,friction3,friction4,"
            "wrench_n,wrench_t1,wrench_t2,wrench_tau1,wrench_tau2,wrench_tau3,"
            "rod_force_x,rod_force_y,rod_force_z,force_included,"
            "q1_m,q2_m,dq1_m_s,dq2_m_s,actuator_id,actuator_ctrl_m,actuator_force_n,"
            "actuator_force_limit_abs_n,tendon_length_m,hold_force_per_finger_n,"
            "hold_reference_tendon_length_m,finger_damping_q1_n_s_per_m,"
            "finger_damping_q2_n_s_per_m,hand_x,hand_y,hand_z,rod_x,rod_y,rod_z,"
            "rod_qw,rod_qx,rod_qy,rod_qz,feedback_seq,weld_left,weld_right,rod_constraint,"
            "actuator_drive_mode,actuator_gaintype,actuator_biastype,actuator_gain0,actuator_bias0,"
            "actuator_bias1,actuator_bias2,contact_solref0,contact_solref1,contact_solimp0,"
            "contact_solimp1,contact_solimp2,contact_solimp3,contact_solimp4\n";
        RCLCPP_INFO(getLogger(), "[contact_probe] per-contact MuJoCo trace: %s", trace_path);
      }
    }

    // Contact-probe-only geometry profile. Keep the original MuJoCo model for
    // ideal_weld and other modes; when explicitly requested, disable the four
    // small auxiliary fingertip boxes and retain the primary pad per finger.
    // The auxiliary boxes are identified by box type, collision group 3, and
    // their 3 mm x-size; the primary pad remains enabled.
    const char* single_pad_env = std::getenv("CONTACT_PROBE_SINGLE_PAD");
    const bool single_primary_pad = single_pad_env != nullptr &&
        std::string(single_pad_env) == "true";
    contact_probe_single_pad_active_ = single_primary_pad;
    contact_probe_first_contact_logged_ = {{false, false}};
    if (single_primary_pad) {
      auto* mutable_model = const_cast<mjModel*>(m_);
      int disabled_auxiliary_pads = 0;
      int tuned_primary_pads = 0;
      for (int geom = 0; geom < mutable_model->ngeom; ++geom) {
        if (mutable_model->geom_type[geom] != mjGEOM_BOX ||
            mutable_model->geom_group[geom] != 3) {
          continue;
        }
        const int body = mutable_model->geom_bodyid[geom];
        const char* body_name = mj_id2name(mutable_model, mjOBJ_BODY, body);
        if (body_name == nullptr ||
            (std::string(body_name).find("_left_finger") == std::string::npos &&
             std::string(body_name).find("_right_finger") == std::string::npos)) {
          continue;
        }
        if (mutable_model->geom_size[3 * geom] < 0.004) {
          mutable_model->geom_contype[geom] = 0;
          mutable_model->geom_conaffinity[geom] = 0;
          ++disabled_auxiliary_pads;
        } else {
          // One slightly broader, softer primary pad per finger. Friction is
          // intentionally unchanged at the model value (3.0); only the
          // contact footprint and normal settling are adjusted to avoid edge
          // contact and solver impulse spikes in the probe.
          //
          // 2026-10-17: a 0.018 m z extent was tried to enlarge the slip budget
          // (the contact trace showed the pad climbing to the rod's top edge),
          // but it broke the closing window (right gripper deadline, left
          // single-finger contact). Reverted to 0.010 m; the slip-budget
          // hypothesis is tested by lift speed instead.
          mutable_model->geom_size[3 * geom] = 0.010;
          mutable_model->geom_size[3 * geom + 1] = 0.006;
          mutable_model->geom_size[3 * geom + 2] = 0.010;
          mutable_model->geom_solref[2 * geom] = 0.02;
          mutable_model->geom_solref[2 * geom + 1] = 1.0;
          // A 0.002 m pad margin was tried to improve contact continuity after
          // NoSlip=20; it made the hold worse (left finger lost contact, 12 mm
          // slip), so the model margin (0.001) is kept.
          ++tuned_primary_pads;
        }
      }
      RCLCPP_INFO(getLogger(),
          "[contact_probe] single-primary-pad profile: disabled %d auxiliary pad geoms, tuned %d primary pads",
          disabled_auxiliary_pads, tuned_primary_pads);
    }
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
            const int gripper_act = arm.robot_->gripper_act_idx_;
            GripperActuatorConfig actuator_base;
            actuator_base.dyntype = m_->actuator_dyntype[gripper_act];
            actuator_base.gaintype = m_->actuator_gaintype[gripper_act];
            actuator_base.biastype = m_->actuator_biastype[gripper_act];
            actuator_base.ctrllimited = m_->actuator_ctrllimited[gripper_act];
            actuator_base.forcelimited = m_->actuator_forcelimited[gripper_act];
            std::copy_n(m_->actuator_dynprm + mjNDYN * gripper_act, mjNDYN, actuator_base.dynprm.begin());
            std::copy_n(m_->actuator_gainprm + mjNGAIN * gripper_act, mjNGAIN, actuator_base.gainprm.begin());
            std::copy_n(m_->actuator_biasprm + mjNBIAS * gripper_act, mjNBIAS, actuator_base.biasprm.begin());
            std::copy_n(m_->actuator_gear + 6 * gripper_act, 6, actuator_base.gear.begin());
            std::copy_n(m_->actuator_ctrlrange + 2 * gripper_act, 2, actuator_base.ctrlrange.begin());
            std::copy_n(m_->actuator_forcerange + 2 * gripper_act, 2, actuator_base.forcerange.begin());
            gripper_actuator_config_base_[arm.robot_name_] = actuator_base;
            gripper_direct_force_active_[arm.robot_name_] = false;
    gripper_force_hold_bias_m_[arm.robot_name_] = 0.0;
            gripper_finger_damping_base_[arm.robot_name_] = {
                m_->dof_damping[arm.robot_->gripper_joint_qvel_indices_[0]],
                m_->dof_damping[arm.robot_->gripper_joint_qvel_indices_[1]]};
            RCLCPP_INFO_STREAM(getLogger(), "Gripper found for " << arm.robot_name_ << ". Creating a gripper server.");
            gripper_states_ptrs_.insert(std::make_pair(arm.robot_name_, std::make_shared<franka_gripper::GripperSharedState>()));
            gripper_nodes_.insert(std::make_pair(arm.robot_name_, std::make_shared<franka_gripper::GripperSimActionServer>(rclcpp::NodeOptions(), arm.robot_name_)));
            gripper_nodes_[arm.robot_name_]->initGripperPtrs(gripper_states_ptrs_[arm.robot_name_]);
            // Validate the model before enabling the optional force-hold command.
            const int act = arm.robot_->gripper_act_idx_;
            bool supported = m_->actuator_trntype[act] == mjTRN_TENDON &&
                m_->actuator_dyntype[act] == mjDYN_NONE && m_->actuator_ctrllimited[act] &&
                m_->actuator_forcelimited[act] &&
                m_->actuator_ctrlrange[2*act] == 0.0 && m_->actuator_ctrlrange[2*act+1] >= 0.04 &&
                m_->actuator_gaintype[act] == mjGAIN_FIXED &&
                m_->actuator_biastype[act] == mjBIAS_AFFINE &&
                m_->actuator_gainprm[mjNGAIN*act] > 0.0 &&
                std::abs(m_->actuator_biasprm[mjNBIAS*act]) < 1e-12 &&
                std::abs(m_->actuator_biasprm[mjNBIAS*act+1] +
                         m_->actuator_gainprm[mjNGAIN*act]) < 1e-9 &&
                std::abs(m_->actuator_biasprm[mjNBIAS*act+2]) < 1e-12 &&
                std::abs(m_->actuator_gear[6*act]-1.0) < 1e-12;
            if (supported) {
              const int tendon = m_->actuator_trnid[2*act];
              supported = tendon >= 0 && m_->tendon_num[tendon] == 2;
              if (supported) for (int j=0; j<2; ++j) {
                const int wrap = m_->tendon_adr[tendon]+j;
                supported = supported && m_->wrap_type[wrap] == mjWRAP_JOINT &&
                    std::abs(m_->wrap_prm[wrap]-0.5) < 1e-12 &&
                    m_->wrap_objid[wrap] == arm.robot_->gripper_joint_indices_[j];
              }
            }
            gripper_states_ptrs_[arm.robot_name_]->force_hold_supported = supported;
            RCLCPP_INFO(getLogger(), "Force-hold model %s supported=%d kp=%.1f tendon ratio=0.5/0.5",
                arm.robot_name_.c_str(), supported, m_->actuator_gainprm[mjNGAIN*act]);
            RCLCPP_INFO(getLogger(),
                "[gripper_actuator_model] arm=%s dyntype=%d gaintype=%d biastype=%d "
                "dynprm0=%.9g gainprm0=%.9g biasprm=(%.9g,%.9g,%.9g) gear0=%.9g "
                "ctrlrange=(%.9g,%.9g) ctrllimited=%d forcerange=(%.9g,%.9g) forcelimited=%d",
                arm.robot_name_.c_str(), static_cast<int>(actuator_base.dyntype),
                static_cast<int>(actuator_base.gaintype), static_cast<int>(actuator_base.biastype),
                actuator_base.dynprm[0], actuator_base.gainprm[0], actuator_base.biasprm[0],
                actuator_base.biasprm[1], actuator_base.biasprm[2], actuator_base.gear[0],
                actuator_base.ctrlrange[0], actuator_base.ctrlrange[1],
                static_cast<int>(actuator_base.ctrllimited), actuator_base.forcerange[0],
                actuator_base.forcerange[1], static_cast<int>(actuator_base.forcelimited));
            executor_->add_node(gripper_nodes_[arm.robot_name_]);
            GripperContactBodyIds contact_ids;
            contact_ids.left_finger = mj_name2id(m_, mjOBJ_BODY,
                (arm.robot_name_ + "_left_finger").c_str());
            contact_ids.right_finger = mj_name2id(m_, mjOBJ_BODY,
                (arm.robot_name_ + "_right_finger").c_str());
            contact_ids.rod = mj_name2id(m_, mjOBJ_BODY, "obj_rod_01");
            contact_ids.rod_geom = mj_name2id(m_, mjOBJ_GEOM, "obj_rod_01");
            contact_ids.hand_link8 = mj_name2id(
                m_, mjOBJ_BODY, (arm.robot_name_ + "_link8").c_str());
            gripper_contact_body_ids_[arm.robot_name_] = contact_ids;
            RCLCPP_INFO(getLogger(),
                "Gripper contact bodies %s: left=%d right=%d rod(obj_rod_01)=%d",
                arm.robot_name_.c_str(), contact_ids.left_finger,
                contact_ids.right_finger, contact_ids.rod);
            for (int geom = 0; geom < m_->ngeom; ++geom) {
              const int body = m_->geom_bodyid[geom];
              if (body == contact_ids.left_finger || body == contact_ids.right_finger ||
                  body == contact_ids.rod) {
                const char* geom_name = mj_id2name(m_, mjOBJ_GEOM, geom);
                RCLCPP_INFO(getLogger(),
                    "  contact geom id=%d name=%s role=%s body_id=%d contype=%d conaffinity=%d "
                    "size=(%.5f,%.5f,%.5f) friction=(%.4f,%.4f,%.4f) condim=%d margin=%.5f "
                    "solref=(%.5f,%.5f) solimp=(%.5f,%.5f,%.5f,%.5f,%.5f) priority=%d solmix=%.3f",
                    geom, geom_name ? geom_name : "<unnamed>",
                    contactGeomRole(m_, geom, mj_name2id(m_, mjOBJ_GEOM, "obj_rod_01")), body,
                    static_cast<int>(m_->geom_contype[geom]),
                    static_cast<int>(m_->geom_conaffinity[geom]),
                    m_->geom_size[3 * geom], m_->geom_size[3 * geom + 1], m_->geom_size[3 * geom + 2],
                    m_->geom_friction[3 * geom], m_->geom_friction[3 * geom + 1],
                    m_->geom_friction[3 * geom + 2], static_cast<int>(m_->geom_condim[geom]),
                    m_->geom_margin[geom], m_->geom_solref[2 * geom], m_->geom_solref[2 * geom + 1],
                    m_->geom_solimp[mjNIMP * geom], m_->geom_solimp[mjNIMP * geom + 1],
                    m_->geom_solimp[mjNIMP * geom + 2], m_->geom_solimp[mjNIMP * geom + 3],
                    m_->geom_solimp[mjNIMP * geom + 4], static_cast<int>(m_->geom_priority[geom]),
                    m_->geom_solmix[geom]);
              }
            }
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

    if (contact_probe_single_pad_active_) {
      auto& filter = g_contact_probe_filter_state;
      filter.model = m_;
      filter.rod_geom = mj_name2id(m_, mjOBJ_GEOM, "obj_rod_01");
      filter.finger_mesh_geoms.clear();
      if (filter.rod_geom < 0) {
        RCLCPP_FATAL(getLogger(),
            "[contact_probe_filter] missing target geom obj_rod_01; refusing ambiguous filter setup");
        return false;
      }
      for (const auto& entry : gripper_contact_body_ids_) {
        const auto& ids = entry.second;
        for (const int finger_body : {ids.left_finger, ids.right_finger}) {
          int mesh_count = 0;
          for (int geom = 0; geom < m_->ngeom; ++geom) {
            if (m_->geom_bodyid[geom] == finger_body && m_->geom_type[geom] == mjGEOM_MESH &&
                m_->geom_group[geom] == 3) {
              filter.finger_mesh_geoms.push_back(geom);
              ++mesh_count;
            }
          }
          if (finger_body < 0 || mesh_count != 1) {
            RCLCPP_FATAL(getLogger(),
                "[contact_probe_filter] expected one collision mesh per finger body; body=%d count=%d",
                finger_body, mesh_count);
            filter.enabled.store(false, std::memory_order_relaxed);
            return false;
          }
        }
      }
      if (filter.finger_mesh_geoms.empty()) {
        RCLCPP_FATAL(getLogger(), "[contact_probe_filter] no finger collision meshes found");
        return false;
      }
      if (mjcb_contactfilter != contactProbeContactFilter) {
        filter.previous_callback = mjcb_contactfilter;
      }
      filter.filtered_pairs.store(0, std::memory_order_relaxed);
      mjcb_contactfilter = contactProbeContactFilter;
      filter.enabled.store(true, std::memory_order_relaxed);
      std::string mesh_ids;
      for (const int geom : filter.finger_mesh_geoms) {
        mesh_ids += (mesh_ids.empty() ? "" : ",") + std::to_string(geom);
      }
      RCLCPP_INFO(getLogger(),
          "[contact_probe_filter] active model=%p rod_geom=%d finger_mesh_geom_ids=[%s] "
          "rule=mesh-rod only, previous_callback=%s; pad/rod and mesh/environment remain enabled",
          static_cast<const void*>(m_), filter.rod_geom, mesh_ids.c_str(),
          filter.previous_callback ? "chained" : "none");
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
        weld_state_pub_ = weld_node_->create_publisher<std_msgs::msg::Bool>(
            "/grasp_weld_state", rclcpp::QoS(10));
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
  if (contact_probe_single_pad_active_) {
    g_contact_probe_filter_state.enabled.store(true, std::memory_order_relaxed);
  }
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.hw_commands_joint_effort_.fill(0);
    arm.control_mode_ = ControlMode::None;
    
    // Initialize gripper states from current MuJoCo position
    if(arm.robot_->has_gripper_){
      const auto& gs = gripper_states_ptrs_[arm.robot_name_];
      const double q1 = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[0]];
      const double q2 = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[1]];
      // The command is the PER-FINGER displacement in metres -- the same unit
      // the position tendon actuator takes -- not the old 0-255 value.
      // Initialising with the measured position holds the gripper where it
      // already is instead of jumping. The un-sanitised q1/q2 are published as
      // the first real feedback so the action server has valid data before its
      // first control cycle.
      double init_cmd = q1;
      if(!std::isfinite(init_cmd)){ init_cmd = 0.0; }
      init_cmd = std::min(std::max(0.0, init_cmd), 0.04);
      franka_gripper::writeCommand(gs, init_cmd);
      franka_gripper::publishFeedback(gs, q1, q2);
      const franka_gripper::GripperFeedback fb = franka_gripper::readFeedback(gs);
      RCLCPP_INFO(getLogger(), "Initialized %s gripper: q1=%.4f q2=%.4f gap=%.4f cmd=%.4f m (seq %llu)",
                  arm.robot_name_.c_str(), fb.finger_q1, fb.finger_q2, fb.totalGap(),
                  fb.command_per_finger, fb.feedback_seq);
    }
  }
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMjHardwareSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  bool waiting_for_force_restore = false;
  for (const auto& arm_container_pair : arms_) {
    const auto state_it = gripper_states_ptrs_.find(arm_container_pair.first);
    if (state_it == gripper_states_ptrs_.end()) continue;
    const auto fb = franka_gripper::readFeedback(state_it->second);
    if (fb.direct_force_mode_active) {
      franka_gripper::requestForceHoldStopAtCurrent(state_it->second);
      waiting_for_force_restore = true;
    }
  }
  // The actuator model may only be restored by write(), which runs in the
  // MuJoCo physics callback. Do not report a successful deactivation while a
  // direct-force actuator could still be live. If stepping is paused, fail
  // closed; the owning simulator must resume its physics callback or destroy
  // the model before this hardware component can deactivate.
  if (waiting_for_force_restore) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
      bool all_restored = true;
      for (const auto& arm_container_pair : arms_) {
        const auto state_it = gripper_states_ptrs_.find(arm_container_pair.first);
        if (state_it != gripper_states_ptrs_.end() &&
            franka_gripper::readFeedback(state_it->second).direct_force_mode_active) {
          all_restored = false;
          break;
        }
      }
      if (all_restored) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for (const auto& arm_container_pair : arms_) {
      const auto state_it = gripper_states_ptrs_.find(arm_container_pair.first);
      if (state_it != gripper_states_ptrs_.end() &&
          franka_gripper::readFeedback(state_it->second).direct_force_mode_active) {
        RCLCPP_ERROR(getLogger(),
            "Refusing hardware deactivation: physics thread did not restore gripper actuator mode");
        return CallbackReturn::ERROR;
      }
    }
  }
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.control_mode_ = ControlMode::None;
  }
  if (contact_probe_single_pad_active_) {
    g_contact_probe_filter_state.enabled.store(false, std::memory_order_relaxed);
    RCLCPP_INFO(getLogger(), "[contact_probe_filter] filtered mesh-rod candidate pairs=%llu",
        static_cast<unsigned long long>(
            g_contact_probe_filter_state.filtered_pairs.load(std::memory_order_relaxed)));
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
      // The ONLY writer of the finger feedback, and it publishes both fingers
      // under the shared lock so a reader can never see q1 from one cycle and
      // q2 from another.
      const auto ids_it = gripper_contact_body_ids_.find(arm.robot_name_);
      const GripperContactBodyIds ids = ids_it == gripper_contact_body_ids_.end()
          ? GripperContactBodyIds{} : ids_it->second;
      bool left_contact = false;
      bool right_contact = false;
      double normals[2]{0.0, 0.0}, tangents[2]{0.0, 0.0};
      double force_xyz[3]{0.0, 0.0, 0.0};
      double force_z = 0.0;
      std::uint32_t rod_pad_contact_count = 0;
      double trace_hold_force = 0.0;
      double trace_hold_reference_length = 0.0;
      std::uint64_t trace_feedback_seq = 0;
      {
        auto& state = *gripper_states_ptrs_[arm.robot_name_];
        std::lock_guard<std::mutex> lock(state.mutex);
        trace_hold_force = state.hold_force_per_finger_n;
        trace_hold_reference_length = state.hold_reference_tendon_length;
        // read() is the single publisher and increments the sequence later in
        // this cycle, so contact and no-contact rows use the sequence that is
        // published for this same MuJoCo snapshot.
        trace_feedback_seq = state.feedback_seq + 1;
      }
      bool trace_rod_constraint = false;
      for (int e=0; e<m_->neq; ++e) {
        if (d_->eq_active[e] && (m_->eq_type[e] == mjEQ_WELD || m_->eq_type[e] == mjEQ_CONNECT) &&
            (m_->eq_obj1id[e] == ids.rod || m_->eq_obj2id[e] == ids.rod)) {
          trace_rod_constraint = true;
        }
      }
      const int trace_weld_left = weld_eq_id_ >= 0 && d_->eq_active[weld_eq_id_] ? 1 : 0;
      const int trace_weld_right = weld_right_eq_id_ >= 0 && d_->eq_active[weld_right_eq_id_] ? 1 : 0;
      const auto body_position = [this](int body, int axis) {
        return body >= 0 ? d_->xpos[3 * body + axis] : std::numeric_limits<double>::quiet_NaN();
      };
      const auto body_quaternion = [this](int body, int component) {
        return body >= 0 ? d_->xquat[4 * body + component] : std::numeric_limits<double>::quiet_NaN();
      };
      // The contact solver can report very large transient impulses while a
      // pad is entering/leaving a stiff box contact.  They are useful for
      // diagnosing the solver, but must not become a valid grasp force.  The
      // configured tendon limit is 150 N, so values above this bound cannot be
      // a credible sustained pad load for this model.
      constexpr double kMaxSustainedPadForceN = 150.0;
      if (ids.rod >= 0 && ids.left_finger >= 0 && ids.right_finger >= 0) {
        for (int i = 0; i < d_->ncon; ++i) {
          const mjContact& contact = d_->contact[i];
          const int body1 = m_->geom_bodyid[contact.geom1];
          const int body2 = m_->geom_bodyid[contact.geom2];
          if (body1 == ids.rod || body2 == ids.rod) {
            const int other = body1 == ids.rod ? body2 : body1;
            const int finger_geom = body1 == ids.rod ? contact.geom2 : contact.geom1;
            if ((other == ids.left_finger || other == ids.right_finger) &&
                contact_probe_single_pad_active_) {
              const std::size_t arm_index =
                  arm.robot_name_ == "mj_left" ? 0u : 1u;
              if (!contact_probe_first_contact_logged_[arm_index]) {
                contact_probe_first_contact_logged_[arm_index] = true;
                const char* role = contactGeomRole(m_, finger_geom, ids.rod_geom);
                if (std::string(role) == "finger_pad_box") {
                  RCLCPP_INFO(getLogger(),
                      "[contact_probe] first finger-rod contact arm=%s geom=%d role=%s; "
                      "expected single-primary-pad profile",
                      arm.robot_name_.c_str(), finger_geom, role);
                } else {
                  RCLCPP_ERROR(getLogger(),
                      "[contact_probe] INVALID contact profile arm=%s geom=%d role=%s; "
                      "expected only finger_pad_box (check that the contact filter "
                      "preserves contype/conaffinity)",
                      arm.robot_name_.c_str(), finger_geom, role);
                }
              }
            }
            // Box collision geoms are the pads; visual meshes never count.
            if ((other == ids.left_finger || other == ids.right_finger) &&
                m_->geom_type[finger_geom] == mjGEOM_BOX && m_->geom_group[finger_geom] == 3) {
              ++rod_pad_contact_count;
              const int side = other == ids.left_finger ? 0 : 1;
              // Geometric contact is separate from a credible sustained-force
              // sample. Stiff-contact impulses can exceed the force filter for
              // a step while the pad is still touching the rod; the later load
              // probe remains responsible for anti-slip/force acceptance.
              if (side == 0) left_contact = true;
              else right_contact = true;
              bool valid_force = true;
              mjtNum wrench[6]{};
              mj_contactForce(m_, d_, i, wrench);
              mjtNum trace_world_force[3]{};
              mju_mulMatTVec3(trace_world_force, contact.frame, wrench);
              const double rod_sign = body2 == ids.rod ? 1.0 : -1.0;
              for (int k = 0; k < 3; ++k) {
                valid_force = valid_force && std::isfinite(static_cast<double>(wrench[k])) &&
                    std::abs(static_cast<double>(wrench[k])) <= kMaxSustainedPadForceN;
              }
              if (valid_force) {
                // mj_contactForce's normal sign depends on geom ordering; a
                // contact magnitude is the absolute normal component.
                normals[side] += std::abs(static_cast<double>(wrench[0]));
                tangents[side] += std::hypot(static_cast<double>(wrench[1]),
                                              static_cast<double>(wrench[2]));
                mjtNum world_force[3]{};
                // MuJoCo stores contact-frame basis axes in ROWS (unlike most
                // other mjData rotation matrices). Map the contact-frame force
                // to world with the transpose; using mat*vec swaps this
                // projection and makes rod_force_xyz disagree with force_z.
                mju_mulMatTVec3(world_force, contact.frame, wrench);
                force_xyz[0] += rod_sign * world_force[0];
                force_xyz[1] += rod_sign * world_force[1];
                force_xyz[2] += rod_sign * world_force[2];
                force_z += rod_sign * world_force[2];
              }
              if (contact_probe_trace_.is_open() && trace_hold_force > 0.0 &&
                  contact_probe_trace_tick_ % 10 == 0) {
                const char* geom1 = mj_id2name(m_, mjOBJ_GEOM, contact.geom1);
                const char* geom2 = mj_id2name(m_, mjOBJ_GEOM, contact.geom2);
                const char* geom1_role = contactGeomRole(m_, contact.geom1, ids.rod_geom);
                const char* geom2_role = contactGeomRole(m_, contact.geom2, ids.rod_geom);
                const char* body1_name = mj_id2name(m_, mjOBJ_BODY, body1);
                const char* body2_name = mj_id2name(m_, mjOBJ_BODY, body2);
                const char* finger_side = side == 0 ? "left" : "right";
                const int act = arm.robot_->gripper_act_idx_;
                const int tendon = (act >= 0 && act < m_->nu) ? m_->actuator_trnid[2 * act] : -1;
                const double tendon_length = tendon >= 0 && tendon < m_->ntendon
                    ? d_->ten_length[tendon] : std::numeric_limits<double>::quiet_NaN();
                const double q1 = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[0]];
                const double q2 = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[1]];
                const double dq1 = d_->qvel[arm.robot_->gripper_joint_qvel_indices_[0]];
                const double dq2 = d_->qvel[arm.robot_->gripper_joint_qvel_indices_[1]];
                contact_probe_trace_ << std::setprecision(10) << d_->time << ',' << arm.robot_name_ << ',' << i << ','
                    << contact.geom1 << ',' << (geom1 ? geom1 : "<unnamed>") << ','
                    << geom1_role << ',' << body1 << ',' << (body1_name ? body1_name : "<unnamed>") << ','
                    << contact.geom2 << ',' << (geom2 ? geom2 : "<unnamed>") << ','
                    << geom2_role << ',' << body2 << ',' << (body2_name ? body2_name : "<unnamed>") << ',' << finger_side << ','
                    << contact.pos[0] << ',' << contact.pos[1] << ',' << contact.pos[2] << ','
                    << contact.frame[0] << ',' << contact.frame[1] << ',' << contact.frame[2] << ','
                    << contact.dim << ',' << contact.dist << ',' << contact.includemargin << ','
                    << contact.friction[0] << ',' << contact.friction[1] << ',' << contact.friction[2] << ','
                    << contact.friction[3] << ',' << contact.friction[4] << ','
                    << wrench[0] << ',' << wrench[1] << ',' << wrench[2] << ','
                    << wrench[3] << ',' << wrench[4] << ',' << wrench[5] << ','
                    << rod_sign * trace_world_force[0] << ',' << rod_sign * trace_world_force[1] << ','
                    << rod_sign * trace_world_force[2] << ',' << (valid_force ? 1 : 0) << ','
                    << q1 << ',' << q2 << ',' << dq1 << ',' << dq2 << ',' << act << ','
                    << (act >= 0 && act < m_->nu ? d_->ctrl[act] : 0.0) << ','
                    << (act >= 0 && act < m_->nu ? d_->actuator_force[act] : 0.0) << ','
                    << (act >= 0 && act < m_->nu ? m_->actuator_forcerange[2 * act + 1] : 0.0) << ','
                    << tendon_length << ','
                    << trace_hold_force << ',' << trace_hold_reference_length << ','
                    << m_->dof_damping[arm.robot_->gripper_joint_qvel_indices_[0]] << ','
                    << m_->dof_damping[arm.robot_->gripper_joint_qvel_indices_[1]] << ','
                    << body_position(ids.hand_link8, 0) << ',' << body_position(ids.hand_link8, 1) << ','
                    << body_position(ids.hand_link8, 2) << ',' << body_position(ids.rod, 0) << ','
                    << body_position(ids.rod, 1) << ',' << body_position(ids.rod, 2) << ','
                    << body_quaternion(ids.rod, 0) << ',' << body_quaternion(ids.rod, 1) << ','
                    << body_quaternion(ids.rod, 2) << ',' << body_quaternion(ids.rod, 3) << ','
                    << trace_feedback_seq << ',' << trace_weld_left << ','
                    << trace_weld_right << ',' << (trace_rod_constraint ? 1 : 0) << ','
                    << (gripper_direct_force_active_[arm.robot_name_] ? "EXPERIMENTAL_force_limited_position_hold" : "position") << ','
                    << static_cast<int>(m_->actuator_gaintype[act]) << ','
                    << static_cast<int>(m_->actuator_biastype[act]) << ','
                    << m_->actuator_gainprm[mjNGAIN * act] << ','
                    << m_->actuator_biasprm[mjNBIAS * act] << ','
                    << m_->actuator_biasprm[mjNBIAS * act + 1] << ','
                    << m_->actuator_biasprm[mjNBIAS * act + 2] << ','
                    << contact.solref[0] << ',' << contact.solref[1] << ','
                    << contact.solimp[0] << ',' << contact.solimp[1] << ','
                    << contact.solimp[2] << ',' << contact.solimp[3] << ','
                    << contact.solimp[4] << '\n';
              }
              // Invalid solver impulses do not count toward the force
              // measurements above, but geometric contact remains observable.
            }
          }
        }
      }
      // One locked physics snapshot, including force/constraint measurements.
      {
        auto& state = *gripper_states_ptrs_[arm.robot_name_];
        std::lock_guard<std::mutex> lock(state.mutex);
        state.finger_q1 = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[0]];
        state.finger_q2 = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[1]];
        state.left_finger_rod_contact = left_contact;
        state.right_finger_rod_contact = right_contact;
        state.normal_left_n = normals[0]; state.normal_right_n = normals[1];
        state.tangent_left_n = tangents[0]; state.tangent_right_n = tangents[1];
        state.rod_force_x_n = force_xyz[0];
        state.rod_force_y_n = force_xyz[1];
        state.rod_force_z_n = force_z;
        state.rod_pad_contact_count = rod_pad_contact_count;
        state.simulation_time = d_->time;
        state.weld_left_active = weld_eq_id_ >= 0 && d_->eq_active[weld_eq_id_];
        state.weld_right_active = weld_right_eq_id_ >= 0 && d_->eq_active[weld_right_eq_id_];
        state.rod_constraint_active = trace_rod_constraint;
        ++state.feedback_seq;
        trace_hold_force = state.hold_force_per_finger_n;
        trace_hold_reference_length = state.hold_reference_tendon_length;
        trace_feedback_seq = state.feedback_seq;
      }
      if (contact_probe_trace_.is_open() && trace_hold_force > 0.0 &&
          contact_probe_trace_tick_ % 10 == 0 && rod_pad_contact_count == 0) {
        const int act = arm.robot_->gripper_act_idx_;
        const int tendon = (act >= 0 && act < m_->nu) ? m_->actuator_trnid[2 * act] : -1;
        contact_probe_trace_ << std::setprecision(10) << d_->time << ',' << arm.robot_name_
            << ",-1,-1,<none>,<none>,-1,<none>,-1,<none>,<none>,-1,<none>,none";
        for (int empty_column = 0; empty_column < 25; ++empty_column) contact_probe_trace_ << ',';
        contact_probe_trace_ << d_->qpos[arm.robot_->gripper_joint_qpos_indices_[0]] << ','
            << d_->qpos[arm.robot_->gripper_joint_qpos_indices_[1]] << ','
            << d_->qvel[arm.robot_->gripper_joint_qvel_indices_[0]] << ','
            << d_->qvel[arm.robot_->gripper_joint_qvel_indices_[1]] << ',' << act << ','
            << (act >= 0 && act < m_->nu ? d_->ctrl[act] : 0.0) << ','
            << (act >= 0 && act < m_->nu ? d_->actuator_force[act] : 0.0) << ','
            << (act >= 0 && act < m_->nu ? m_->actuator_forcerange[2 * act + 1] : 0.0) << ','
            << (tendon >= 0 && tendon < m_->ntendon ? d_->ten_length[tendon] : 0.0) << ','
            << trace_hold_force << ',' << trace_hold_reference_length << ','
            << m_->dof_damping[arm.robot_->gripper_joint_qvel_indices_[0]] << ','
            << m_->dof_damping[arm.robot_->gripper_joint_qvel_indices_[1]] << ','
            << body_position(ids.hand_link8, 0) << ',' << body_position(ids.hand_link8, 1) << ','
            << body_position(ids.hand_link8, 2) << ',' << body_position(ids.rod, 0) << ','
            << body_position(ids.rod, 1) << ',' << body_position(ids.rod, 2) << ','
            << body_quaternion(ids.rod, 0) << ',' << body_quaternion(ids.rod, 1) << ','
            << body_quaternion(ids.rod, 2) << ',' << body_quaternion(ids.rod, 3) << ','
            << trace_feedback_seq << ',' << trace_weld_left << ','
            << trace_weld_right << ',' << (trace_rod_constraint ? 1 : 0) << ','
            << (gripper_direct_force_active_[arm.robot_name_] ? "EXPERIMENTAL_force_limited_position_hold" : "position") << ','
            << static_cast<int>(m_->actuator_gaintype[act]) << ','
            << static_cast<int>(m_->actuator_biastype[act]) << ','
            << m_->actuator_gainprm[mjNGAIN * act] << ','
            << m_->actuator_biasprm[mjNBIAS * act] << ','
            << m_->actuator_biasprm[mjNBIAS * act + 1] << ','
            << m_->actuator_biasprm[mjNBIAS * act + 2] << ",,,,,,,\n";
      }
    }
  }
  if (contact_probe_trace_.is_open()) {
    ++contact_probe_trace_tick_;
    if (contact_probe_trace_tick_ % 100 == 0) contact_probe_trace_.flush();
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
                                                               const rclcpp::Duration& period) {
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

      // P5/S3 observation: prove whether the internal compliance is actually
      // triggered and how far it moves the gains. Rate-limited only; no control
      // value or threshold is changed here.
      if (loop_cnt % 1000 == 0) {
          RCLCPP_INFO(getLogger(),
              "[internal_compliance] internal_stress=%.4f K_scale=%.4f D_scale=%.4f",
              internal_stress, current_K_scale, current_D_scale);
      }

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
      // The physics thread is the only writer of MuJoCo ctrl. The target comes
      // from the action server through the shared handoff; a non-finite target
      // is never written into the simulation.
      const franka_gripper::GripperFeedback fb =
          franka_gripper::readFeedback(gripper_states_ptrs_[arm.robot_name_]);
      const int act = arm.robot_->gripper_act_idx_;
      auto* mutable_model = const_cast<mjModel*>(m_);
      const auto actuator_base_it = gripper_actuator_config_base_.find(arm.robot_name_);
      const bool direct_force_requested = fb.hold_force_per_finger_n > 0.0 && fb.force_hold_supported;
      const bool direct_force_active = gripper_direct_force_active_[arm.robot_name_];
      static const bool adaptive_probe_hold = []() {
        const char* raw = std::getenv("CONTACT_PROBE_ADAPTIVE_FORCE_HOLD");
        if (raw == nullptr) return false;
        std::string value(raw);
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value == "1" || value == "true" || value == "on" || value == "yes";
      }();
      bool suppress_position_command = direct_force_requested || fb.stop_at_current_position;
      bool actuator_force_fault = false;
      if (actuator_base_it != gripper_actuator_config_base_.end()) {
        const auto& base = actuator_base_it->second;
        const auto restore_actuator = [&]() {
          mutable_model->actuator_dyntype[act] = base.dyntype;
          mutable_model->actuator_gaintype[act] = base.gaintype;
          mutable_model->actuator_biastype[act] = base.biastype;
          mutable_model->actuator_ctrllimited[act] = base.ctrllimited;
          mutable_model->actuator_forcelimited[act] = base.forcelimited;
          std::copy(base.dynprm.begin(), base.dynprm.end(), mutable_model->actuator_dynprm + mjNDYN * act);
          std::copy(base.gainprm.begin(), base.gainprm.end(), mutable_model->actuator_gainprm + mjNGAIN * act);
          std::copy(base.biasprm.begin(), base.biasprm.end(), mutable_model->actuator_biasprm + mjNBIAS * act);
          std::copy(base.gear.begin(), base.gear.end(), mutable_model->actuator_gear + 6 * act);
          std::copy(base.ctrlrange.begin(), base.ctrlrange.end(), mutable_model->actuator_ctrlrange + 2 * act);
          std::copy(base.forcerange.begin(), base.forcerange.end(), mutable_model->actuator_forcerange + 2 * act);
          gripper_direct_force_active_[arm.robot_name_] = false;
          gripper_force_hold_bias_m_[arm.robot_name_] = 0.0;
          franka_gripper::publishDirectForceModeActive(
              gripper_states_ptrs_[arm.robot_name_], false);
        };
        bool direct_force_range_valid = true;
        if (direct_force_requested) {
          double lower = 0.0;
          double upper = 0.0;
          direct_force_range_valid = franka_gripper::forceHoldActuatorForceRange(
              fb.closing_force_limit_per_finger_n, lower, upper);
          if (direct_force_range_valid) {
            if (!direct_force_active) {
              // Keep the compiled position servo and only apply the existing
              // explicit-contact force cap. A fixed target around the latched
              // contact length provides restoring stiffness; the former
              // constant-force mode had no position reference in common mode.
              mutable_model->actuator_forcelimited[act] = 1;
              mutable_model->actuator_forcerange[2 * act] = lower;
              mutable_model->actuator_forcerange[2 * act + 1] = upper;
              gripper_direct_force_active_[arm.robot_name_] = true;
              gripper_force_hold_bias_m_[arm.robot_name_] = 0.0;
              franka_gripper::publishDirectForceModeActive(
                  gripper_states_ptrs_[arm.robot_name_], true);
              const int tendon = m_->actuator_trnid[2 * act];
              const int tendon_first_wrap = (tendon >= 0 && tendon < m_->ntendon)
                  ? m_->tendon_adr[tendon] : -1;
              const double coefficient_left = tendon_first_wrap >= 0
                  ? m_->wrap_prm[tendon_first_wrap] : std::numeric_limits<double>::quiet_NaN();
              const double coefficient_right = tendon_first_wrap >= 0
                  ? m_->wrap_prm[tendon_first_wrap + 1] : std::numeric_limits<double>::quiet_NaN();
              RCLCPP_INFO(getLogger(),
                  "[contact_probe_drive] arm=%s mode=%s ctrl_units=m "
                  "request_per_finger=%.3f limit=(%.3f,%.3f) kp=%.3f gear0=%.3f "
                  "tendon_coefficients=(%.3f,%.3f); reference_length=%.6f",
                  arm.robot_name_.c_str(), adaptive_probe_hold ?
                    "EXPERIMENTAL_adaptive_normal_force_hold" : "EXPERIMENTAL_fixed_position_hold",
                  fb.closing_force_limit_per_finger_n, lower, upper,
                  base.gainprm[0], base.gear[0], coefficient_left, coefficient_right,
                  fb.hold_reference_tendon_length);
            } else {
              mutable_model->actuator_forcelimited[act] = 1;
              mutable_model->actuator_forcerange[2 * act] = lower;
              mutable_model->actuator_forcerange[2 * act + 1] = upper;
            }
          } else {
            RCLCPP_ERROR(getLogger(),
                "[contact_probe_drive] invalid direct-force request arm=%s",
                arm.robot_name_.c_str());
            actuator_force_fault = true;
          }
        }
        const bool hold_is_active = gripper_direct_force_active_[arm.robot_name_];
        if (hold_is_active && direct_force_requested && direct_force_range_valid) {
          double position_target = franka_gripper::forceHoldPositionTarget(
              fb.hold_reference_tendon_length, fb.hold_force_per_finger_n,
              base.gainprm[0], franka_gripper::kContactHoldForceCommandFraction,
              base.ctrlrange[0], base.ctrlrange[1]);
          if (std::isfinite(position_target)) {
            if (adaptive_probe_hold) {
              const double measured_normal =
                  0.5 * (std::max(0.0, fb.normal_left_n) +
                         std::max(0.0, fb.normal_right_n));
              const double force_error = fb.hold_force_per_finger_n - measured_normal;
              const double dt = period.seconds() > 0.0 && period.seconds() < 0.1
                  ? period.seconds() : 0.001;
              // Position actuator stiffness is kp in tendon metres. A signed
              // correction proportional to the measured force deficit closes
              // the fingers; the small bound prevents this diagnostic mode
              // from becoming an unbounded force/position controller.
              constexpr double kAdaptiveGain = 0.90;
              constexpr double kAdaptiveBiasLimitM = 0.004;
              double& bias = gripper_force_hold_bias_m_[arm.robot_name_];
              const double kp = std::max(1.0, base.gainprm[0]);
              const double correction = -kAdaptiveGain * force_error / kp;
              // First-order smoothing avoids reacting to a single contact
              // impulse; the correction remains bounded and deterministic.
              const double alpha = std::clamp(8.0 * dt, 0.0, 1.0);
              bias = std::clamp(bias + alpha * (correction - bias),
                                -kAdaptiveBiasLimitM, kAdaptiveBiasLimitM);
              position_target = std::clamp(position_target + bias,
                                            base.ctrlrange[0], base.ctrlrange[1]);
            }
            d_->ctrl[act] = position_target;
          } else {
            const double nominal_actuator_command_n =
                -2.0 * fb.hold_force_per_finger_n *
                franka_gripper::kContactHoldForceCommandFraction;
            const double requested_ctrl_m = fb.hold_reference_tendon_length +
                nominal_actuator_command_n / base.gainprm[0];
            RCLCPP_ERROR(getLogger(),
                "[contact_probe_drive] rejected unreachable fixed position target arm=%s "
                "request_per_finger_n=%.3f nominal_actuator_command_n=%.3f "
                "requested_ctrl_m=%.6f ctrlrange_m=(%.6f,%.6f) reference_length_m=%.6f "
                "measured_pad_normal_n=(%.3f,%.3f)",
                arm.robot_name_.c_str(), fb.hold_force_per_finger_n,
                nominal_actuator_command_n, requested_ctrl_m,
                base.ctrlrange[0], base.ctrlrange[1], fb.hold_reference_tendon_length,
                fb.normal_left_n, fb.normal_right_n);
            restore_actuator();
            actuator_force_fault = true;
          }
        } else if (direct_force_active) {
          // Restore every actuator field touched by the temporary mode before
          // interpreting the next shared command as a position in metres.
          restore_actuator();
          gripper_force_hold_bias_m_[arm.robot_name_] = 0.0;
          RCLCPP_INFO(getLogger(),
              "[contact_probe_drive] arm=%s mode=position restored gain=%g bias1=%g "
              "ctrlrange=(%g,%g) ctrllimited=%d forcerange=(%g,%g) forcelimited=%d",
              arm.robot_name_.c_str(), base.gainprm[0], base.biasprm[1],
              base.ctrlrange[0], base.ctrlrange[1], static_cast<int>(base.ctrllimited),
              base.forcerange[0], base.forcerange[1], static_cast<int>(base.forcelimited));
        }
        if (actuator_force_fault) {
          const double current_tendon = d_->actuator_length[act];
          if (std::isfinite(current_tendon)) {
            failForceHoldAtCurrent(gripper_states_ptrs_[arm.robot_name_], current_tendon);
            d_->ctrl[act] = std::clamp(current_tendon, base.ctrlrange[0], base.ctrlrange[1]);
          } else {
            requestForceHoldStopAtCurrent(gripper_states_ptrs_[arm.robot_name_], true);
          }
          suppress_position_command = true;
        }
        if (fb.stop_at_current_position && !actuator_force_fault) {
          const double current_tendon = d_->actuator_length[act];
          if (std::isfinite(current_tendon)) {
            d_->ctrl[act] = std::clamp(current_tendon, base.ctrlrange[0], base.ctrlrange[1]);
            publishStoppedTendonPosition(gripper_states_ptrs_[arm.robot_name_], current_tendon);
          }
          suppress_position_command = true;
        }
      }
      // The action server requests contact hold; the MuJoCo control callback
      // owns the physical parameter update. Add velocity-opposing finger
      // damping while force hold is active; restore the exact model baseline
      // when the explicit hold clears.
      const auto damping_it = gripper_finger_damping_base_.find(arm.robot_name_);
      if (damping_it != gripper_finger_damping_base_.end()) {
        for (std::size_t finger = 0; finger < 2; ++finger) {
          const int dof = arm.robot_->gripper_joint_qvel_indices_[finger];
          mutable_model->dof_damping[dof] = franka_gripper::contactHoldFingerDamping(
              damping_it->second[finger], fb.hold_force_per_finger_n > 0.0);
        }
      }
      if (suppress_position_command) {
        // The fixed contact-hold target or stop target was written above; do
        // not run the ordinary command law in parallel. Invalid hold requests
        // cannot fall back to an ordinary position target.
      } else if(std::isfinite(fb.command_per_finger)){
        double target = fb.command_per_finger;
        if (fb.closing_force_limit_per_finger_n > 0.0 && fb.force_hold_supported) {
          const double bounded = franka_gripper::boundedForceLimitPositionTarget(d_->actuator_length[act],
              fb.hold_force_per_finger_n > 0.0 ? fb.hold_reference_tendon_length :
                                                  d_->actuator_length[act],
              m_->actuator_gainprm[mjNGAIN*act], fb.closing_force_limit_per_finger_n,
              m_->actuator_ctrlrange[2*act], m_->actuator_ctrlrange[2*act+1]);
          target = std::max(target, bounded); // force limit also during unilateral approach
        }
        if (std::isfinite(target)) d_->ctrl[act] = target;
      }
    }
  }

  // ===== Toggle grasp weld constraints (dual: left + right) =====
  if (weld_eq_id_ >= 0) {
    bool desired = weld_active_desired_.load();
    if (desired && !weld_currently_active_) {
      if (weld_right_eq_id_ < 0 || weld_eq_id_ < 0) {
        RCLCPP_ERROR(getLogger(), "Dual grasp weld activation rejected: both equality constraints are required");
        if (weld_state_pub_) {
          std_msgs::msg::Bool state;
          state.data = false;
          weld_state_pub_->publish(state);
        }
        return hardware_interface::return_type::OK;
      }
      // Activate left-hand weld
      updateWeldRelpose();
      d_->eq_active[weld_eq_id_] = 1;
      // Activate right-hand weld (if available)
      if (weld_right_eq_id_ >= 0) {
        updateWeldRelposeRight();
        d_->eq_active[weld_right_eq_id_] = 1;
      }
      weld_currently_active_ = weld_right_eq_id_ >= 0 &&
          d_->eq_active[weld_eq_id_] != 0 && d_->eq_active[weld_right_eq_id_] != 0;
      RCLCPP_INFO(getLogger(), "Grasp weld %s (left + right; physical thread)",
                  weld_currently_active_ ? "ACTIVATED" : "REJECTED: dual equality unavailable");
      if (weld_state_pub_) {
        std_msgs::msg::Bool state;
        state.data = weld_currently_active_;
        weld_state_pub_->publish(state);
      }
    } else if (!desired && weld_currently_active_) {
      d_->eq_active[weld_eq_id_] = 0;
      if (weld_right_eq_id_ >= 0) {
        d_->eq_active[weld_right_eq_id_] = 0;
      }
      weld_currently_active_ = false;
      RCLCPP_INFO(getLogger(), "Grasp weld DEACTIVATED (left%s)",
                  weld_right_eq_id_ >= 0 ? " + right)" : " only)");
      if (weld_state_pub_) {
        std_msgs::msg::Bool state;
        state.data = false;
        weld_state_pub_->publish(state);
      }
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
