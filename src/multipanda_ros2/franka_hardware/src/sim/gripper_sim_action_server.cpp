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

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <cstdlib>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <franka_hardware/sim/gripper_sim_action_server.hpp>
#include <franka_hardware/sim/gripper_joint_names.hpp>

namespace franka_gripper {
GripperSimActionServer::GripperSimActionServer(const rclcpp::NodeOptions& options, std::string robot_name)
    : Node(robot_name + "_gripper_sim_node", options), robot_name_(std::move(robot_name)) {
  // this->declare_parameter("robot_ip", std::string());
  // this->declare_parameter("default_grasp_epsilon.inner", k_default_grasp_epsilon);
  // this->declare_parameter("default_grasp_epsilon.outer", k_default_grasp_epsilon);
  // this->declare_parameter("default_speed", k_default_speed);
  // this->declare_parameter("joint_names", std::vector<std::string>());
  // this->declare_parameter("state_publish_rate", k_default_state_publish_rate);
  // this->declare_parameter("feedback_publish_rate", k_default_feedback_publish_rate);
  // std::string robot_ip;
  // if (!this->get_parameter<std::string>("robot_ip", robot_ip)) {
  //   RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_ip' not set");
  //   throw std::invalid_argument("Parameter 'robot_ip' not set");
  // }
  /*
  state_publish_rate: 50  # [Hz]
    feedback_publish_rate: 30 # [Hz]
    default_speed: 0.1  # [m/s]
    default_grasp_epsilon:
      inner: 0.005 # [m]
      outer: 0.005 # [m]
  */

  this->default_speed_ = 0.1;
  this->default_epsilon_inner_ = 0.005;
  this->default_epsilon_outer_ = 0.005;
  // cmd_rate不再使用，改用std::this_thread::sleep_for
  // this->cmd_rate_ = std::make_shared<rclcpp::Rate>(1.0/dt);
  this->joint_names_ = fingerJointNamesForRobot(robot_name_);
  contact_feedback_publisher_ = create_publisher<dual_arm_a1_msgs::msg::GripperContactFeedback>(
      "/gripper_contact_feedback", 20);
  // if (!this->get_parameter("joint_names", this->joint_names_)) {
  //   RCLCPP_WARN(this->get_logger(), "Parameter 'joint_names' not set");
  //   this->joint_names_ = {"", ""};
  // }

  // if (this->joint_names_.size() != 2) {
  //   RCLCPP_FATAL(this->get_logger(),
  //                "Parameter 'joint_names' needs exactly two arguments, got %ld instead",
  //                this->joint_names_.size());
  //   throw std::invalid_argument("Parameter 'joint_names' has wrong number of arguments");
  // }

  const double kStatePublishRate = 50.0;
  const double kFeedbackPublishRate = 30.0;
  this->future_wait_timeout_ = rclcpp::WallRate(kFeedbackPublishRate).period();

  // RCLCPP_INFO(this->get_logger(), "Trying to establish a connection with the gripper");
  // try {
  //   this->gripper_ = std::make_unique<franka::Gripper>(robot_ip);
  // } catch (const franka::Exception& exception) {
  //   RCLCPP_FATAL(this->get_logger(), exception.what());
  //   throw exception;
  // }
  // RCLCPP_INFO(this->get_logger(), "Connected to gripper");
  const auto kHomingTask = Task::kHoming;
  this->stop_service_ =  // NOLINTNEXTLINE
      create_service<Trigger>("~/stop",
                              [this](std::shared_ptr<Trigger::Request> /*request*/,  // NOLINT
                                     std::shared_ptr<Trigger::Response> response) {  // NOLINT
                                return stopServiceCallback(std::move(response));     // NOLINT
                              });

  this->homing_server_ = rclcpp_action::create_server<Homing>(
      this, "~/homing",
      [this, kHomingTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(kHomingTask); },
      [this, kHomingTask](const auto& /*goal_handle*/) { return handleCancel(kHomingTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeHoming(goal_handle); }}.detach();
      });
  const auto kMoveTask = Task::kMove;
  this->move_server_ = rclcpp_action::create_server<Move>(
      this, "~/move",
      [this, kMoveTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(kMoveTask); },
      [this, kMoveTask](const auto& /*goal_handle*/) { return handleCancel(kMoveTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeMove(goal_handle); }}.detach();
      });

  const auto kGraspTask = Task::kGrasp;
  this->grasp_server_ = rclcpp_action::create_server<Grasp>(
      this, "~/grasp",
      [this, kGraspTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(kGraspTask); },
      [this, kGraspTask](const auto& /*goal_handle*/) { return handleCancel(kGraspTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeGrasp(goal_handle); }}.detach();
      });

  const auto kGripperCommandTask = Task::kGripperCommand;
  this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
      this, "~/gripper_action",
      [this, kGripperCommandTask](auto /*uuid*/, auto /*goal*/) {
        return handleGoal(kGripperCommandTask);
      },
      [this, kGripperCommandTask](const auto& /*goal_handle*/) {
        return handleCancel(kGripperCommandTask);
      },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { onExecuteGripperCommand(goal_handle); }}
            .detach();
      });

  this->joint_states_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
  this->timer_ = this->create_wall_timer(rclcpp::WallRate(kStatePublishRate).period(),
                                         [this]() { return publishGripperState(); });
  openTraceIfRequested(robot_name_);
  RCLCPP_INFO(get_logger(), "Started gripper sim");
}

void GripperSimActionServer::openTraceIfRequested(const std::string& robot_name) {
  const char* prefix = std::getenv("GRIPPER_TRACE_CSV");
  if (prefix == nullptr || prefix[0] == '\0') {
    return;
  }
  const std::string path = std::string(prefix) + "_" + robot_name + ".csv";
  trace_.open(path, std::ios::out | std::ios::trunc);
  if (!trace_) {
    RCLCPP_WARN(get_logger(), "[trace] could not open %s; no trace written", path.c_str());
    return;
  }
  trace_begin_ = std::chrono::steady_clock::now();
  // `commanded_gap_m` is the TOTAL gap the loop asked for, not a per-finger
  // value -- driveToGap works in total gaps and only halves them when writing
  // the actuator. The column was previously labelled `command_per_finger_m`,
  // which was wrong; the label is fixed for future traces and the existing CSVs
  // are left as they are.
  trace_ << "t_sec,robot,commanded_gap_m,command_per_finger_m,closing_force_limit_per_finger_n,q1_m,q2_m,total_gap_m,feedback_seq,target_gap_m,"
            "left_finger_obj_rod_01,right_finger_obj_rod_01,rod_pad_contact_count,phase,normal_left_n,normal_right_n,tangent_left_n,tangent_right_n,force_per_finger_n,rod_force_x_n,rod_force_y_n,rod_force_z_n,sim_time,weld_left,weld_right,rod_constraint\n";
  trace_.flush();
  RCLCPP_INFO(get_logger(), "[trace] writing raw finger feedback to %s", path.c_str());
}

void GripperSimActionServer::traceRow(double commanded_m, double target_m,
                                      const GripperFeedback& fb, const char* phase) {
  if (!trace_.is_open()) {
    return;
  }
  const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                 trace_begin_).count();
  std::lock_guard<std::mutex> lock(trace_mutex_);
  trace_ << t << ',' << this->get_name() << ',' << commanded_m << ','
         << fb.command_per_finger << ',' << fb.closing_force_limit_per_finger_n << ','
         << fb.finger_q1 << ',' << fb.finger_q2 << ',' << fb.totalGap() << ',' << fb.feedback_seq << ',' << target_m
         << ',' << (fb.left_finger_rod_contact ? 1 : 0) << ','
         << (fb.right_finger_rod_contact ? 1 : 0) << ',' << fb.rod_pad_contact_count << ',' << phase << ','
         << fb.normal_left_n << ',' << fb.normal_right_n << ',' << fb.tangent_left_n << ',' << fb.tangent_right_n << ','
         << fb.hold_force_per_finger_n << ',' << fb.rod_force_x_n << ',' << fb.rod_force_y_n << ','
         << fb.rod_force_z_n << ',' << fb.simulation_time << ',' << fb.weld_left_active << ','
         << fb.weld_right_active << ',' << fb.rod_constraint_active << '\n';
  // Flushed every row: a crash must not lose the run that explains it.
  trace_.flush();
}

void GripperSimActionServer::initGripperPtrs(std::shared_ptr<GripperSharedState> states_ptr){
  gripper_states_ptr_ = states_ptr;
}

rclcpp_action::CancelResponse GripperSimActionServer::handleCancel(Task task) {
  RCLCPP_INFO(this->get_logger(), "Received request to handleCancel %s", getTaskName(task).c_str());
  is_canceled_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GripperSimActionServer::handleGoal(Task task) {
  RCLCPP_INFO(this->get_logger(), "Received %s request", getTaskName(task).c_str());
  is_canceled_ = false;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperSimActionServer::executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle) {
  const auto kCommand = [this]() { return simGripperHoming(); };
  executeCommand(goal_handle, Task::kHoming, kCommand);
}
bool GripperSimActionServer::simGripperHoming(){
  // Homing: close, then open to the maximum width. Both stages are now
  // feedback-driven and share the same unit contract as every other command.
  constexpr double kMaxWidth = 0.08;
  if(!driveToGap(0.0, default_speed_, 0.002,
                 GripperCompletionPolicy::kPositionTarget).reached && !is_canceled_){
    return false;
  }
  if(is_canceled_){
    return false;
  }
  return driveToGap(kMaxWidth, default_speed_, 0.002,
                    GripperCompletionPolicy::kPositionTarget).reached;
}

void GripperSimActionServer::executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle) {
  auto command = [goal_handle, this]() {
    const auto kGoal = goal_handle->get_goal();
    return simGripperMove(kGoal->width, kGoal->speed);
  };
  executeCommand(goal_handle, Task::kMove, command);
}
bool GripperSimActionServer::simGripperMove(double width, double speed){
  return driveToGap(width, speed, 0.002,
                    GripperCompletionPolicy::kPositionTarget).reached;
}

void GripperSimActionServer::executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle) {
  auto command = [goal_handle, this]() {
    const auto kGoal = goal_handle->get_goal();
    return simGripperGrasp(kGoal->width, kGoal->speed, kGoal->force, kGoal->epsilon.inner,
                           kGoal->epsilon.outer);
  };
  executeCommand(goal_handle, Task::kGrasp, command);
}
bool GripperSimActionServer::simGripperGrasp(double width, double speed, double force,
                                              double /*epsilon_inner*/, double /*epsilon_outer*/){
  if (!std::isfinite(force) || force < 0.0 ||
      force > kMaxContactForcePerFingerN) return false;
  if (force > 0.0) return forceContactGrasp(width, speed, force);
  // Zero-force legacy Grasp keeps its existing position-preload semantics.
  // force / epsilon remain unimplemented (the real gripper does not expose
  // them), so no effort estimate is claimed here. This explicit Grasp action
  // is the object-contact mode; plain Move/GripperCommand retain empty-target
  // arrival semantics. The extra "+50" open-loop steps
  // that could overshoot the target are gone: the loop is bounded by the
  // distance and stops on real finger feedback.
  const GripperArrival arrival = driveToGap(width, speed, 0.002,
      explicitGraspCompletionPolicy(), true);
  RCLCPP_INFO(get_logger(),
      "[contact_action] protocol=%s outcome=%s reached_empty_target=0 gap=%.6f m seq=%llu",
      arrival.contact_confirmed ? "SUCCEEDED" : "FAILED",
      arrival.contact_confirmed ? "contact_blocked_stable" : "contact_unproven",
      arrival.has_snapshot ? arrival.snapshot.totalGap() : measuredGap(),
      arrival.has_snapshot ? arrival.snapshot.feedback_seq : 0ULL);
  return arrival.contact_confirmed;
}

// Positive-force Grasp: rate-limited approach, then persistent bounded tendon
// force. A missing contact frame does not remove the holding drive. This is
// contact establishment only; the task separately proves unsupported loading.
bool GripperSimActionServer::forceContactGrasp(double width, double speed, double force) {
  if (!std::isfinite(width) || width != 0.0 || !std::isfinite(speed) || speed <= 0.0 ||
      speed > 0.02 || !readFeedback(gripper_states_ptr_).force_hold_supported) return false;
  double commanded = measuredGap();
  if (!std::isfinite(commanded)) return false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);
  bool holding = false;
  double ramp_force = 0.0;
  double hold_reference_tendon_length = 0.0;
  unsigned long long last_seq = 0;
  double last_sim_time = -1.0;
  struct Sample { double gap; bool left; bool right; double time; };
  std::vector<Sample> samples;
  std::string exit_reason = "deadline";
  while (rclcpp::ok() && !is_canceled_ && std::chrono::steady_clock::now() < deadline) {
    const auto fb = readFeedback(gripper_states_ptr_);
    if (!fb.valid) {
      exit_reason = "feedback_invalid";
      RCLCPP_ERROR(get_logger(),
          "[CONTACT_FORCE] abort feedback_invalid valid=%d seq=%llu sim_time=%.6f q=(%.6f,%.6f)",
          fb.valid ? 1 : 0, fb.feedback_seq, fb.simulation_time, fb.finger_q1, fb.finger_q2);
      break;
    }
    if (fb.force_hold_fault) {
      exit_reason = "direct_force_mode_fault";
      RCLCPP_ERROR(get_logger(),
          "[CONTACT_FORCE] abort direct_force_mode_fault seq=%llu sim_time=%.6f",
          fb.feedback_seq, fb.simulation_time);
      break;
    }
    if (fb.weld_left_active || fb.weld_right_active || fb.rod_constraint_active) {
      exit_reason = "constraint_active";
      RCLCPP_ERROR(get_logger(),
          "[CONTACT_FORCE] abort constraint_active weld=(%d,%d) rod_constraint=%d",
          fb.weld_left_active ? 1 : 0, fb.weld_right_active ? 1 : 0,
          fb.rod_constraint_active ? 1 : 0);
      break;
    }
    if (fb.feedback_seq > last_seq && fb.simulation_time > last_sim_time) {
      last_seq = fb.feedback_seq;
      last_sim_time = fb.simulation_time;
      if (!holding && fb.left_finger_rod_contact && fb.right_finger_rod_contact) {
        holding = true;
        hold_reference_tendon_length = 0.5 * (fb.finger_q1 + fb.finger_q2);
        RCLCPP_INFO(get_logger(),
            "[CONTACT_FORCE] approach->direct_tendon_force_hold geometric_bilateral_contact "
            "gap=%.6f tendon_reference=%.6f force_request_per_finger=%.2f N; load_unverified",
            fb.totalGap(), hold_reference_tendon_length, force);
      }
      if (holding) {
        ramp_force = std::min(force, ramp_force + 0.05); // <=5 N/s at the 100 Hz command loop
        const bool bilateral_force = fb.left_finger_rod_contact && fb.right_finger_rod_contact &&
            fb.normal_left_n >= 0.25 && fb.normal_right_n >= 0.25;
        if (!writeForceHold(gripper_states_ptr_, ramp_force, hold_reference_tendon_length)) {
          exit_reason = "write_force_hold_failed";
          RCLCPP_ERROR(get_logger(),
              "[CONTACT_FORCE] abort write_force_hold_failed ramp=%.3f supported=%d "
              "q=(%.6f,%.6f) contacts=(%d,%d)",
              ramp_force, fb.force_hold_supported ? 1 : 0, fb.finger_q1, fb.finger_q2,
              fb.left_finger_rod_contact ? 1 : 0, fb.right_finger_rod_contact ? 1 : 0);
          break;
        }
        if (ramp_force >= force) {
          samples.push_back({fb.totalGap(), bilateral_force, bilateral_force,
                             fb.simulation_time});
          if (samples.size() > 20) samples.erase(samples.begin());
          double lo=fb.totalGap(), hi=lo;
          int left=0, right=0;
          for (const auto& sample : samples) {
            lo=std::min(lo,sample.gap); hi=std::max(hi,sample.gap);
            left+=sample.left; right+=sample.right;
          }
          const double window_duration = samples.empty() ? 0.0 :
              samples.back().time - samples.front().time;
          if (contactForceWindowAccepted(samples.size(), window_duration, left, right, hi-lo)) {
            RCLCPP_INFO(get_logger(),
                "[CONTACT_FORCE] bilateral_force_established request_per_finger_n=%.3f "
                "measured_pad_normal_per_finger_n=(%.3f,%.3f) samples=20 "
                "pad_contact_occupancy=(%d,%d) gap_spread=%.6f gap_spread_limit=%.6f "
                "persistent_hold=true; load_unverified",
                force,fb.normal_left_n,fb.normal_right_n,
                left,right,hi-lo,kContactForceStableGapSpreadM);
            return true;
          }
        }
      } else {
        commanded=std::max(width,commanded-speed*dt);
        if (!writeCommand(gripper_states_ptr_, commanded/2.0, force)) {
          exit_reason = "write_approach_command_failed";
          RCLCPP_ERROR(get_logger(),
              "[CONTACT_FORCE] abort write_approach_command_failed command=%.6f force=%.3f supported=%d",
              commanded, force, fb.force_hold_supported ? 1 : 0);
          break;
        }
        if (fb.totalGap() < 0.005) {
          exit_reason = "empty_closure";
          RCLCPP_ERROR(get_logger(), "[CONTACT_FORCE] abort empty_closure gap=%.6f", fb.totalGap());
          break;
        } // empty closure is never success
      }
      traceRow(commanded, width, fb, holding ? "force_hold" : "force_approach");
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }
  // No load has been commanded here. Stop the approach; explicit Stop/Move
  // also disables force mode. The caller will never lift on this failure.
  const double gap=measuredGap();
  const auto final_fb = readFeedback(gripper_states_ptr_);
  double sample_gap_min = std::numeric_limits<double>::infinity();
  double sample_gap_max = -std::numeric_limits<double>::infinity();
  int sample_left_contact = 0, sample_right_contact = 0;
  for (const auto& sample : samples) {
    sample_gap_min = std::min(sample_gap_min, sample.gap);
    sample_gap_max = std::max(sample_gap_max, sample.gap);
    sample_left_contact += sample.left ? 1 : 0;
    sample_right_contact += sample.right ? 1 : 0;
  }
  const double sample_gap_spread = samples.empty() ? std::numeric_limits<double>::quiet_NaN() :
      sample_gap_max - sample_gap_min;
  if (is_canceled_) exit_reason = "canceled";
  else if (!rclcpp::ok()) exit_reason = "rclcpp_shutdown";
  else if (std::chrono::steady_clock::now() >= deadline && exit_reason == "deadline") exit_reason = "deadline";
  // Every failed/canceled grasp leaves direct-force mode atomically with the
  // position handoff. Invalid feedback asks the physics thread to hold its
  // current tendon position rather than replaying a stale approach target.
  if (std::isfinite(gap)) writeGapCommand(gap);
  else requestForceHoldStopAtCurrent(gripper_states_ptr_);
  RCLCPP_ERROR(get_logger(),
      "[CONTACT_FORCE] failed reason=%s holding=%d ramp_force=%.3f samples=%zu "
      "final_gap=%.6f final_seq=%llu final_sim_time=%.6f final_hold_force=%.3f "
      "measured_pad_normal_per_finger_n=(%.3f,%.3f) final_contacts=(%d,%d) "
      "last_window_gap_spread=%.6f last_window_occupancy=(%d,%d) "
      "last_window_time=%.6f..%.6f no_load_proven=true",
      exit_reason.c_str(), holding ? 1 : 0, ramp_force, samples.size(), gap,
      final_fb.feedback_seq, final_fb.simulation_time, final_fb.hold_force_per_finger_n,
      final_fb.normal_left_n, final_fb.normal_right_n,
      final_fb.left_finger_rod_contact ? 1 : 0, final_fb.right_finger_rod_contact ? 1 : 0,
      sample_gap_spread, sample_left_contact, sample_right_contact,
      samples.empty() ? -1.0 : samples.front().time,
      samples.empty() ? -1.0 : samples.back().time);
  return false;
}

void GripperSimActionServer::onExecuteGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle) {
  const auto kGoal = goal_handle->get_goal();
  const double kTargetWidth = 2 * kGoal->command.position;
  constexpr double kMaxWidth = 0.08;

  const double kCurrentWidth = measuredGap();

  RCLCPP_INFO(this->get_logger(), "[onExecuteGripperCommand] goal_position=%.4f, kTargetWidth=%.4f, kCurrentWidth=%.4f",
              kGoal->command.position, kTargetWidth, kCurrentWidth);

  // A non-finite command is rejected outright. The old code fed `2 * NaN`
  // through the width check and then into a helper that mapped non-finite input
  // to 0.0 -- i.e. a NaN silently became a full close.
  if (!std::isfinite(kGoal->command.position) || !std::isfinite(kTargetWidth)) {
    RCLCPP_ERROR(this->get_logger(),
                 "GripperServer: non-finite command position %f; rejected, NOT converted to 0",
                 kGoal->command.position);
    auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
    result->reached_goal = false;
    result->stalled = false;
    result->effort = 0.0;
    result->position = std::numeric_limits<double>::quiet_NaN();
    goal_handle->abort(result);
    return;
  }
  if (kTargetWidth > kMaxWidth || kTargetWidth < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "GripperServer: Commanding out of range width! max_width = %f command = %f",
                 kMaxWidth, kTargetWidth);
    auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
    result->reached_goal = false;
    result->stalled = false;
    result->effort = 0.0;
    result->position = std::isfinite(kCurrentWidth) ? kCurrentWidth
                                                    : std::numeric_limits<double>::quiet_NaN();
    goal_handle->abort(result);
    return;
  }

  // There is deliberately no "already at target" shortcut any more. It returned
  // success from a cached width without touching fresh feedback, so an
  // already-satisfied command and a real move did not go through the same
  // check. Every command now runs the same stability window inside driveToGap;
  // a gripper that is already there simply satisfies it.
  const auto completion_policy = gripperCommandCompletionPolicy(kTargetWidth, kCurrentWidth);
  auto command = [kTargetWidth, completion_policy, this]() -> GripperArrival {
    RCLCPP_INFO(this->get_logger(), "[onExecuteGripperCommand] Calling position move");
    return driveToGap(kTargetWidth, default_speed_, 0.002, completion_policy);
  };

  executeGripperCommand(goal_handle, command);
}

void GripperSimActionServer::executeGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
    const std::function<GripperArrival()>& command_handler) {
  const auto kTaskName = getTaskName(Task::kGripperCommand);
  RCLCPP_INFO(this->get_logger(), "Gripper %s...", kTaskName.c_str());

  auto command_execution_thread = [command_handler, this]() {
    auto arrival = std::make_shared<GripperArrival>();
    try {
      *arrival = command_handler();
    } catch (const std::exception& e) {
      arrival->reached = false;
      arrival->has_snapshot = false;
      RCLCPP_ERROR(this->get_logger(), e.what());
    }
    return arrival;
  };

  std::future<std::shared_ptr<GripperArrival>> result_future =
      std::async(std::launch::async, command_execution_thread);

  while (!resultIsReady(result_future, future_wait_timeout_) && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      simGripperStop();
      const auto kArrival = result_future.get();
      auto result = std::make_shared<GripperCommand::Result>();
      // A cancelled move is never arrival, whatever the handler produced.
      result->reached_goal = false;
      result->stalled = false;
      result->effort = 0.;
      result->position = kArrival->has_snapshot ? kArrival->snapshot.totalGap() : measuredGap();
      RCLCPP_INFO(get_logger(), "Gripper %s canceled", kTaskName.c_str());
      goal_handle->canceled(result);
      return;
    }
    publishGripperCommandFeedback(goal_handle);
  }
  if (rclcpp::ok()) {
    const auto kArrival = result_future.get();
    auto result = std::make_shared<GripperCommand::Result>();
    result->effort = 0.;
    result->stalled = false;
    result->reached_goal = kArrival->reached;
    // The SAVED arrival snapshot, not a value re-read after the handler
    // returned. A move that did not reach its window has no snapshot, and then
    // the current measurement is reported for diagnosis only -- with
    // reached_goal already false.
    result->position = kArrival->has_snapshot ? kArrival->snapshot.totalGap() : measuredGap();
    if (kArrival->reached) {
      RCLCPP_INFO(get_logger(), "Gripper %s succeeded", kTaskName.c_str());
      goal_handle->succeed(result);
    } else {
      RCLCPP_INFO(get_logger(), "Gripper %s failed", kTaskName.c_str());
      goal_handle->abort(result);
    }
  }
}

void GripperSimActionServer::stopServiceCallback(const std::shared_ptr<Trigger::Response>& response) {
  RCLCPP_INFO(this->get_logger(), "Stopping gripper_...");
  auto action_result = withResultGenerator<Homing>([this]() { return simGripperStop(); })();
  response->success = action_result->success;
  response->message = action_result->error;
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Gripper stopped");
  } else {
    RCLCPP_INFO(this->get_logger(), "Gripper could not be stopped");
  }
  if (!response->message.empty()) {
    RCLCPP_ERROR(this->get_logger(), response->message.c_str());
  }
}

bool GripperSimActionServer::simGripperStop(){
  is_canceled_ = true;
  // Hold the last measured opening, in the same per-finger metre units as every
  // other command (no 0-255 conversion). If feedback is unavailable, ask the
  // physics thread to capture its live tendon length; do not replay the old
  // position target after leaving direct-force mode.
  const double gap = measuredGap();
  if(!std::isfinite(gap)){
    RCLCPP_WARN(this->get_logger(),
                "[simGripperStop] no valid finger feedback; stopping at live tendon position");
    requestForceHoldStopAtCurrent(gripper_states_ptr_);
    return true;
  }
  writeGapCommand(gap);
  std::cout << "Should stop " << std::endl;
  return true;
}

bool GripperSimActionServer::writeGapCommand(double gap_m){
  // The actuator is a position actuator driving the fixed tendon
  //   0.5*q1 + 0.5*q2
  // so its control input is in metres of PER-FINGER displacement (equal to the
  // tendon length for symmetric fingers), NOT a 0-255 value. ctrlrange is
  // [0, 0.08] but the joints only reach 0.04, so the command is clamped there.
  constexpr double kMaxPerFinger = 0.04;
  if(!std::isfinite(gap_m)){
    // Never turn a NaN into 0.0: that would command a full close.
    RCLCPP_ERROR(this->get_logger(),
                 "[writeGapCommand] rejecting non-finite gap %f; command left unchanged", gap_m);
    return false;
  }
  double per_finger = gap_m / 2.0;
  per_finger = std::min(std::max(0.0, per_finger), kMaxPerFinger);
  return writeCommand(gripper_states_ptr_, per_finger);
}

double GripperSimActionServer::measuredGap(){
  // q1 and q2 are written by FrankaMjHardwareSystem::read() from the real
  // MuJoCo finger qpos, so this is measured feedback rather than a model.
  // The total is their SUM, not one finger doubled.
  const GripperFeedback fb = readFeedback(gripper_states_ptr_);
  if(!fb.valid){
    return std::numeric_limits<double>::quiet_NaN();
  }
  return fb.totalGap();
}

GripperSimActionServer::GripperArrival GripperSimActionServer::driveToGap(
    double width_m, double speed_m_s, double tol_m,
    GripperCompletionPolicy completion_policy,
    bool hold_after_bilateral_contact){
  GripperArrival arrival;
  constexpr double kMaxWidth = 0.08;
  constexpr int kStableSamples = 10;     // genuinely NEW samples inside tolerance
  constexpr double kStableDelta = 5e-4;  // contact-establishment gap spread (m)
  constexpr double kMinSpeed = 1e-4;     // m/s: below this the command step is meaningless

  if(!std::isfinite(width_m)){
    RCLCPP_ERROR(this->get_logger(),
                 "[driveToGap] rejecting non-finite target width %f (NOT treated as full close)",
                 width_m);
    return arrival;
  }
  // Explicit valid speed range. The previous version floored the step at 1e-5 m
  // so that a tiny speed still moved, which let the command change faster than
  // speed*dt. A speed below the supported range is now rejected instead.
  if(!std::isfinite(speed_m_s) || speed_m_s < kMinSpeed){
    RCLCPP_ERROR(this->get_logger(),
                 "[driveToGap] rejecting speed %g m/s: outside the supported range [%g, inf)",
                 speed_m_s, kMinSpeed);
    return arrival;
  }

  const double target = std::min(std::max(0.0, width_m), kMaxWidth);
  // The speed really bounds the command: no floor is added, so no step can
  // exceed speed*dt.
  const double max_step = speed_m_s * dt;
  // Explicit Grasp contact gets a 12 s protocol budget. Its client waits up to
  // 13.5 s for the result, leaving 1.5 s for action scheduling/serialization. The
  // empty-target Move/GripperCommand paths keep their historical 8 s budget.
  const auto deadline = std::chrono::steady_clock::now() +
      (hold_after_bilateral_contact ? std::chrono::seconds(12) : std::chrono::seconds(8));

  double commanded = measuredGap();
  if(!std::isfinite(commanded)){
    RCLCPP_ERROR(this->get_logger(),
                 "[driveToGap] no valid finger feedback at entry; not commanding blind");
    return arrival;
  }
  StabilityWindow window(kStableSamples, tol_m, kStableDelta);
  BilateralContactHold contact_hold(12, kStableDelta);
  int bilateral_contact_samples = 0;
  unsigned long long last_contact_seq = 0;
  std::vector<double> contact_gaps;

  while(std::chrono::steady_clock::now() < deadline){
    if(is_canceled_){
      if (hold_after_bilateral_contact) {
        arrival.snapshot = readFeedback(gripper_states_ptr_);
        arrival.has_snapshot = arrival.snapshot.valid;
        RCLCPP_WARN(get_logger(),
            "[contact_hold] canceled stage=%s hold_target=%.6f m window=%d spread=%.6f m",
            contact_hold.stage() == BilateralContactHold::Stage::HOLD ? "HOLD" : "APPROACH",
            contact_hold.holdTargetGap(), contact_hold.windowSamples(), contact_hold.windowSpread());
      }
      return arrival;
    }
    if (hold_after_bilateral_contact) {
      const GripperFeedback fb = readFeedback(gripper_states_ptr_);
      if(!fb.valid) return arrival;
      const auto update = contact_hold.update(fb, commanded, target, max_step);
      if (update.entered_hold) {
        RCLCPP_INFO(get_logger(),
            "[contact_hold] APPROACH->HOLD seq=%llu latched_measured_gap=%.6f m previous_command_gap=%.6f m hold_target=%.6f m total_preload=%.6f m; target latched once",
            fb.feedback_seq, contact_hold.latchedContactGap(), commanded,
            update.hold_target_gap_m, contact_hold.latchedContactGap() - update.hold_target_gap_m);
      } else if (update.contact_lost) {
        RCLCPP_WARN(get_logger(),
            "[contact_hold] bilateral contact lost seq=%llu; acceptance window reset; fixed hold_target=%.6f m retained",
            fb.feedback_seq, update.hold_target_gap_m);
      }
      if (!writeGapCommand(update.command_gap_m)) return arrival;
      commanded = update.command_gap_m;
      const double trace_target = update.stage == BilateralContactHold::Stage::HOLD
                                      ? update.hold_target_gap_m : target;
      traceRow(commanded, trace_target, fb,
               update.stage == BilateralContactHold::Stage::HOLD ? "contact_hold" : "approach");
      if (update.confirmed) {
        if(!contactSuccessAllowed(true, is_canceled_,
                                  std::chrono::steady_clock::now() >= deadline)) return arrival;
        arrival.reached = true;
        arrival.contact_confirmed = true;
        arrival.has_snapshot = true;
        arrival.snapshot = fb;
        RCLCPP_INFO(get_logger(),
            "[contact_hold] CONFIRMED seq=%llu q1=%.6f q2=%.6f measured_gap=%.6f m hold_target=%.6f m window=%d spread=%.6f m",
            fb.feedback_seq, fb.finger_q1, fb.finger_q2, fb.totalGap(),
            update.hold_target_gap_m, update.window_samples, update.window_spread_m);
        return arrival;
      }
      const bool commanded_done =
          std::abs(commanded - target) <= max_step &&
          update.stage == BilateralContactHold::Stage::APPROACH;
      if (commanded_done && window.update(fb, target)) {
        if (completion_policy == GripperCompletionPolicy::kBilateralContact) {
          arrival.has_snapshot = true;
          arrival.snapshot = window.snapshot();
          RCLCPP_WARN(get_logger(),
                      "[contact_hold] empty target reached without bilateral rod contact; contact failed");
          return arrival;
        }
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      continue;
    }

    // Progressive command: move the target toward the goal by at most one step.
    const double delta = target - commanded;
    if(std::abs(delta) > max_step){
      commanded += (delta > 0 ? max_step : -max_step);
    } else {
      commanded = target;
    }
    if(!writeGapCommand(commanded)){
      return arrival;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    const GripperFeedback fb = readFeedback(gripper_states_ptr_);
    if(!fb.valid) return arrival;
    traceRow(commanded, target, fb);
      if (completion_policy == GripperCompletionPolicy::kBilateralContact &&
          fb.feedback_seq > last_contact_seq) {
      last_contact_seq = fb.feedback_seq;
      if (fb.left_finger_rod_contact && fb.right_finger_rod_contact) {
        contact_gaps.push_back(fb.totalGap());
        if (contact_gaps.size() > 16) {
          contact_gaps.erase(contact_gaps.begin());
        }
        bilateral_contact_samples = std::min(16, bilateral_contact_samples + 1);
        if (bilateral_contact_samples == 16) {
          const auto bounds = std::minmax_element(contact_gaps.begin(), contact_gaps.end());
          if (bounds.second != contact_gaps.end() && *bounds.second - *bounds.first <= kStableDelta) {
            arrival.reached = true;
            arrival.contact_confirmed = true;
            arrival.has_snapshot = true;
            arrival.snapshot = fb;
            RCLCPP_INFO(get_logger(),
                "[driveToGap] bilateral rod contact confirmed: q1=%.5f q2=%.5f gap=%.5f seq=%llu",
                fb.finger_q1, fb.finger_q2, fb.totalGap(), fb.feedback_seq);
            return arrival;
          }
        }
      } else {
        bilateral_contact_samples = 0;
        contact_gaps.clear();
      }
    }
    const bool commanded_done = std::abs(commanded - target) <= max_step;
    // The window only starts once the whole command has been issued, and only
    // counts feedback whose sequence number has advanced.
    if(commanded_done && window.update(fb, target)){
      if(is_canceled_ || std::chrono::steady_clock::now() >= deadline){
        // Cancelled, or the window only filled at/after the deadline: a late
        // arrival is never reported as arrival.
        return arrival;
      }
      if (completion_policy == GripperCompletionPolicy::kBilateralContact) {
        // Reaching an empty-space target without both named finger contacts is
        // an aborted contact attempt, never a grasp success.
        arrival.has_snapshot = true;
        arrival.snapshot = window.snapshot();
        RCLCPP_WARN(get_logger(),
                    "[driveToGap] target reached without bilateral obj_rod_01 contact; contact failed");
        return arrival;
      }
      arrival.reached = true;
      arrival.has_snapshot = true;
      arrival.snapshot = window.snapshot();
      return arrival;
    }
  }
  if (hold_after_bilateral_contact) {
    arrival.snapshot = readFeedback(gripper_states_ptr_);
    arrival.has_snapshot = arrival.snapshot.valid;
    RCLCPP_WARN(this->get_logger(),
        "[contact_hold] deadline expired stage=%s commanded_gap=%.6f m measured_gap=%.6f m approach_gap=%.6f m hold_target=%.6f m window=%d spread=%.6f m seq=%llu",
        contact_hold.stage() == BilateralContactHold::Stage::HOLD ? "HOLD" : "APPROACH",
        commanded, arrival.has_snapshot ? arrival.snapshot.totalGap() : measuredGap(), target,
        contact_hold.holdTargetGap(), contact_hold.windowSamples(), contact_hold.windowSpread(),
        arrival.has_snapshot ? arrival.snapshot.feedback_seq : 0ULL);
  } else {
    RCLCPP_WARN(this->get_logger(),
                "[driveToGap] gave up: commanded %.4f, measured %.4f, target %.4f, window %d/%d spread %.5f",
                commanded, measuredGap(), target, window.count(), window.required(), window.spread());
  }
  return arrival;
}

franka::GripperState GripperSimActionServer::getGripperState(const GripperFeedback& fb){
  // Takes the already-read snapshot: the caller holds gripper_state_mutex_, and
  // reading the shared feedback here as well would be a second, later sample.
  franka::GripperState new_state;
  new_state.temperature = 0;
  new_state.time = franka::Duration(0); // time is honestly kinda useless?
  new_state.max_width = 0.08; // temporarily, until I figure out the sites
  new_state.width = fb.valid ? fb.totalGap() : 0.0; // q1 + q2, both fingers
  new_state.is_grasped = false; // this should be decided by reading the force at the actuator, and the current width
  return new_state;
}

void GripperSimActionServer::publishGripperState() {
  const GripperFeedback fb = readFeedback(gripper_states_ptr_);
  if (!fb.valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[publishGripperState] no valid finger feedback yet; not publishing");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    current_gripper_state_ = getGripperState(fb);
  }
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = this->now();
  joint_states.name.push_back(this->joint_names_[0]);
  joint_states.name.push_back(this->joint_names_[1]);
  // The two REAL finger positions. The previous version published width/2
  // twice, i.e. one derived number copied onto both joints, which hid any
  // finger asymmetry from every subscriber.
  joint_states.position.push_back(fb.finger_q1);
  joint_states.position.push_back(fb.finger_q2);
  joint_states.velocity.push_back(0.0);
  joint_states.velocity.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states_publisher_->publish(joint_states);

  dual_arm_a1_msgs::msg::GripperContactFeedback contact;
  contact.header.stamp = this->now();
  contact.arm = robot_name_;
  contact.finger_q1 = fb.finger_q1;
  contact.finger_q2 = fb.finger_q2;
  contact.command_per_finger_m = fb.command_per_finger;
  contact.closing_force_limit_per_finger_n = fb.closing_force_limit_per_finger_n;
  contact.left_finger_rod_contact = fb.left_finger_rod_contact;
  contact.right_finger_rod_contact = fb.right_finger_rod_contact;
  contact.update_seq = fb.feedback_seq;
  contact.normal_left_n = fb.normal_left_n;
  contact.normal_right_n = fb.normal_right_n;
  contact.tangent_left_n = fb.tangent_left_n;
  contact.tangent_right_n = fb.tangent_right_n;
  contact.rod_force_x_n = fb.rod_force_x_n;
  contact.rod_force_y_n = fb.rod_force_y_n;
  contact.rod_force_z_n = fb.rod_force_z_n;
  contact.simulation_time = fb.simulation_time;
  contact.rod_pad_contact_count = fb.rod_pad_contact_count;
  contact.hold_force_per_finger_n = fb.hold_force_per_finger_n;
  contact.force_hold_supported = fb.force_hold_supported;
  contact.weld_left_active = fb.weld_left_active;
  contact.weld_right_active = fb.weld_right_active;
  contact.rod_constraint_active = fb.rod_constraint_active;

  contact_feedback_publisher_->publish(contact);
}

void GripperSimActionServer::publishGripperCommandFeedback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle) {
  auto gripper_feedback = std::make_shared<GripperCommand::Feedback>();
  // Fresh q1 + q2 from the shared handoff, not the 50 Hz cached state.
  gripper_feedback->position = measuredGap();
  gripper_feedback->effort = 0.;
  goal_handle->publish_feedback(gripper_feedback);
}
}  // namespace franka_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(franka_gripper::GripperSimActionServer)  // NOLINT
