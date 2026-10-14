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

#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <franka/gripper_state.h>
#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>
#include <franka_hardware/sim/gripper_command_policy.hpp>
#include <franka_hardware/sim/gripper_shared_state.hpp>
#include <dual_arm_a1_msgs/msg/gripper_contact_feedback.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
namespace franka_gripper {

/// checks whether an asynchronous command has finished
/// @tparam T the expected return type of the future
/// @param t the future which should be checked
/// @param future_wait_timeout how long to wait for the result before returning false
/// @return whether the asynchronous function has already finished
template <typename T>
bool resultIsReady(std::future<T>& t, std::chrono::nanoseconds future_wait_timeout) {
  return t.wait_for(future_wait_timeout) == std::future_status::ready;
}

/// ROS node that offers multiple actions to use the gripper.
class GripperSimActionServer : public rclcpp::Node {
 public:
  using Homing = franka_msgs::action::Homing;
  using GoalHandleHoming = rclcpp_action::ServerGoalHandle<Homing>;

  using Move = franka_msgs::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  using Grasp = franka_msgs::action::Grasp;
  using GoalHandleGrasp = rclcpp_action::ServerGoalHandle<Grasp>;

  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

  using Trigger = std_srvs::srv::Trigger;

  /// creates an instance of a GripperActionServer
  /// @param options options for node initialization
  explicit GripperSimActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), std::string robot_name="panda");
  // GripperSimActionServer(const rclcpp::NodeOptions& options, double &gripper_cmd, std::array<double, 2> &gripper_states);
  void initGripperPtrs(std::shared_ptr<GripperSharedState> states_ptr);
 private:
  /// describes the different tasks. Each task corresponds to one action server
  enum class Task { kHoming, kMove, kGrasp, kGripperCommand };

  /// returns a string version of the Task enum
  static std::string getTaskName(Task task) {
    switch (task) {
      case Task::kHoming:
        return {"Homing"};
      case Task::kMove:
        return {"Moving"};
      case Task::kGrasp:
        return {"Grasping"};
      case Task::kGripperCommand:
        return {"GripperCommand"};
      default:
        throw std::invalid_argument("getTaskName is not implemented for this case");
    }
  };

  const double k_default_grasp_epsilon = 0.005;    // default inner and outer grasp epsilon in meter
  const double k_default_speed = 0.1;              // default gripper speed in m/s
  const double dt = 0.01;                          // make the gripper update only at 100hz
  const int k_default_state_publish_rate = 30;     // default gripper state publish rate
  const int k_default_feedback_publish_rate = 10;  // default action feedback publish rate

  std::shared_ptr<GripperSharedState> gripper_states_ptr_;  // guarded handoff, see the header

  rclcpp_action::Server<Homing>::SharedPtr homing_server_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_command_server_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  std::mutex gripper_state_mutex_;
  franka::GripperState current_gripper_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<dual_arm_a1_msgs::msg::GripperContactFeedback>::SharedPtr contact_feedback_publisher_;
  std::string robot_name_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Rate> cmd_rate_;
  bool is_canceled_;

  double default_speed_;          // default gripper speed parameter value in m/s
  double default_epsilon_inner_;  // default gripper inner epsilon parameter value in m
  double default_epsilon_outer_;  //  default gripper outer epsilon parameter value in m
  std::vector<std::string> joint_names_;
  std::chrono::nanoseconds future_wait_timeout_{0};

  /// What one bounded, feedback-driven move produced.
  struct GripperArrival {
    bool reached {false};
    bool contact_confirmed {false};
    /// True only when `snapshot` is the feedback sample that satisfied arrival.
    /// The action result uses this saved snapshot rather than re-reading.
    bool has_snapshot {false};
    GripperFeedback snapshot;
  };

  /// Builds the franka_msgs-style state from one feedback snapshot.
  franka::GripperState getGripperState(const GripperFeedback& fb);
  void publishGripperState();

  /// Writes the actuator command for a desired TOTAL gap.
  ///
  /// The model's gripper actuator is a position actuator over the fixed tendon
  /// `0.5*q1 + 0.5*q2`, i.e. its control input is in metres of PER-FINGER
  /// displacement (equal to the tendon length when the fingers are symmetric),
  /// with ctrlrange [0, 0.08] and a joint range of [0, 0.04]. Commanding the
  /// 0-255 value the previous code used therefore saturated the actuator for
  /// every command.
  ///
  /// Returns false, and writes nothing, for a non-finite gap: a NaN command
  /// must never be silently turned into a full close, which is where the old
  /// `isFinite ? x : 0.0` fallback ended up.
  bool writeGapCommand(double gap_m);

  /// Real total gap `q1 + q2` from the finger joint feedback, or NaN when no
  /// valid feedback is available. Not one finger doubled.
  double measuredGap();

  /// Bounded, feedback-driven move to a total gap `width_m`.
  ///
  /// Returns reached=true ONLY when the measurement arrived inside `tol` on a
  /// run of `kStableSamples` genuinely new feedback samples (strictly
  /// increasing sequence numbers) whose whole-window spread is within
  /// `kStableDelta`, with the full command already issued and no cancel or
  /// deadline expiry. Contact completion is selected only by an explicit
  /// Grasp action. On success `out.snapshot` holds that feedback sample.
  GripperArrival driveToGap(double width_m, double speed_m_s, double tol_m,
                            GripperCompletionPolicy completion_policy,
                            bool hold_after_bilateral_contact = false);

  /// Optional raw trace of the four real finger values while a move runs.
  ///
  /// Enabled by the environment variable `GRIPPER_TRACE_CSV`: when it holds a
  /// path prefix, this node appends its rows to `<prefix>_<node name>.csv`.
  /// Off by default, and deliberately a plain CSV append rather than a logging
  /// system. Only ever written from the move loop, never from a callback.
  void openTraceIfRequested(const std::string& robot_name);
  void traceRow(double commanded_m, double target_m, const GripperFeedback& fb,
                const char* phase = "move");
  std::ofstream trace_;
  std::mutex trace_mutex_;
  std::chrono::steady_clock::time_point trace_begin_;

  /// stops the gripper and writes the result into the response
  /// @param[out] response  will be updated with the success status and error message
  void stopServiceCallback(const std::shared_ptr<Trigger::Response>& response);

  /// accepts any cancel request
  rclcpp_action::CancelResponse handleCancel(Task task);

  /// accepts any goal request
  rclcpp_action::GoalResponse handleGoal(Task task);

  /// performs homing
  void executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle);
  bool simGripperHoming();

  /// performs move
  void executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle);
  bool simGripperMove(double width, double speed);

  /// performs grasp
  void executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle);
  bool simGripperGrasp(double width, double speed, double force, double epsilon_inner,
                           double epsilon_outer);

  /// Stops the sim gripper command
  bool simGripperStop();

  /// Performs the moveit grasp command
  /// @param goal_handle
  /// @param command_handler a bounded move defined by onExecuteGripperCommand
  void executeGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
                             const std::function<GripperArrival()>& command_handler);

  /// Defines a function for either grasping or moving the gripper, depending on the current gripper
  /// state and the commanded goal. Then it calls executeGripperCommand to execute that function
  void onExecuteGripperCommand(const std::shared_ptr<GoalHandleGripperCommand>& goal_handle);

  /// Executes a gripper command
  /// @tparam T A gripper action message type (Move, Grasp, Homing)
  /// @param[in] goal_handle The goal handle from the action server
  /// @param[in] task The type of the Task
  /// @param[in] command_handler a function that performs the the task. Returns true on success.
  /// This function is allowed to throw a franka::Exception
  template <typename T>
  void executeCommand(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle,
                      Task task,
                      const std::function<bool()>& command_handler) {
    const auto kTaskName = getTaskName(task);
    RCLCPP_INFO(this->get_logger(), "Gripper %s...", kTaskName.c_str());

    auto command_execution_thread = withResultGenerator<T>(command_handler);

    std::future<std::shared_ptr<typename T::Result>> result_future =
        std::async(std::launch::async, command_execution_thread);

    while (!resultIsReady(result_future, future_wait_timeout_) && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        simGripperStop();
        auto result = result_future.get();
        RCLCPP_INFO(get_logger(), "Gripper %s canceled", kTaskName.c_str());
        goal_handle->canceled(result);
        return;
      }
      publishGripperWidthFeedback(goal_handle);
    }
    if (rclcpp::ok()) {
      const auto kResult = result_future.get();
      if (kResult->success) {
        RCLCPP_INFO(get_logger(), "Gripper %s succeeded", kTaskName.c_str());
        goal_handle->succeed(kResult);
      } else {
        RCLCPP_INFO(get_logger(), "Gripper %s failed", kTaskName.c_str());
        goal_handle->abort(kResult);
      }
    }
  }

  /// Creates a function that catches exceptions for the gripper command function and returns a
  /// result
  /// @tparam T A gripper action message type (Move, Grasp, Homing)
  /// @param[in] command_handler a function that performs the the task. Returns true on success.
  /// This function is allowed to throw a franka::Exception
  /// @return[in] enhanced command_handler that now returns a result an does not throw a
  /// franka::exception anymore
  template <typename T>
  auto withResultGenerator(const std::function<bool()>& command_handler)
      -> std::function<std::shared_ptr<typename T::Result>()> {
    return [command_handler]() {
      auto result = std::make_shared<typename T::Result>();
      try {
        result->success = command_handler();
      } catch (const std::exception& e) {
        result->success = false;
        result->error = e.what();
      }
      return result;
    };
  }

  /// Publishes the gripper width as feedback for actions
  /// @tparam T A gripper action message type (Move, Grasp, Homing)
  /// @param[in] goal_handle The goal handle from the action server
  template <typename T>
  void publishGripperWidthFeedback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>>& goal_handle) {
    auto gripper_feedback = std::make_shared<typename T::Feedback>();
    std::lock_guard<std::mutex> guard(gripper_state_mutex_);
    gripper_feedback->current_width = current_gripper_state_.width;
    goal_handle->publish_feedback(gripper_feedback);
  }

  /// Publishes the gripper width as feedback for the GripperCommand action
  void publishGripperCommandFeedback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle);
};
}  // namespace franka_gripper
