// No-load validation driver for the dual-arm Cartesian impedance chain.
//
// It performs the whole sequence in one place so that the ordering is
// explicit and observable:
//   1. deactivate the joint trajectory controller, activate the multi-mode one
//   2. wait until the impedance controller really is subscribed to its goal
//   3. hold the measured pose for `hold_seconds`
//   4. move both hands up by `move_amplitude` with a quintic-shaped ramp
//   5. hold again for `final_hold_seconds`
//   6. switch back and re-seed the trajectory controller from the CURRENT joints
//
// The reference is generated from the arm's own measured state (O_T_EE of the
// franka state), so it starts where the robot actually is. No gripper, weld or
// object interaction is commanded.

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <multi_mode_control_msgs/srv/get_controllers.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <multi_mode_control_msgs/msg/dual_cartesian_impedance_goal.hpp>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <multi_mode_control_msgs/srv/set_cartesian_impedance.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <Eigen/Geometry>

#include "dual_arm_carry_task/async_service_query.hpp"
#include "dual_arm_carry_task/controller_fault_latch.hpp"
#include "dual_arm_carry_task/mmc_state_validation.hpp"
#include "dual_arm_carry_task/mmc_activation_handshake.hpp"
#include "dual_arm_carry_task/mmc_controller_preflight.hpp"
#include "dual_arm_carry_task/payload_feedforward.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include <memory>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace
{
using GoalMsg = multi_mode_control_msgs::msg::DualCartesianImpedanceGoal;
using GetRobotStates = multi_mode_control_msgs::srv::GetRobotStates;
using SetCartesianImpedance = multi_mode_control_msgs::srv::SetCartesianImpedance;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GetControllers = multi_mode_control_msgs::srv::GetControllers;

struct ArmReference
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 7, 1> q{Eigen::Matrix<double, 7, 1>::Zero()};
};

struct ActivationHooks
{
  std::function<bool(std::chrono::steady_clock::time_point)> switchStrict;
  std::function<bool(std::chrono::steady_clock::time_point)> controllersMutuallyExclusive;
  std::function<bool(std::chrono::steady_clock::time_point, std::uint64_t&)> referenceSequence;
  std::function<bool()> protectionHealthy;
  std::function<bool()> controllerHealthy;
  std::function<bool()> goalSubscriberReady;
  std::function<void()> publishInitialReference;
  std::function<void(std::chrono::steady_clock::time_point)> waitTick;
  std::function<void()> stopPublishing;
};

const char* activationResultName(dual_arm_carry_task::MmcActivationResult result)
{
  switch (result) {
    case dual_arm_carry_task::MmcActivationResult::ACCEPTED: return "accepted";
    case dual_arm_carry_task::MmcActivationResult::SWITCH_FAILED: return "strict_switch_failed";
    case dual_arm_carry_task::MmcActivationResult::CONTROLLER_STATE_INVALID: return "active_state_not_exclusive";
    case dual_arm_carry_task::MmcActivationResult::STATUS_UNAVAILABLE: return "status_or_reference_sequence_unavailable";
    case dual_arm_carry_task::MmcActivationResult::PROTECTION_FAILED: return "task_protection_stopped";
    case dual_arm_carry_task::MmcActivationResult::CONTROLLER_UNHEALTHY: return "controller_unhealthy";
    case dual_arm_carry_task::MmcActivationResult::FIRST_REFERENCE_TIMEOUT: return "first_reference_deadline_exceeded";
  }
  return "unknown";
}

// C2 ramp used for the short move: zero velocity and acceleration at both ends.
double quinticRamp(double u)
{
  u = std::clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
}

double quinticRampRate(double u, double duration)
{
  const double du = 1.0 / std::max(1e-9, duration);
  u = std::clamp(u, 0.0, 1.0);
  return 30.0 * u * u * (1.0 - 2.0 * u + u * u) * du;
}
}  // namespace

class DualImpedanceNoLoadBridge : public rclcpp::Node
{
public:
  using StatusQuery = dual_arm_carry_task::AsyncServiceQuery<rclcpp::Client<GetControllers>>;
  using ListQuery = dual_arm_carry_task::AsyncServiceQuery<rclcpp::Client<ListControllers>>;
  using QueryStatus = dual_arm_carry_task::AsyncServiceStatus;

  // Health queries run at 50 ms. One unanswered request is abandoned after
  // 100 ms (two query intervals); it is not a fault unless complete health
  // observations stay missing for the latch's 250 ms freshness budget.
  static constexpr auto kHealthQueryInterval = std::chrono::milliseconds(50);
  static constexpr auto kSingleHealthQueryBudget = std::chrono::milliseconds(100);
  // Fresh-process DDS endpoint matching can delay the first request well past
  // the graph's service-ready signal. The first complete observation gets a
  // 1 s request budget inside a 3 s startup window; the steady policy starts
  // only after that observation.
  static constexpr auto kStartupHealthQueryBudget = std::chrono::milliseconds(1000);

  DualImpedanceNoLoadBridge() : rclcpp::Node("dual_impedance_no_load_bridge")
  {
    hold_seconds_ = declare_parameter("hold_seconds", 3.0);
    final_hold_seconds_ = declare_parameter("final_hold_seconds", 3.0);
    move_amplitude_ = declare_parameter("move_amplitude_m", 0.005);
    move_seconds_ = declare_parameter("move_seconds", 1.0);
    rate_hz_ = declare_parameter("rate_hz", 100.0);
    stop_reference_after_seconds_ = declare_parameter("stop_reference_after_seconds", -1.0);
    goal_topic_ = declare_parameter(
        "goal_topic",
        std::string("mj_left_and_mj_right/dual_cartesian_impedance_controller/desired_pose"));
    robot_states_service_ =
        declare_parameter("robot_states_service", std::string("mj_left_and_mj_right/get_robot_states"));
    jtc_action_ = declare_parameter("jtc_action",
                                    std::string("dual_panda_arm_controller/follow_joint_trajectory"));
    jtc_controller_ = declare_parameter("jtc_controller", std::string("dual_panda_arm_controller"));
    mmc_controller_ = declare_parameter("mmc_controller", std::string("sim_multi_mode_controller"));
    mmc_status_service_ = declare_parameter(
        "mmc_status_service", std::string("/sim_multi_mode_controller/get_controllers"));
    wall_timeout_seconds_ = declare_parameter("wall_timeout_seconds", 60.0);
    hold_only_ = declare_parameter("hold_only", false);
    trace_csv_ = declare_parameter("trace_csv", std::string());
    control_variant_ = declare_parameter("control_variant", std::string("fixed"));
    rod_disturbance_enabled_ = declare_parameter("rod_disturbance_enabled", false);
    // Explicit rod-weight feedforward, off by default. The disabled branch
    // sends zero and the controller law is unchanged.
    payload_feedforward_enabled_ = declare_parameter("payload_feedforward_enabled", false);
    payload_mass_kg_ = declare_parameter("payload_mass_kg", 0.64);
    feedforward_ramp_seconds_ = declare_parameter("feedforward_ramp_seconds", 0.5);
    preflight_only_ = declare_parameter("preflight_only", false);
    state_read_budget_seconds_ = declare_parameter("state_read_budget_seconds", 3.0);
    start_gate_file_ = declare_parameter("start_gate_file", std::string());
    start_gate_timeout_seconds_ = declare_parameter("start_gate_timeout_seconds", 60.0);

    goal_pub_ = create_publisher<GoalMsg>(goal_topic_, rclcpp::QoS(10));
    robot_states_client_ = create_client<GetRobotStates>(robot_states_service_);
    switch_client_ = create_client<SwitchController>("/controller_manager/switch_controller");
    list_controllers_client_ = create_client<ListControllers>("/controller_manager/list_controllers");
    mmc_status_client_ = create_client<GetControllers>(mmc_status_service_);
    impedance_parameters_client_ = create_client<SetCartesianImpedance>(
        "/mj_left_and_mj_right/dual_cartesian_impedance_controller/parameters");
    if (rod_disturbance_enabled_) {
      rod_force_arm_pub_ = create_publisher<std_msgs::msg::Bool>(
          "/contact_probe/rod_force_arm", rclcpp::QoS(1).reliable());
    }
    jtc_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, jtc_action_);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joints_mutex_);
          latest_joints_ = *msg;
          latest_joints_wall_ = std::chrono::steady_clock::now();
        });

    RCLCPP_INFO(get_logger(),
                "[NO_LOAD] hold=%.1fs hold_only=%d move=%.1fmm/%.1fs holds=%.1fs "
                "goal_topic_parameter=%s resolved_goal_topic=%s goal_type=%s state_service=%s",
                hold_seconds_, hold_only_ ? 1 : 0, move_amplitude_ * 1000.0, move_seconds_, final_hold_seconds_,
                goal_topic_.c_str(), goal_pub_->get_topic_name(),
                "multi_mode_control_msgs/msg/DualCartesianImpedanceGoal",
                robot_states_client_->get_service_name());
    if (stop_reference_after_seconds_ >= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "[NO_LOAD] watchdog debug enabled: stop publishing reference at %.3f s",
                  stop_reference_after_seconds_);
    }
  }

  bool run()
  {
    bool use_sim_time = false;
    const bool has_sim_time = get_parameter("use_sim_time", use_sim_time);
    RCLCPP_INFO(get_logger(),
                "[MMC_CLOCK_CHECK] node=%s parameter=use_sim_time available=%s value=%s phase=%s",
                get_fully_qualified_name(), has_sim_time ? "true" : "false",
                use_sim_time ? "true" : "false", preflight_only_ ? "bridge-preflight" : "hold");
    if (!has_sim_time || !use_sim_time)
      return fail("use_sim_time is not enabled for bridge");

    if (preflight_only_) {
      if (!validateGoalInterface()) return fail("preflight goal topic/type/config mismatch");
      const auto interface_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
      if (!controllerStates(false, interface_deadline))
        return fail("preflight expected JTC active and MMC inactive");
      if (!mmcServiceResponds(interface_deadline)) return fail("preflight MMC status service unavailable");
      if (!switchControllers({}, {}, interface_deadline, "preflight-noop"))
        return fail("preflight switch service request failed");
      if (!controllerStates(false, interface_deadline))
        return fail("preflight controller state changed after no-op switch");
      RCLCPP_INFO(get_logger(), "[MMC_CONFIG_PREFLIGHT] state_service=%s type=multi_mode_control_msgs/srv/GetRobotStates "
                  "status_service=%s type=multi_mode_control_msgs/srv/GetControllers "
                  "switch_service=%s type=controller_manager_msgs/srv/SwitchController",
                  robot_states_client_->get_service_name(), mmc_status_client_->get_service_name(),
                  switch_client_->get_service_name());
      std::array<ArmReference, 2> preflight;
      const bool ok = readArms(preflight, std::chrono::duration<double>(state_read_budget_seconds_),
                               "pre-lift-preflight");
      RCLCPP_INFO(get_logger(), "[MMC_STATE_PREFLIGHT] service=%s result=%s "
                  "source=task_joint_feedback_MoveIt_FK order=mj_left,mj_right "
                  "frames=mj_left_link0,mj_right_link0 elapsed_wall_sec=%.6f "
                  "goal_topic=%s goal_type=multi_mode_control_msgs/msg/DualCartesianImpedanceGoal",
                  robot_states_client_->get_service_name(), ok ? "valid" : "failed",
                  last_state_read_elapsed_, goal_pub_->get_topic_name());
      return ok;
    }
    std::string fault_reason;
    // Bounded integration check in the same process: exercise the continuous
    // health-query path across first service discovery before committing to the
    // strict switch. JTC is still active, so the expected state is MMC=inactive.
    if (!runHealthContinuityProbe(1.0, fault_reason)) {
      return fail("MMC health continuity probe: " + fault_reason);
    }
    // The bridge may be armed before the grasp/lift. Keep health queries and
    // discovery alive while waiting, then capture the start reference only
    // after the task confirms the lift is complete.
    if (!waitForStartGate(fault_reason)) {
      return fail("MMC start gate: " + fault_reason);
    }
    const bool direct_controller_state_source =
        robot_states_service_.find("mj_left_and_mj_right") != std::string::npos;
    health_expect_mmc_active_ = true;

    std::array<ArmReference, 2> start;
    dual_arm_carry_task::MmcActivationResult activation_result =
        dual_arm_carry_task::MmcActivationResult::FIRST_REFERENCE_TIMEOUT;
    const auto activation_started = std::chrono::steady_clock::now();
    activation_started_ = activation_started;
    const double activation_budget_sec = direct_controller_state_source ? 3.0 : 1.5;
    if (direct_controller_state_source) {
      // The standalone bridge talks to SwitchEngine's state service. That
      // service reads RobotData, whose hardware snapshot is populated when
      // MultiModeController activates. Capture the pose after the strict
      // switch for this source; the task-owned MoveIt-FK service already has
      // a valid pre-switch snapshot and keeps the legacy ordering below.
      const auto deadline = activation_started + std::chrono::seconds(3);
      if (!switchControllers({mmc_controller_}, {jtc_controller_}, deadline, "activate-mmc-direct"))
        return fail("MMC direct activation switch failed");
      if (!controllerStates(true, deadline))
        return fail("MMC direct activation state is not exclusive");
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
             goal_pub_->get_subscription_count() == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (goal_pub_->get_subscription_count() == 0)
        return fail("MMC direct goal subscriber did not appear");
      if (!readArms(start, std::chrono::duration<double>(state_read_budget_seconds_),
                    "post-switch-direct-state"))
        return fail(last_state_read_error_);
      reference_start_ = start;
      RCLCPP_INFO(get_logger(),
                  "[MMC_HOLD_ENTRY] state_read=success service=%s source=SwitchEngine_after_activation "
                  "reference_frame=mj_left_link0,mj_right_link0 elapsed_wall_sec=%.6f",
                  robot_states_client_->get_service_name(), last_state_read_elapsed_);
      RCLCPP_INFO(get_logger(), "[NO_LOAD] initial left=(%.4f,%.4f,%.4f) right=(%.4f,%.4f,%.4f)",
                  start[0].position.x(), start[0].position.y(), start[0].position.z(),
                  start[1].position.x(), start[1].position.y(), start[1].position.z());
      std::uint64_t baseline_sequence = 0;
      if (!getReferenceSequence(deadline, baseline_sequence))
        return fail("MMC direct status unavailable before first reference");
      publishGoal(start, 0.0, 0.0, 0.0);
      ++first_reference_publish_count_;
      first_reference_publish_wall_ = std::chrono::steady_clock::now();
      bool accepted = false;
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        if (!checkControllerHealth(fault_reason)) return fail("MMC direct health: " + fault_reason);
        std::uint64_t sequence = baseline_sequence;
        if (getReferenceSequence(deadline, sequence) && sequence > baseline_sequence) {
          accepted = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      activation_result = accepted ? dual_arm_carry_task::MmcActivationResult::ACCEPTED :
                                      dual_arm_carry_task::MmcActivationResult::FIRST_REFERENCE_TIMEOUT;
      if (accepted)
        RCLCPP_INFO(get_logger(), "[MMC_FIRST_REFERENCE_SENT] direct activation accepted");
    } else {
      // The task-owned MoveIt-FK service is valid before the switch.
      if (!readArms(start, std::chrono::duration<double>(state_read_budget_seconds_),
                    "post-lift-before-switch"))
        return fail(last_state_read_error_);
      RCLCPP_INFO(get_logger(), "[MMC_HOLD_ENTRY] state_read=success service=%s source=task_joint_feedback_MoveIt_FK "
                  "reference_frame=mj_left_link0,mj_right_link0 elapsed_wall_sec=%.6f",
                  robot_states_client_->get_service_name(), last_state_read_elapsed_);
      reference_start_ = start;
      RCLCPP_INFO(get_logger(), "[NO_LOAD] initial left=(%.4f,%.4f,%.4f) right=(%.4f,%.4f,%.4f)",
                  start[0].position.x(), start[0].position.y(), start[0].position.z(),
                  start[1].position.x(), start[1].position.y(), start[1].position.z());
      constexpr auto kFirstReferenceBudget = std::chrono::milliseconds(1500);
      const auto activation_deadline = activation_started + kFirstReferenceBudget;
      auto hooks = makeActivationHooks(start, fault_reason);
      activation_result = dual_arm_carry_task::activateAndPrimeMmc(hooks, activation_deadline);
    }
    const double activation_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - activation_started).count();
    RCLCPP_INFO(get_logger(), "[MMC_FIRST_REFERENCE] result=%s accepted_seq=%llu elapsed_wall_sec=%.6f "
                "budget_sec=%.3f watchdog_startup_grace_sec=2.000",
                activation_result == dual_arm_carry_task::MmcActivationResult::ACCEPTED ? "accepted" : "failed",
                static_cast<unsigned long long>(last_reference_sequence_), activation_elapsed,
                activation_budget_sec);
    if (activation_result == dual_arm_carry_task::MmcActivationResult::ACCEPTED) {
      RCLCPP_INFO(get_logger(), "[MMC_REFERENCE_RECEIVED] accepted_seq=%llu publications=%llu "
                  "callback_after_first_publish_ms=%.3f",
                  static_cast<unsigned long long>(last_reference_sequence_),
                  static_cast<unsigned long long>(first_reference_publish_count_),
                  first_reference_publish_wall_ == std::chrono::steady_clock::time_point{} ? 0.0 :
                    std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - first_reference_publish_wall_).count());
    }
    if (activation_result != dual_arm_carry_task::MmcActivationResult::ACCEPTED) {
      const char* reason = fault_reason.empty() ? activationResultName(activation_result) : fault_reason.c_str();
      return fail(std::string("MMC activation/first-reference handshake: ") + reason);
    }

    if (!configureDirectionalParameters())
      return fail("directional impedance parameter service failed before HOLD");

    const auto began = std::chrono::steady_clock::now();
    const double sim_began = now().seconds();
    // Rod payload feedforward. The first accepted MMC reference and the
    // already-confirmed carried state define the enable instant; the force is
    // ramped linearly in simulation time. The target is fixed (half the rod
    // weight per arm) and is NOT adapted from load feedback this round.
    const double payload_ff_force_n =
        payload_ff::perArmForceN(payload_mass_kg_, payload_feedforward_enabled_);
    const Eigen::Vector3d payload_ff_world(0.0, 0.0, payload_ff_force_n);
    // Both mj_*_link0 bodies have quat=(1,0,0,0) in the MJCF, so the base frame
    // used by the controller's zero Jacobian has the world orientation. The
    // world->base rotation is therefore identity; translation is irrelevant to
    // a pure force.
    const Eigen::Vector3d payload_ff_base =
        payload_ff::worldToBaseForce(Eigen::Quaterniond::Identity(), payload_ff_world);
    bool payload_ff_target_logged = !payload_feedforward_enabled_;
    RCLCPP_INFO(get_logger(),
                "[PAYLOAD_FF] enabled=%d mass_kg=%.6f target_per_arm_n=%.6f "
                "world_force=(0,0,%.6f)N base_force=(%.6f,%.6f,%.6f)N base_frame=link0_identity "
                "ramp_sim_sec=%.3f start_sim=%.9f "
                "torque_path=J_linear^T F -> shared_magnitude_rate_limits",
                payload_feedforward_enabled_ ? 1 : 0, payload_mass_kg_, payload_ff_force_n,
                payload_ff_world.z(), payload_ff_base.x(), payload_ff_base.y(), payload_ff_base.z(),
                feedforward_ramp_seconds_, sim_began);
    if (rod_disturbance_enabled_) {
      std_msgs::msg::Bool arm_request;
      arm_request.data = true;
      rod_force_arm_pub_->publish(arm_request);
      RCLCPP_INFO(get_logger(), "[ROD_FORCE_REQUEST] event=arm sim_time=%.9f world_force=(+2,0,0)N "
                  "start_after_sim_sec=1.0 duration_sim_sec=plugin_configured "
                  "(CONTACT_PROBE_ROD_FORCE_DURATION_SEC)", sim_began);
    }
    int cycle = 0;
    auto elapsed = [&began]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    };
    const double move_begin = hold_only_ ? hold_seconds_ : hold_seconds_;
    const double move_end = move_begin + move_seconds_;
    const double sequence_end = move_end + final_hold_seconds_;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    std::ofstream trace;
    if (!trace_csv_.empty()) {
      trace.open(trace_csv_, std::ios::out | std::ios::trunc);
      if (!trace) return fail("cannot open MMC hold trace");
      trace << "wall_elapsed_sec,sim_elapsed_sec,sim_absolute_sec,";
      for (const char* arm : {"left", "right"}) {
        for (const char* pose : {"expected", "actual"})
          for (const char* axis : {"x", "y", "z"}) trace << arm << '_' << pose << '_' << axis << ',';
        for (const char* pose : {"expected", "actual"})
          for (const char* axis : {"qx", "qy", "qz", "qw"}) trace << arm << '_' << pose << '_' << axis << ',';
        for (const char* state : {"q_ref", "q_actual"})
          for (int j = 1; j <= 7; ++j) trace << arm << '_' << state << '_' << j << ',';
      }
      const char* twist_axes[] = {"vx", "vy", "vz", "wx", "wy", "wz"};
      for (int arm = 0; arm < 2; ++arm)
        for (int axis = 0; axis < 6; ++axis)
          trace << (arm == 0 ? "left" : "right") << "_twist_" << twist_axes[axis] << ',';
      trace << "reference_publish_count,reference_interval_ms,"
               "health_query_count,health_success_count,health_miss_count,"
               "health_consecutive_miss,health_recovery_count,"
               "health_last_query_elapsed_ms,health_last_success_age_ms,"
               "health_in_flight,"
               "left_payload_ff_force_n,right_payload_ff_force_n,payload_ff_scale\n";
    }

    while (rclcpp::ok() && (hold_only_ ? now().seconds() - sim_began < hold_seconds_ : elapsed() < sequence_end)) {
      const double t = elapsed();
      double offset = 0.0;
      double rate = 0.0;
      if (!hold_only_ && t >= move_begin && t < move_end) {
        const double u = (t - move_begin) / move_seconds_;
        offset = move_amplitude_ * quinticRamp(u);
        rate = move_amplitude_ * quinticRampRate(u, move_seconds_);
      } else if (!hold_only_ && t >= move_end) {
        offset = move_amplitude_;
      }
      const bool publish_reference = stop_reference_after_seconds_ < 0.0 ||
          t < stop_reference_after_seconds_;
      const double payload_ff_scale = payload_feedforward_enabled_ ?
          payload_ff::rampScale(now().seconds() - sim_began, feedforward_ramp_seconds_) : 0.0;
      const double payload_ff_now = payload_ff_force_n * payload_ff_scale;
      if (payload_feedforward_enabled_ && !payload_ff_target_logged &&
          payload_ff_scale >= 1.0) {
        payload_ff_target_logged = true;
        RCLCPP_INFO(get_logger(),
                    "[PAYLOAD_FF] target_reached sim_time=%.9f per_arm_n=%.6f "
                    "elapsed_from_first_reference_sim_sec=%.6f",
                    now().seconds(), payload_ff_now, now().seconds() - sim_began);
      }
      // Health queries are polled asynchronously: this call never waits for a
      // service response, so a slow status query cannot delay the reference.
      if (!pollControllerHealthNonBlocking(fault_reason)) {
        // Do not publish another goal, switch controllers, or claim success.
        return fail("MMC status: " + fault_reason);
      }
      if (publish_reference) {
        publishGoal(start, offset, rate, payload_ff_now);
        if (trace && cycle % 5 == 0 &&
            !traceSample(trace, start, t, now().seconds() - sim_began,
                         payload_ff_now, payload_ff_scale))
          return fail("MMC trace state unavailable");
      } else if (cycle % 100 == 0) {
        RCLCPP_WARN(get_logger(),
                    "[NO_LOAD] watchdog debug: reference publishing is stopped at t=%.3f s",
                    t);
      }
      if (cycle % 10 == 0) {
        double pe = 0.0, re = 0.0;
        if (measureTracking(offset, pe, re)) {
          max_tracking_position_ = std::max(max_tracking_position_, pe);
          max_tracking_rotation_ = std::max(max_tracking_rotation_, re);
          if (t >= move_end) max_final_hold_position_ = std::max(max_final_hold_position_, pe);
        }
      }
      ++cycle;
      if (elapsed() > wall_timeout_seconds_) return fail("sequence wall-clock timeout");
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(period));
    }
    cancelRodForceRequest("hold_complete");
    // Keep the final pose commanded while the switch back is prepared.
    if (stop_reference_after_seconds_ < 0.0 && !hold_only_) {
      for (int i = 0; i < 5; ++i) {
        if (!checkControllerHealth(fault_reason)) return fail("MMC status: " + fault_reason);
        publishGoal(start, move_amplitude_, 0.0, payload_ff_force_n);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
      }
    }
    RCLCPP_INFO(get_logger(),
        "[NO_LOAD] impedance sequence complete after %.2f s tracking_max=(%.6f m, %.6f deg) "
        "final_hold_max=(%.6f m)",
        elapsed(), max_tracking_position_, max_tracking_rotation_ * 180.0 / M_PI,
        max_final_hold_position_);

    if (stop_reference_after_seconds_ >= 0.0) {
      RCLCPP_INFO(get_logger(),
                  "[NO_LOAD] watchdog debug complete; skipping JTC switch-back so the "
                  "latched controller state can be inspected");
      return true;
    }

    if (!checkControllerHealth(fault_reason)) return fail("MMC status: " + fault_reason);
    const auto switch_back_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    if (!switchControllers({jtc_controller_}, {mmc_controller_}, switch_back_deadline, "switch-back-jtc"))
      return fail("switch back");
    if (!controllerStates(false, std::chrono::steady_clock::now() + std::chrono::seconds(2)))
      return fail("JTC active/MMC inactive state not confirmed after switchback");
    if (!seedJtcFromCurrentState()) return fail("seed trajectory controller from current state");
    RCLCPP_INFO(get_logger(), "[NO_LOAD] returned to the trajectory controller");
    return true;
  }

private:
  bool validateGoalInterface()
  {
    const std::string expected = "/mj_left_and_mj_right/dual_cartesian_impedance_controller/desired_pose";
    std::string resolved = goal_pub_->get_topic_name();
    if (!resolved.empty() && resolved.front() != '/') resolved.insert(resolved.begin(), '/');
    if (resolved != expected) {
      RCLCPP_ERROR(get_logger(),
          "[MMC_CONFIG_PREFLIGHT] goal_topic_mismatch parameter=%s resolved=%s expected=%s",
          goal_topic_.c_str(), resolved.c_str(), expected.c_str());
      return false;
    }
    constexpr char kGoalType[] = "multi_mode_control_msgs/msg/DualCartesianImpedanceGoal";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto graph = get_topic_names_and_types();
      const auto it = graph.find(expected);
      if (it != graph.end() &&
          std::find(it->second.begin(), it->second.end(), kGoalType) != it->second.end()) {
        RCLCPP_INFO(get_logger(),
            "[MMC_CONFIG_PREFLIGHT] goal_topic_parameter=%s resolved=%s goal_type=%s "
            "hold_seconds=%.3f hold_only=%d jtc=%s mmc=%s",
            goal_topic_.c_str(), expected.c_str(), kGoalType, hold_seconds_, hold_only_ ? 1 : 0,
            jtc_controller_.c_str(), mmc_controller_.c_str());
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_ERROR(get_logger(), "[MMC_CONFIG_PREFLIGHT] goal_topic_type_missing topic=%s expected_type=%s",
                 expected.c_str(), kGoalType);
    return false;
  }

  bool mmcServiceResponds(std::chrono::steady_clock::time_point deadline)
  {
    const auto discovery = dual_arm_carry_task::waitForServiceUntil(
        deadline, [this]() { return mmc_status_client_->service_is_ready(); },
        [](auto next) { std::this_thread::sleep_until(next); });
    if (discovery != dual_arm_carry_task::ServiceDiscoveryResult::READY) {
      RCLCPP_ERROR(get_logger(), "[MMC_CONFIG_PREFLIGHT] result=service_discovery_timeout service=%s",
                   mmc_status_client_->get_service_name());
      return false;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      RCLCPP_ERROR(get_logger(), "[MMC_CONFIG_PREFLIGHT] result=request_timeout_before_send service=%s",
                   mmc_status_client_->get_service_name());
      return false;
    }
    auto future = mmc_status_client_->async_send_request(std::make_shared<GetControllers::Request>());
    if (future.wait_for(remaining) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[MMC_CONFIG_PREFLIGHT] result=request_response_timeout service=%s",
                   mmc_status_client_->get_service_name());
      return false;
    }
    GetControllers::Response::SharedPtr response;
    try {
      response = future.get();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "[MMC_CONFIG_PREFLIGHT] result=request_exception service=%s what=%s",
                   mmc_status_client_->get_service_name(), error.what());
      return false;
    }
    if (!response) {
      RCLCPP_ERROR(get_logger(), "[MMC_CONFIG_PREFLIGHT] result=empty_response service=%s",
                   mmc_status_client_->get_service_name());
      return false;
    }
    RCLCPP_INFO(get_logger(), "[MMC_CONFIG_PREFLIGHT] status_service=%s response=received faulted=%d",
                mmc_status_client_->get_service_name(), response->faulted ? 1 : 0);
    return !response->faulted;
  }

  bool getReferenceSequence(std::chrono::steady_clock::time_point deadline, std::uint64_t& sequence)
  {
    if (!mmc_status_client_->service_is_ready()) return false;
    auto future = mmc_status_client_->async_send_request(std::make_shared<GetControllers::Request>());
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0 || future.wait_for(remaining) != std::future_status::ready) return false;
    const auto response = future.get();
    if (response->faulted) return false;
    const auto now_ns = steadyNowNs();
    if (response->status_steady_time_ns == 0 || now_ns < response->status_steady_time_ns ||
        now_ns - response->status_steady_time_ns > 500000000ULL)
      return false;
    if (response->reference_sequence > 0 &&
        (response->reference_steady_time_ns == 0 || now_ns < response->reference_steady_time_ns ||
         now_ns - response->reference_steady_time_ns > 500000000ULL))
      return false;
    sequence = response->reference_sequence;
    return true;
  }

  ActivationHooks makeActivationHooks(const std::array<ArmReference, 2>& start,
                                      std::string& fault_reason)
  {
    return ActivationHooks{
      [this](auto deadline) {
        return switchControllers({mmc_controller_}, {jtc_controller_}, deadline, "activate-mmc");
      },
      [this](auto deadline) { return controllerStates(true, deadline); },
      [this](auto deadline, std::uint64_t& sequence) {
        const bool ok = getReferenceSequence(deadline, sequence);
        if (ok && !reference_baseline_logged_) {
          reference_baseline_logged_ = true;
          RCLCPP_INFO(get_logger(), "[MMC_REFERENCE_BASELINE] active_controller_sequence=%llu",
                      static_cast<unsigned long long>(sequence));
        }
        if (ok && sequence > last_reference_sequence_) {
          RCLCPP_INFO(get_logger(), "[MMC_REFERENCE_RECEIVED] accepted_seq=%llu",
                      static_cast<unsigned long long>(sequence));
        }
        if (ok) last_reference_sequence_ = sequence;
        return ok;
      },
      [this]() { return rclcpp::ok(); },
      [this, &fault_reason]() { return checkControllerHealth(fault_reason); },
      [this]() {
        const bool ready = goal_pub_ && goal_pub_->get_subscription_count() > 0;
        if (ready && !goal_subscriber_seen_) {
          goal_subscriber_seen_ = true;
          RCLCPP_INFO(get_logger(), "[MMC_GOAL_SUBSCRIBER] appeared topic=%s after_switch_elapsed_ms=%.3f",
                      goal_pub_->get_topic_name(),
                      std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - activation_started_).count());
        }
        return ready;
      },
      [this, &start]() {
        // First reference: zero feedforward; the 0.5 s ramp starts here.
        publishGoal(start, 0.0, 0.0, 0.0);
        ++first_reference_publish_count_;
        if (first_reference_publish_count_ == 1) {
          first_reference_publish_wall_ = std::chrono::steady_clock::now();
          RCLCPP_INFO(get_logger(), "[MMC_FIRST_REFERENCE_SENT] frame=left_link0,right_link0 "
                      "twist=zero q=measured publication=%llu",
                      static_cast<unsigned long long>(first_reference_publish_count_));
        }
      },
      [](auto deadline) {
        const auto next = std::min(deadline, std::chrono::steady_clock::now() + std::chrono::milliseconds(10));
        std::this_thread::sleep_until(next);
      },
      [this]() { goal_pub_.reset(); }
    };
  }

  // Synchronous health check used only where the caller is already inside a
  // bounded handshake/switch budget (activation and switch-back). The steady
  // reference loop uses pollControllerHealthNonBlocking instead.
  bool checkControllerHealth(std::string& reason)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!fault_latch_.healthy()) {
      reason = fault_latch_.failure();
      return false;
    }
    if (now < next_health_check_) return true;
    next_health_check_ = now + kHealthQueryInterval;

    if (!startHealthRound(now)) {
      if (!fault_latch_.observeQueryMiss(steadyNowNs(), health_freshness_budget_ns_)) {
        reason = fault_latch_.failure();
        reportHealthFailure(reason);
        return false;
      }
      return true;
    }
    health_round_started_ = now;

    const auto deadline = now + kSingleHealthQueryBudget;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto round = pollHealthRound(std::chrono::steady_clock::now(), now,
                                         kSingleHealthQueryBudget, reason);
      if (round == HealthRoundStatus::COMPLETE) return fault_latch_.healthy();
      if (round == HealthRoundStatus::ABANDONED) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (status_query_.inFlight() || list_query_.inFlight()) {
      status_query_.abandon(*mmc_status_client_);
      list_query_.abandon(*list_controllers_client_);
      noteHealthMiss(now, "request_timeout");
    }
    if (!fault_latch_.observeQueryMiss(steadyNowNs(), health_freshness_budget_ns_)) {
      reason = fault_latch_.failure();
      reportHealthFailure(reason);
      return false;
    }
    return true;
  }

  // Steady-loop path: starts at most one request per service, polls them with
  // wait_for(0), and never blocks reference publication.
  bool pollControllerHealthNonBlocking(std::string& reason)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!fault_latch_.healthy()) {
      reason = fault_latch_.failure();
      return false;
    }

    if (status_query_.inFlight() || list_query_.inFlight()) {
      const auto round = pollHealthRound(now, health_round_started_,
                                         kSingleHealthQueryBudget, reason);
      if (round == HealthRoundStatus::COMPLETE) {
        next_health_check_ = now + kHealthQueryInterval;
        return fault_latch_.healthy();
      }
      if (round == HealthRoundStatus::ABANDONED) {
        // One request budget has already elapsed; retry immediately so a
        // second response can still arrive inside the freshness budget.
        next_health_check_ = now;
      }
    }

    if (!status_query_.inFlight() && !list_query_.inFlight() &&
        std::chrono::steady_clock::now() >= next_health_check_) {
      const auto start_now = std::chrono::steady_clock::now();
      if (startHealthRound(start_now)) health_round_started_ = start_now;
    }

    if (!fault_latch_.observeQueryMiss(steadyNowNs(), health_freshness_budget_ns_)) {
      reason = fault_latch_.failure();
      reportHealthFailure(reason);
      return false;
    }
    return true;
  }

  bool startHealthRound(std::chrono::steady_clock::time_point now)
  {
    if (status_query_.inFlight() || list_query_.inFlight()) return false;
    if (!mmc_status_client_->service_is_ready() || !list_controllers_client_->service_is_ready()) {
      noteHealthMiss(now, "service_not_ready");
      next_health_check_ = now + kHealthQueryInterval;
      return false;
    }
    const bool status_started = status_query_.start(*mmc_status_client_, now);
    const bool list_started = list_query_.start(*list_controllers_client_, now);
    if (!status_started || !list_started) {
      if (status_started) status_query_.abandon(*mmc_status_client_);
      if (list_started) list_query_.abandon(*list_controllers_client_);
      noteHealthMiss(now, "request_start_failed");
      next_health_check_ = now + kHealthQueryInterval;
      return false;
    }
    // Bind the expected controller state to this request round. A response
    // that was sent before the gate/switch may arrive after the expectation
    // changes; it must be judged against the state that was true when the
    // request was issued.
    health_round_expect_mmc_active_ = health_expect_mmc_active_;
    ++health_query_count_;
    if (health_query_count_ <= 5) {
      RCLCPP_INFO(get_logger(),
                  "[MMC_HEALTH_QUERY] round=%llu started status_service=%s controllers_service=%s",
                  static_cast<unsigned long long>(health_query_count_),
                  mmc_status_client_->get_service_name(),
                  list_controllers_client_->get_service_name());
    }
    return true;
  }

  enum class HealthRoundStatus { PENDING, COMPLETE, ABANDONED };

  // Non-blocking poll of both outstanding requests. `round_started` is only
  // used to record the completed round's wall duration.
  HealthRoundStatus pollHealthRound(std::chrono::steady_clock::time_point now,
                                    std::chrono::steady_clock::time_point round_started,
                                    std::chrono::milliseconds single_query_budget,
                                    std::string& reason)
  {
    const auto status = status_query_.poll(now, single_query_budget);
    const auto listed = list_query_.poll(now, single_query_budget);
    if (status == QueryStatus::READY && listed == QueryStatus::READY) {
      last_health_query_elapsed_ms_ =
          std::chrono::duration<double, std::milli>(now - round_started).count();
      evaluateHealthRound(now, reason);
      return HealthRoundStatus::COMPLETE;
    }
    if (status == QueryStatus::TIMEOUT || listed == QueryStatus::TIMEOUT) {
      const char* category = status == QueryStatus::TIMEOUT && listed == QueryStatus::TIMEOUT ?
          "both_requests_timeout" :
          (status == QueryStatus::TIMEOUT ? "status_request_timeout" : "controllers_request_timeout");
      status_query_.abandon(*mmc_status_client_);
      list_query_.abandon(*list_controllers_client_);
      noteHealthMiss(now, category);
      return HealthRoundStatus::ABANDONED;
    }
    return HealthRoundStatus::PENDING;
  }

  void evaluateHealthRound(std::chrono::steady_clock::time_point now, std::string& reason)
  {
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    const auto status = status_query_.response();
    const auto listed = list_query_.response();
    std::string mmc_state;
    std::string jtc_state;
    bool mmc_seen = false;
    bool jtc_seen = false;
    for (const auto& controller : listed->controller) {
      if (controller.name == mmc_controller_) {
        mmc_seen = true;
        mmc_state = controller.state;
      }
      if (controller.name == jtc_controller_) {
        jtc_seen = true;
        jtc_state = controller.state;
      }
    }
    const bool mmc_active = mmc_seen && mmc_state == "active";
    const bool jtc_active = jtc_seen && jtc_state == "active";
    const bool state_matches = health_round_expect_mmc_active_ ?
        (mmc_active && !jtc_active) : (!mmc_active && jtc_active);
    if (!mmc_seen || !jtc_seen || !state_matches) {
      const std::string detail =
          "expected " + std::string(health_round_expect_mmc_active_ ? "MMC=active,JTC=inactive" :
                                                                     "MMC=inactive,JTC=active") +
          " but got " + mmc_controller_ + "=" + (mmc_seen ? mmc_state : "<missing>") +
          "," + jtc_controller_ + "=" + (jtc_seen ? jtc_state : "<missing>");
      fault_latch_.latchControllerStateConflict(detail);
    } else if (!fault_latch_.observe(
                 true, true, true, status->faulted, status->fault_source,
                 status->fault_sequence, status->fault_steady_time_ns,
                 status->status_steady_time_ns, now_ns)) {
      // The latch already holds the precise reason (fault/stale/clock).
    }

    if (!fault_latch_.healthy()) {
      reason = fault_latch_.failure();
      reportHealthFailure(reason);
      return;
    }
    ++health_success_count_;
    if (health_success_count_ <= 5) {
      RCLCPP_INFO(get_logger(),
                  "[MMC_HEALTH_OK] successes=%llu query_elapsed_ms=%.3f "
                  "mmc_state=%s jtc_state=%s faulted=%d",
                  static_cast<unsigned long long>(health_success_count_),
                  last_health_query_elapsed_ms_, mmc_state.c_str(), jtc_state.c_str(),
                  status->faulted ? 1 : 0);
    }
    if (health_consecutive_miss_ > 0) {
      ++health_recovery_count_;
      RCLCPP_INFO(get_logger(),
                  "[MMC_HEALTH_RECOVERED] consecutive_misses=%llu total_misses=%llu "
                  "query_elapsed_ms=%.3f last_success_age_ms=%.3f",
                  static_cast<unsigned long long>(health_consecutive_miss_),
                  static_cast<unsigned long long>(health_miss_count_),
                  last_health_query_elapsed_ms_, currentHealthAgeMs(now_ns));
      health_consecutive_miss_ = 0;
    }
  }

  void noteHealthMiss(std::chrono::steady_clock::time_point now, const char* category)
  {
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    ++health_miss_count_;
    ++health_consecutive_miss_;
    last_health_miss_category_ = category;
    RCLCPP_WARN(get_logger(),
                "[MMC_HEALTH_MISS] category=%s misses=%llu consecutive=%llu "
                "last_success_age_ms=%.3f query_interval_ms=%lld single_query_budget_ms=%lld "
                "freshness_budget_ms=%llu",
                category, static_cast<unsigned long long>(health_miss_count_),
                static_cast<unsigned long long>(health_consecutive_miss_),
                currentHealthAgeMs(now_ns),
                static_cast<long long>(kHealthQueryInterval.count()),
                static_cast<long long>(kSingleHealthQueryBudget.count()),
                static_cast<unsigned long long>(health_freshness_budget_ns_ / 1000000ULL));
  }

  void reportHealthFailure(const std::string& reason)
  {
    RCLCPP_ERROR(get_logger(),
                 "[MMC_HEALTH_LATCH] reason=%s queries=%llu successes=%llu misses=%llu "
                 "last_miss=%s last_query_elapsed_ms=%.3f last_success_age_ms=%.3f "
                 "elapsed_wall_sec=%.3f",
                 reason.c_str(), static_cast<unsigned long long>(health_query_count_),
                 static_cast<unsigned long long>(health_success_count_),
                 static_cast<unsigned long long>(health_miss_count_),
                 last_health_miss_category_,
                 last_health_query_elapsed_ms_, currentHealthAgeMs(steadyNowNs()),
                 std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                               health_probe_started_).count());
  }

  double currentHealthAgeMs(std::uint64_t now_ns) const
  {
    const auto last = fault_latch_.lastGoodNs();
    if (last == 0 || now_ns < last) return -1.0;
    return static_cast<double>(now_ns - last) / 1e6;
  }

  // Optional arming gate. The task starts the bridge before the grasp/lift so
  // service discovery and the health probe are not an extra post-lift hover.
  // While waiting, health queries continue; the freshness budget never
  // expires just because the lift is still in progress.
  bool waitForStartGate(std::string& reason)
  {
    if (start_gate_file_.empty()) return true;
    const auto previous_budget = health_freshness_budget_ns_;
    // No reference is published while the bridge is armed; a longer bounded
    // freshness budget covers the grasp/lift load without weakening the
    // 250 ms HOLD policy.
    health_freshness_budget_ns_ = 2000000000ULL;
    const auto began = std::chrono::steady_clock::now();
    const auto deadline = began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(start_gate_timeout_seconds_));
    RCLCPP_INFO(get_logger(),
                "[MMC_START_GATE] waiting file=%s timeout_sec=%.3f freshness_budget_ms=%llu",
                start_gate_file_.c_str(), start_gate_timeout_seconds_,
                static_cast<unsigned long long>(health_freshness_budget_ns_ / 1000000ULL));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (std::ifstream(start_gate_file_).good()) {
        // Drop any pre-gate round. Its response may be generated on either
        // side of the controller switch; the first post-gate round is sent
        // after activation with the post-switch expectation.
        status_query_.abandon(*mmc_status_client_);
        list_query_.abandon(*list_controllers_client_);
        health_freshness_budget_ns_ = previous_budget;
        RCLCPP_INFO(get_logger(), "[MMC_START_GATE] released after %.3f s",
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count());
        return true;
      }
      if (!pollControllerHealthNonBlocking(reason)) {
        health_freshness_budget_ns_ = previous_budget;
        reason = "health query unavailable while waiting: " + reason;
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    health_freshness_budget_ns_ = previous_budget;
    reason = "start_gate_timeout";
    RCLCPP_ERROR(get_logger(),
                 "[MMC_START_GATE] result=timeout file=%s elapsed_sec=%.3f",
                 start_gate_file_.c_str(),
                 std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count());
    return false;
  }

  // Bounded pre-switch integration probe. It uses the same query path as the
  // hold loop and confirms that discovery and repeated updates work before the
  // strict controller switch.
  bool runHealthContinuityProbe(double seconds, std::string& reason)
  {
    health_probe_started_ = std::chrono::steady_clock::now();
    health_expect_mmc_active_ = false;
    const auto began = std::chrono::steady_clock::now();
    // DDS discovery of a newly spawned bridge process is allowed its own
    // bounded grace; the steady freshness budget only applies once a complete
    // health observation has been confirmed.
    const auto discovery_deadline = began + std::chrono::seconds(3);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < discovery_deadline &&
           !(mmc_status_client_->service_is_ready() && list_controllers_client_->service_is_ready())) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const double discovery_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    if (!mmc_status_client_->service_is_ready() || !list_controllers_client_->service_is_ready()) {
      reason = "service_discovery_timeout";
      RCLCPP_ERROR(get_logger(),
                   "[MMC_HEALTH_PROBE] result=fail reason=%s discovery_sec=%.3f "
                   "status_service_ready=%d controllers_service_ready=%d",
                   reason.c_str(), discovery_sec,
                   mmc_status_client_->service_is_ready() ? 1 : 0,
                   list_controllers_client_->service_is_ready() ? 1 : 0);
      return false;
    }
    // The graph can report a service ready before the new process's request
    // endpoint is matched; B_12's first fresh-process state read took 1.07 s.
    // Give the first complete observation its own bounded budget, then apply
    // the steady 100 ms / 250 ms policy to the continuous window.
    const auto establish_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < establish_deadline) {
      if (!status_query_.inFlight() && !list_query_.inFlight()) {
        const auto start_now = std::chrono::steady_clock::now();
        if (startHealthRound(start_now)) {
          health_round_started_ = start_now;
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
      }
      const auto round = pollHealthRound(
          std::chrono::steady_clock::now(), health_round_started_, kStartupHealthQueryBudget, reason);
      if (round == HealthRoundStatus::COMPLETE) {
        if (!fault_latch_.healthy()) return false;
        break;
      }
      if (round == HealthRoundStatus::ABANDONED) {
        next_health_check_ = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (fault_latch_.lastGoodNs() == 0) {
      reason = "no_complete_health_query_within_startup_budget";
      RCLCPP_ERROR(get_logger(),
                   "[MMC_HEALTH_PROBE] result=fail reason=%s discovery_sec=%.3f "
                   "queries=%llu misses=%llu",
                   reason.c_str(), discovery_sec,
                   static_cast<unsigned long long>(health_query_count_),
                   static_cast<unsigned long long>(health_miss_count_));
      return false;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(seconds));
    const auto queries_before = health_query_count_;
    const auto successes_before = health_success_count_;
    const auto misses_before = health_miss_count_;
    RCLCPP_INFO(get_logger(),
                "[MMC_HEALTH_PROBE] start duration_sec=%.3f discovery_sec=%.3f expected_mmc_active=0 "
                "query_interval_ms=%lld single_query_budget_ms=%lld freshness_budget_ms=%llu",
                seconds, discovery_sec,
                static_cast<long long>(kHealthQueryInterval.count()),
                static_cast<long long>(kSingleHealthQueryBudget.count()),
                static_cast<unsigned long long>(
                    dual_arm_carry_task::ControllerFaultLatch::kMaxStatusAgeNs / 1000000ULL));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (!pollControllerHealthNonBlocking(reason)) {
        RCLCPP_ERROR(get_logger(),
                     "[MMC_HEALTH_PROBE] result=fail reason=%s queries=%llu successes=%llu misses=%llu",
                     reason.c_str(),
                     static_cast<unsigned long long>(health_query_count_ - queries_before),
                     static_cast<unsigned long long>(health_success_count_ - successes_before),
                     static_cast<unsigned long long>(health_miss_count_ - misses_before));
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto queries = health_query_count_ - queries_before;
    const auto successes = health_success_count_ - successes_before;
    const auto misses = health_miss_count_ - misses_before;
    RCLCPP_INFO(get_logger(),
                "[MMC_HEALTH_PROBE] result=%s queries=%llu successes=%llu misses=%llu "
                "last_query_elapsed_ms=%.3f last_success_age_ms=%.3f",
                successes > 0 ? "pass" : "no_successful_query",
                static_cast<unsigned long long>(queries),
                static_cast<unsigned long long>(successes),
                static_cast<unsigned long long>(misses),
                last_health_query_elapsed_ms_, currentHealthAgeMs(steadyNowNs()));
    return successes > 0 && fault_latch_.healthy();
  }

  bool controllerStates(bool expect_mmc, std::chrono::steady_clock::time_point deadline)
  {
    const auto discovery = dual_arm_carry_task::waitForServiceUntil(
        deadline, [this]() { return list_controllers_client_->service_is_ready(); },
        [](auto next) { std::this_thread::sleep_until(next); });
    if (discovery != dual_arm_carry_task::ServiceDiscoveryResult::READY) {
      RCLCPP_ERROR(get_logger(), "[MMC_ACTIVE_VERIFY] result=service_discovery_timeout service=%s",
                   list_controllers_client_->get_service_name());
      return false;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      RCLCPP_ERROR(get_logger(), "[MMC_ACTIVE_VERIFY] result=request_timeout_before_send service=%s",
                   list_controllers_client_->get_service_name());
      return false;
    }
    auto future = list_controllers_client_->async_send_request(
        std::make_shared<ListControllers::Request>());
    if (future.wait_for(remaining) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[MMC_ACTIVE_VERIFY] result=request_response_timeout service=%s",
                   list_controllers_client_->get_service_name());
      return false;
    }
    ListControllers::Response::SharedPtr response;
    try {
      response = future.get();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "[MMC_ACTIVE_VERIFY] result=request_exception service=%s what=%s",
                   list_controllers_client_->get_service_name(), error.what());
      return false;
    }
    if (!response) {
      RCLCPP_ERROR(get_logger(), "[MMC_ACTIVE_VERIFY] result=empty_response service=%s",
                   list_controllers_client_->get_service_name());
      return false;
    }
    std::vector<std::pair<std::string, std::string>> states;
    states.reserve(response->controller.size());
    for (const auto& controller : response->controller)
      states.emplace_back(controller.name, controller.state);
    dual_arm_carry_task::ControllerStateObservation observed;
    const auto result = dual_arm_carry_task::evaluateControllerStates(
        states, jtc_controller_, mmc_controller_, expect_mmc, observed);
    if (result == dual_arm_carry_task::ControllerStateResult::MISSING_JTC ||
        result == dual_arm_carry_task::ControllerStateResult::MISSING_MMC) {
      const char* missing = result == dual_arm_carry_task::ControllerStateResult::MISSING_JTC ?
          jtc_controller_.c_str() : mmc_controller_.c_str();
      RCLCPP_ERROR(get_logger(), "[MMC_ACTIVE_VERIFY] result=controller_missing missing=%s "
                   "jtc_name=%s jtc_state=%s mmc_name=%s mmc_state=%s service=%s",
                   missing, jtc_controller_.c_str(),
                   observed.jtc_state.empty() ? "<missing>" : observed.jtc_state.c_str(),
                   mmc_controller_.c_str(), observed.mmc_state.empty() ? "<missing>" : observed.mmc_state.c_str(),
                   list_controllers_client_->get_service_name());
      return false;
    }
    const bool valid = result == dual_arm_carry_task::ControllerStateResult::MATCH;
    RCLCPP_INFO(get_logger(), "[MMC_ACTIVE_VERIFY] result=%s expected_mmc_active=%d "
                "jtc_name=%s jtc_state=%s mmc_name=%s mmc_state=%s service=%s",
                valid ? "match" : "unexpected_state", expect_mmc ? 1 : 0,
                jtc_controller_.c_str(), observed.jtc_state.c_str(), mmc_controller_.c_str(),
                observed.mmc_state.c_str(), list_controllers_client_->get_service_name());
    return valid;
  }

  bool traceSample(std::ofstream& trace, const std::array<ArmReference, 2>& expected,
                   double elapsed, double sim_elapsed,
                   double payload_ff_force_n, double payload_ff_scale)
  {
    std::array<ArmReference, 2> actual;
    if (!readArms(actual, std::chrono::milliseconds(200), "tracking-sample")) return false;
    trace << std::setprecision(10) << elapsed << ',' << sim_elapsed << ',' << now().seconds();
    for (int arm = 0; arm < 2; ++arm) {
      const auto write_xyz = [&trace](const Eigen::Vector3d& v) {
        trace << ',' << v.x() << ',' << v.y() << ',' << v.z();
      };
      const auto write_q = [&trace](const Eigen::Quaterniond& q) {
        trace << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
      };
      write_xyz(expected[arm].position);
      write_xyz(actual[arm].position);
      write_q(expected[arm].orientation);
      write_q(actual[arm].orientation);
      for (int j = 0; j < 7; ++j) trace << ',' << expected[arm].q[j];
      for (int j = 0; j < 7; ++j) trace << ',' << actual[arm].q[j];
    }
    // The hold-only bridge commands zero twist; write both arms' six twist
    // columns so the row matches the 12 twist names in the header.
    trace << ",0,0,0,0,0,0,0,0,0,0,0,0";
    trace << ',' << reference_publish_count_ << ',' << last_reference_interval_ms_ << ','
          << health_query_count_ << ',' << health_success_count_ << ','
          << health_miss_count_ << ',' << health_consecutive_miss_ << ','
          << health_recovery_count_ << ',' << last_health_query_elapsed_ms_ << ','
          << currentHealthAgeMs(steadyNowNs()) << ','
          << ((status_query_.inFlight() || list_query_.inFlight()) ? 1 : 0) << ','
          << payload_ff_force_n << ',' << payload_ff_force_n << ',' << payload_ff_scale << '\n';
    trace.flush();
    return static_cast<bool>(trace);
  }

  static std::uint64_t steadyNowNs()
  {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  bool fail(const std::string& what)
  {
    cancelRodForceRequest("bridge_failure");
    goal_pub_.reset();
    RCLCPP_ERROR(get_logger(), "[NO_LOAD] failed: %s", what.c_str());
    return false;
  }

  void cancelRodForceRequest(const char* reason)
  {
    if (!rod_force_arm_pub_) return;
    std_msgs::msg::Bool request;
    request.data = false;
    rod_force_arm_pub_->publish(request);
    RCLCPP_INFO(get_logger(), "[ROD_FORCE_REQUEST] event=cancel reason=%s", reason);
  }

  bool configureDirectionalParameters()
  {
    if (control_variant_ == "fixed") {
      RCLCPP_INFO(get_logger(), "[MMC_CONTROL_VARIANT] fixed defaults retained");
      return true;
    }
    if (control_variant_ != "directional") {
      RCLCPP_ERROR(get_logger(), "[MMC_CONTROL_VARIANT] unsupported variant=%s", control_variant_.c_str());
      return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (rclcpp::ok() && !impedance_parameters_client_->service_is_ready() &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (!impedance_parameters_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "[MMC_CONTROL_VARIANT] parameter service unavailable service=%s",
                   impedance_parameters_client_->get_service_name());
      return false;
    }
    auto request = std::make_shared<SetCartesianImpedance::Request>();
    // The service stores a column-major 6x6 matrix. Both link0 frames have
    // identity orientation, so local +x is the same world +X direction.
    request->stiffness.fill(0.0);
    request->stiffness[0] = 200.0;
    request->stiffness[7] = 400.0;
    request->stiffness[14] = 400.0;
    request->stiffness[21] = 20.0;
    request->stiffness[28] = 20.0;
    request->stiffness[35] = 20.0;
    request->damping_ratio.fill(0.8);
    request->nullspace_stiffness = 10.0;
    auto future = impedance_parameters_client_->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[MMC_CONTROL_VARIANT] parameter request timeout service=%s",
                   impedance_parameters_client_->get_service_name());
      return false;
    }
    future.get();
    RCLCPP_INFO(get_logger(), "[MMC_CONTROL_VARIANT] directional service=%s Kdiag=(200,400,400,20,20,20) "
                "zeta=0.8 nullspace=10 D=online_operational_inertia",
                impedance_parameters_client_->get_service_name());
    return true;
  }

  bool switchControllers(const std::vector<std::string>& activate,
                         const std::vector<std::string>& deactivate,
                         std::chrono::steady_clock::time_point deadline,
                         const char* stage)
  {
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
           !switch_client_->service_is_ready())
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!switch_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "[MMC_SWITCH] stage=%s result=service_not_discovered service=%s",
                   stage, switch_client_->get_service_name());
      return false;
    }
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    // STRICT: a switch that cannot reach exactly this controller set must
    // fail, so the trajectory controller and the impedance controller can
    // never end up active at the same time.
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) return false;
    request->timeout.sec = static_cast<std::int32_t>(remaining.count() / 1000000000LL);
    request->timeout.nanosec = static_cast<std::uint32_t>(remaining.count() % 1000000000LL);
    RCLCPP_INFO(get_logger(), "[MMC_SWITCH] stage=%s request=STRICT activate=%s deactivate=%s "
                "service=%s remaining_budget_ms=%lld",
                stage, activate.empty()?"-":activate.front().c_str(),
                deactivate.empty()?"-":deactivate.front().c_str(), switch_client_->get_service_name(),
                static_cast<long long>(remaining.count()/1000000LL));
    auto future = switch_client_->async_send_request(request);
    const auto wait_left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (wait_left.count() <= 0 || future.wait_for(wait_left) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[MMC_SWITCH] stage=%s result=request_timeout", stage);
      return false;
    }
    bool ok = false;
    try {
      const auto response = future.get();
      ok = response && response->ok;
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "[MMC_SWITCH] stage=%s result=request_exception service=%s what=%s",
                   stage, switch_client_->get_service_name(), error.what());
      return false;
    }
    RCLCPP_INFO(get_logger(), "[MMC_SWITCH] stage=%s result=%s activate=%s deactivate=%s ok=%d",
                stage, ok?"response_received":"response_rejected",
                activate.empty() ? "-" : activate.front().c_str(),
                deactivate.empty() ? "-" : deactivate.front().c_str(), ok ? 1 : 0);
    return ok;
  }

  bool readArms(std::array<ArmReference, 2>& out, std::chrono::duration<double> budget,
                const char* stage)
  {
    if (!std::isfinite(budget.count()) || budget.count() <= 0.0) {
      last_state_read_error_ = std::string("state read invalid_budget service=") +
          robot_states_client_->get_service_name() + " stage=" + stage;
      RCLCPP_ERROR(get_logger(), "[MMC_STATE_READ] result=failed reason=invalid_budget service=%s stage=%s",
                   robot_states_client_->get_service_name(), stage);
      return false;
    }
    const auto began = std::chrono::steady_clock::now();
    const auto deadline = began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(budget);
    auto remaining = [&]() {
      return std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    };
    const std::string service = robot_states_client_->get_service_name();
    auto fail_read = [&](const char* reason) {
      last_state_read_elapsed_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      last_state_read_error_ = std::string("state read ") + reason + " service=" + service +
          " stage=" + stage + " elapsed_wall_sec=" + std::to_string(last_state_read_elapsed_);
      RCLCPP_ERROR(get_logger(), "[MMC_STATE_READ] result=failed reason=%s service=%s stage=%s elapsed_wall_sec=%.6f",
                   reason, service.c_str(), stage, last_state_read_elapsed_);
      return false;
    };
    while (rclcpp::ok() && remaining().count() > 0 &&
           !robot_states_client_->wait_for_service(std::chrono::milliseconds(0)))
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!robot_states_client_->service_is_ready()) return fail_read("service_not_discovered");
    auto future = robot_states_client_->async_send_request(
        std::make_shared<GetRobotStates::Request>());
    const auto left = remaining();
    if (left.count() <= 0 || future.wait_for(left) != std::future_status::ready)
      return fail_read("request_timeout");
    GetRobotStates::Response::SharedPtr response;
    try {
      response = future.get();
    } catch (const std::exception& error) {
      return fail_read((std::string("service_response_failed: ") + error.what()).c_str());
    }
    if (!response) return fail_read("service_response_failed");
    sensor_msgs::msg::JointState joint_sample;
    std::chrono::steady_clock::time_point joint_received;
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      joint_sample = latest_joints_;
      joint_received = latest_joints_wall_;
    }
    if (joint_received == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - joint_received > std::chrono::milliseconds(500))
      return fail_read("stale_joint_state_source");
    // The service response is the authoritative synchronized pair for this
    // read. Its FrankaState.time values are generated by the same task-side
    // feedback service; comparing them against /joint_states or this node's
    // independently advancing /clock can reject a fresh response at the
    // controller-switch boundary. The response header wall age above still
    // enforces freshness, while the validator checks the fields and frames.
    if (response->states.size() != 2 ||
        !std::isfinite(response->states[0].time) ||
        !std::isfinite(response->states[1].time))
      return fail_read("invalid_response_time");
    const double response_sim_time = response->states[0].time;
    const auto validation=dual_arm_carry_task::validateMmcRobotStates(
        response->states, response_sim_time, dual_arm_carry_task::systemWallNowNs());
    if (!validation.valid) return fail_read(validation.reason.c_str());
    for (int i = 0; i < 2; ++i) {
      const auto& state = response->states[static_cast<std::size_t>(i)];
      Eigen::Matrix4d transform;
      bool row_major = false;
      if (!dual_arm_carry_task::decodeMmcTransform(state.o_t_ee, transform, row_major))
        return fail_read("invalid_transform_or_quaternion");
      (void)row_major;
      const Eigen::Quaterniond raw(transform.block<3, 3>(0, 0));
      out[static_cast<std::size_t>(i)].position = transform.block<3, 1>(0, 3);
      out[static_cast<std::size_t>(i)].orientation = raw.normalized();
      for (int j = 0; j < 7; ++j) out[static_cast<std::size_t>(i)].q[j] = state.q[static_cast<std::size_t>(j)];
      if (!out[static_cast<std::size_t>(i)].position.allFinite() ||
          !out[static_cast<std::size_t>(i)].orientation.coeffs().allFinite() ||
          !out[static_cast<std::size_t>(i)].q.allFinite()) return fail_read("invalid_response_fields");
    }
    last_state_read_elapsed_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    RCLCPP_INFO(get_logger(), "[MMC_STATE_READ] result=valid service=%s stage=%s "
                "source=task_joint_feedback_MoveIt_FK "
                "order=mj_left,mj_right frames=(%s,%s) sample_time=(%.6f,%.6f) joint_time=%.6f "
                "wall_age_max_ms<=500 sample_lag_max_s<=0.5 elapsed_wall_sec=%.6f",
                service.c_str(), stage, response->states[0].header.frame_id.c_str(),
                response->states[1].header.frame_id.c_str(), response->states[0].time,
                response->states[1].time, response_sim_time, last_state_read_elapsed_);
    return true;
  }

  void publishGoal(const std::array<ArmReference, 2>& start, double offset, double rate,
                   double payload_ff_force_n)
  {
    GoalMsg msg;
    msg.l_pose = poseOf(start[0], offset);
    msg.r_pose = poseOf(start[1], offset);
    msg.l_twist.linear.z = rate;
    msg.r_twist.linear.z = rate;
    // Force in each arm's link0 (zero-Jacobian) frame. Both bases have the
    // world orientation in this scene (MJCF quat=(1,0,0,0)); see run().
    const Eigen::Vector3d base_force = payload_ff::worldToBaseForce(
        Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, payload_ff_force_n));
    msg.l_ff_force.x = base_force.x();
    msg.l_ff_force.y = base_force.y();
    msg.l_ff_force.z = base_force.z();
    msg.r_ff_force.x = base_force.x();
    msg.r_ff_force.y = base_force.y();
    msg.r_ff_force.z = base_force.z();
    for (int j = 0; j < 7; ++j) {
      msg.l_q_n[static_cast<std::size_t>(j)] = start[0].q[j];
      msg.r_q_n[static_cast<std::size_t>(j)] = start[1].q[j];
    }
    const auto now = std::chrono::steady_clock::now();
    if (last_reference_wall_ != std::chrono::steady_clock::time_point{}) {
      last_reference_interval_ms_ =
          std::chrono::duration<double, std::milli>(now - last_reference_wall_).count();
    }
    last_reference_wall_ = now;
    ++reference_publish_count_;
    goal_pub_->publish(msg);
  }

  // Max over both arms of |position error| and rotation error against the
  // commanded reference. Returns false if the state service is unavailable.
  bool measureTracking(double offset, double& position_error, double& rotation_error)
  {
    auto future = robot_states_client_->async_send_request(
        std::make_shared<GetRobotStates::Request>());
    if (future.wait_for(std::chrono::milliseconds(200)) != std::future_status::ready) return false;
    const auto response = future.get();
    if (response->states.size() < 2) return false;
    position_error = 0.0;
    rotation_error = 0.0;
    for (int i = 0; i < 2; ++i) {
      const auto& state = response->states[static_cast<std::size_t>(i)];
      const Eigen::Matrix4d transform = Eigen::Matrix4d::Map(state.o_t_ee.data());
      const Eigen::Vector3d position = transform.block<3, 1>(0, 3);
      const Eigen::Quaterniond orientation =
          Eigen::Quaterniond(transform.block<3, 3>(0, 0)).normalized();
      Eigen::Vector3d target = reference_start_[static_cast<std::size_t>(i)].position;
      target.z() += offset;
      position_error = std::max(position_error, (target - position).norm());
      rotation_error = std::max(rotation_error,
          orientation.angularDistance(reference_start_[static_cast<std::size_t>(i)].orientation));
    }
    return true;
  }

  geometry_msgs::msg::Pose poseOf(const ArmReference& arm, double offset) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = arm.position.x();
    pose.position.y = arm.position.y();
    pose.position.z = arm.position.z() + offset;
    pose.orientation.x = arm.orientation.x();
    pose.orientation.y = arm.orientation.y();
    pose.orientation.z = arm.orientation.z();
    pose.orientation.w = arm.orientation.w();
    return pose;
  }

  bool seedJtcFromCurrentState()
  {
    // The trajectory controller must start from where the arm actually is, not
    // from whatever it last commanded before the switch.
    sensor_msgs::msg::JointState joint_snapshot;
    {
      std::lock_guard<std::mutex> lock(joints_mutex_);
      if (latest_joints_.name.empty() ||
          std::chrono::steady_clock::now() - latest_joints_wall_ > std::chrono::milliseconds(250)) return false;
      joint_snapshot = latest_joints_;
    }
    if (!jtc_client_->wait_for_action_server(std::chrono::seconds(10))) return false;
    std::map<std::string, double> positions;
    for (std::size_t i = 0; i < joint_snapshot.name.size() && i < joint_snapshot.position.size(); ++i)
      positions[joint_snapshot.name[i]] = joint_snapshot.position[i];

    const std::vector<std::string> joints{
        "mj_right_joint1", "mj_right_joint2", "mj_right_joint3", "mj_right_joint4",
        "mj_right_joint5", "mj_right_joint6", "mj_right_joint7", "mj_left_joint1",
        "mj_left_joint2",  "mj_left_joint3",  "mj_left_joint4",  "mj_left_joint5",
        "mj_left_joint6",  "mj_left_joint7"};
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joints;
    // A zero stamp means "start immediately" for JointTrajectoryController.
    // Do not use this node's clock here: this bridge may use system time while
    // the simulation controller manager uses ROS/sim time, and rclcpp rejects
    // comparisons between those two time sources.
    goal.trajectory.header.stamp = builtin_interfaces::msg::Time{};
    for (int k = 0; k < 2; ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      for (const auto& name : joints) {
        const auto it = positions.find(name);
        if (it == positions.end()) {
          RCLCPP_ERROR(get_logger(), "[NO_LOAD] joint %s missing from /joint_states", name.c_str());
          return false;
        }
        point.positions.push_back(it->second);
        point.velocities.push_back(0.0);
      }
      point.time_from_start = rclcpp::Duration::from_seconds(k == 0 ? 0.0 : 0.5);
      goal.trajectory.points.push_back(point);
    }
    auto future = jtc_client_->async_send_goal(goal);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;
    const auto handle = future.get();
    if (!handle) return false;
    auto result_future = jtc_client_->async_get_result(handle);
    if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[NO_LOAD] seeded JTC trajectory result timed out");
      return false;
    }
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "[NO_LOAD] seeded JTC trajectory did not succeed (code=%d)",
                   static_cast<int>(result.code));
      return false;
    }
    RCLCPP_INFO(get_logger(), "[NO_LOAD] trajectory controller seeded from the measured joint state");
    return true;
  }

  double hold_seconds_{3.0};
  double final_hold_seconds_{3.0};
  double move_amplitude_{0.005};
  double move_seconds_{1.0};
  double rate_hz_{100.0};
  double stop_reference_after_seconds_{-1.0};
  double wall_timeout_seconds_{60.0};
  std::string goal_topic_;
  std::string robot_states_service_;
  std::string jtc_action_;
  std::string jtc_controller_;
  std::string mmc_controller_;
  std::string mmc_status_service_;
  bool hold_only_{false};
  std::string trace_csv_;
  std::string control_variant_;
  bool rod_disturbance_enabled_{false};
  bool payload_feedforward_enabled_{false};
  double payload_mass_kg_{0.64};
  double feedforward_ramp_seconds_{0.5};
  bool preflight_only_{false};
  double state_read_budget_seconds_{3.0};
  std::string start_gate_file_;
  double start_gate_timeout_seconds_{60.0};
  std::string last_state_read_error_{"state_read_failed"};
  double last_state_read_elapsed_{0.0};
  std::uint64_t last_reference_sequence_{0};
  std::uint64_t first_reference_publish_count_{0};
  std::chrono::steady_clock::time_point activation_started_{};
  std::chrono::steady_clock::time_point first_reference_publish_wall_{};
  bool goal_subscriber_seen_{false};
  bool reference_baseline_logged_{false};

  rclcpp::Publisher<GoalMsg>::SharedPtr goal_pub_;
  rclcpp::Client<GetRobotStates>::SharedPtr robot_states_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<GetControllers>::SharedPtr mmc_status_client_;
  rclcpp::Client<SetCartesianImpedance>::SharedPtr impedance_parameters_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rod_force_arm_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr jtc_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::mutex joints_mutex_;
  sensor_msgs::msg::JointState latest_joints_;
  std::chrono::steady_clock::time_point latest_joints_wall_{};
  std::array<ArmReference, 2> reference_start_;
  double max_tracking_position_{0.0};
  double max_tracking_rotation_{0.0};
  double max_final_hold_position_{0.0};
  dual_arm_carry_task::ControllerFaultLatch fault_latch_;
  std::chrono::steady_clock::time_point next_health_check_{std::chrono::steady_clock::now()};
  StatusQuery status_query_;
  ListQuery list_query_;
  bool health_expect_mmc_active_{true};
  bool health_round_expect_mmc_active_{false};
  std::chrono::steady_clock::time_point health_round_started_{};
  std::chrono::steady_clock::time_point health_probe_started_{std::chrono::steady_clock::now()};
  std::uint64_t health_query_count_{0};
  std::uint64_t health_success_count_{0};
  std::uint64_t health_miss_count_{0};
  std::uint64_t health_consecutive_miss_{0};
  std::uint64_t health_recovery_count_{0};
  std::uint64_t health_freshness_budget_ns_{
      dual_arm_carry_task::ControllerFaultLatch::kMaxStatusAgeNs};
  double last_health_query_elapsed_ms_{-1.0};
  const char* last_health_miss_category_{"none"};
  std::uint64_t reference_publish_count_{0};
  double last_reference_interval_ms_{-1.0};
  std::chrono::steady_clock::time_point last_reference_wall_{};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualImpedanceNoLoadBridge>();
  // The sequence waits on service responses, action results and a subscription,
  // so the node must actually be spun while run() blocks.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  const bool ok = node->run();
  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
