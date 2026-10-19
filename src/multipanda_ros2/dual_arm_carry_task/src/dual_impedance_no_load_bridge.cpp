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
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <multi_mode_control_msgs/msg/dual_cartesian_impedance_goal.hpp>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include <memory>
#include <string>
#include <vector>

namespace
{
using GoalMsg = multi_mode_control_msgs::msg::DualCartesianImpedanceGoal;
using GetRobotStates = multi_mode_control_msgs::srv::GetRobotStates;
using SwitchController = controller_manager_msgs::srv::SwitchController;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

struct ArmReference
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 7, 1> q{Eigen::Matrix<double, 7, 1>::Zero()};
};

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
    wall_timeout_seconds_ = declare_parameter("wall_timeout_seconds", 60.0);

    goal_pub_ = create_publisher<GoalMsg>(goal_topic_, rclcpp::QoS(10));
    robot_states_client_ = create_client<GetRobotStates>(robot_states_service_);
    switch_client_ = create_client<SwitchController>("/controller_manager/switch_controller");
    jtc_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, jtc_action_);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { latest_joints_ = *msg; });

    RCLCPP_INFO(get_logger(),
                "[NO_LOAD] hold=%.1fs move=%.1fmm/%.1fs holds=%.1fs goal_topic=%s",
                hold_seconds_, move_amplitude_ * 1000.0, move_seconds_, final_hold_seconds_,
                goal_topic_.c_str());
    if (stop_reference_after_seconds_ >= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "[NO_LOAD] watchdog debug enabled: stop publishing reference at %.3f s",
                  stop_reference_after_seconds_);
    }
  }

  bool run()
  {
    if (!switchControllers({mmc_controller_}, {jtc_controller_})) return fail("switch to impedance");
    RCLCPP_INFO(get_logger(), "[NO_LOAD] impedance controller requested");

    if (!waitForGoalSubscriber()) return fail("impedance goal subscriber never appeared");

    std::array<ArmReference, 2> start;
    if (!readArms(start)) return fail("read initial arm state");
    reference_start_ = start;
    RCLCPP_INFO(get_logger(), "[NO_LOAD] initial left=(%.4f,%.4f,%.4f) right=(%.4f,%.4f,%.4f)",
                start[0].position.x(), start[0].position.y(), start[0].position.z(),
                start[1].position.x(), start[1].position.y(), start[1].position.z());

    const auto began = std::chrono::steady_clock::now();
    int cycle = 0;
    auto elapsed = [&began]() {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    };
    const double move_begin = hold_seconds_;
    const double move_end = move_begin + move_seconds_;
    const double sequence_end = move_end + final_hold_seconds_;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));

    while (rclcpp::ok() && elapsed() < sequence_end) {
      const double t = elapsed();
      double offset = 0.0;
      double rate = 0.0;
      if (t >= move_begin && t < move_end) {
        const double u = (t - move_begin) / move_seconds_;
        offset = move_amplitude_ * quinticRamp(u);
        rate = move_amplitude_ * quinticRampRate(u, move_seconds_);
      } else if (t >= move_end) {
        offset = move_amplitude_;
      }
      const bool publish_reference = stop_reference_after_seconds_ < 0.0 ||
          t < stop_reference_after_seconds_;
      if (publish_reference) {
        publishGoal(start, offset, rate);
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
    // Keep the final pose commanded while the switch back is prepared.
    if (stop_reference_after_seconds_ < 0.0) {
      for (int i = 0; i < 5; ++i) {
        publishGoal(start, move_amplitude_, 0.0);
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

    if (!switchControllers({jtc_controller_}, {mmc_controller_})) return fail("switch back");
    if (!seedJtcFromCurrentState()) return fail("seed trajectory controller from current state");
    RCLCPP_INFO(get_logger(), "[NO_LOAD] returned to the trajectory controller");
    return true;
  }

private:
  bool fail(const std::string& what)
  {
    RCLCPP_ERROR(get_logger(), "[NO_LOAD] failed: %s", what.c_str());
    return false;
  }

  bool switchControllers(const std::vector<std::string>& activate,
                         const std::vector<std::string>& deactivate)
  {
    if (!switch_client_->wait_for_service(std::chrono::seconds(10))) return false;
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    // STRICT: a switch that cannot reach exactly this controller set must
    // fail, so the trajectory controller and the impedance controller can
    // never end up active at the same time.
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) return false;
    const bool ok = future.get()->ok;
    RCLCPP_INFO(get_logger(), "[NO_LOAD] switch activate=%s deactivate=%s ok=%d",
                activate.empty() ? "-" : activate.front().c_str(),
                deactivate.empty() ? "-" : deactivate.front().c_str(), ok ? 1 : 0);
    return ok;
  }

  bool waitForGoalSubscriber()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (goal_pub_->get_subscription_count() > 0) return true;
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  bool readArms(std::array<ArmReference, 2>& out)
  {
    if (!robot_states_client_->wait_for_service(std::chrono::seconds(10))) return false;
    auto future = robot_states_client_->async_send_request(
        std::make_shared<GetRobotStates::Request>());
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) return false;
    const auto response = future.get();
    if (response->states.size() < 2) return false;
    for (int i = 0; i < 2; ++i) {
      const auto& state = response->states[static_cast<std::size_t>(i)];
      const Eigen::Matrix4d transform = Eigen::Matrix4d::Map(state.o_t_ee.data());
      out[static_cast<std::size_t>(i)].position = transform.block<3, 1>(0, 3);
      out[static_cast<std::size_t>(i)].orientation =
          Eigen::Quaterniond(transform.block<3, 3>(0, 0)).normalized();
      for (int j = 0; j < 7; ++j) out[static_cast<std::size_t>(i)].q[j] = state.q[static_cast<std::size_t>(j)];
    }
    return true;
  }

  void publishGoal(const std::array<ArmReference, 2>& start, double offset, double rate)
  {
    GoalMsg msg;
    msg.l_pose = poseOf(start[0], offset);
    msg.r_pose = poseOf(start[1], offset);
    msg.l_twist.linear.z = rate;
    msg.r_twist.linear.z = rate;
    for (int j = 0; j < 7; ++j) {
      msg.l_q_n[static_cast<std::size_t>(j)] = start[0].q[j];
      msg.r_q_n[static_cast<std::size_t>(j)] = start[1].q[j];
    }
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
    if (latest_joints_.name.empty()) return false;
    if (!jtc_client_->wait_for_action_server(std::chrono::seconds(10))) return false;
    std::map<std::string, double> positions;
    for (std::size_t i = 0; i < latest_joints_.name.size() && i < latest_joints_.position.size(); ++i)
      positions[latest_joints_.name[i]] = latest_joints_.position[i];

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

  rclcpp::Publisher<GoalMsg>::SharedPtr goal_pub_;
  rclcpp::Client<GetRobotStates>::SharedPtr robot_states_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr jtc_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  sensor_msgs::msg::JointState latest_joints_;
  std::array<ArmReference, 2> reference_start_;
  double max_tracking_position_{0.0};
  double max_tracking_rotation_{0.0};
  double max_final_hold_position_{0.0};
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
