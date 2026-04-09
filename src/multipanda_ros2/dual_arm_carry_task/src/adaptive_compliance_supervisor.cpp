#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <multi_mode_control_msgs/srv/set_cartesian_impedance.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

class AdaptiveComplianceSupervisor : public rclcpp::Node
{
public:
  AdaptiveComplianceSupervisor()
  : Node("adaptive_compliance_supervisor")
  {
    this->declare_parameter<std::string>(
      "left_wrench_topic", "/force_torque_sensor_broadcaster_left/wrench");
    this->declare_parameter<std::string>(
      "right_wrench_topic", "/force_torque_sensor_broadcaster_right/wrench");
    this->declare_parameter<std::string>("task_stage_topic", "/task_stage");
    this->declare_parameter<std::string>("task_status_topic", "/task_status");
    this->declare_parameter<std::string>(
      "impedance_service", "/left_and_right/dual_cartesian_impedance_controller/parameters");
    this->declare_parameter<std::string>("adaptive_params_topic", "/adaptive_impedance_params");
    this->declare_parameter<std::vector<std::string>>(
      "active_stages", std::vector<std::string>{"TRANSPORT", "ROTATE", "DESCEND"});

    this->declare_parameter<double>("update_rate_hz", 20.0);
    this->declare_parameter<double>("force_lpf_alpha", 0.25);
    this->declare_parameter<double>("force_deadband_n", 1.0);

    this->declare_parameter<double>("nominal_translation_stiffness", 400.0);
    this->declare_parameter<double>("min_translation_stiffness", 150.0);
    this->declare_parameter<double>("max_translation_stiffness", 700.0);
    this->declare_parameter<double>("rotation_stiffness", 20.0);
    this->declare_parameter<double>("nominal_damping_ratio", 0.8);
    this->declare_parameter<double>("min_damping_ratio", 0.6);
    this->declare_parameter<double>("max_damping_ratio", 1.2);
    this->declare_parameter<double>("nullspace_stiffness", 10.0);

    this->declare_parameter<double>("force_gain", 2.0);
    this->declare_parameter<double>("imbalance_gain", 8.0);
    this->declare_parameter<double>("stiffness_update_epsilon", 5.0);
    this->declare_parameter<double>("damping_update_epsilon", 0.02);
    this->declare_parameter<double>("min_service_interval_sec", 0.20);
    this->declare_parameter<double>("max_refresh_interval_sec", 1.0);
    this->declare_parameter<double>("service_startup_wait_sec", 2.0);
    this->declare_parameter<double>("service_recheck_interval_sec", 2.0);

    this->declare_parameter<bool>("stop_on_done", true);

    left_wrench_topic_ = this->get_parameter("left_wrench_topic").as_string();
    right_wrench_topic_ = this->get_parameter("right_wrench_topic").as_string();
    task_stage_topic_ = this->get_parameter("task_stage_topic").as_string();
    task_status_topic_ = this->get_parameter("task_status_topic").as_string();
    impedance_service_ = this->get_parameter("impedance_service").as_string();
    adaptive_params_topic_ = this->get_parameter("adaptive_params_topic").as_string();

    update_rate_hz_ = this->get_parameter("update_rate_hz").as_double();
    force_lpf_alpha_ = this->get_parameter("force_lpf_alpha").as_double();
    force_deadband_n_ = this->get_parameter("force_deadband_n").as_double();

    nominal_translation_stiffness_ =
      this->get_parameter("nominal_translation_stiffness").as_double();
    min_translation_stiffness_ = this->get_parameter("min_translation_stiffness").as_double();
    max_translation_stiffness_ = this->get_parameter("max_translation_stiffness").as_double();
    rotation_stiffness_ = this->get_parameter("rotation_stiffness").as_double();
    nominal_damping_ratio_ = this->get_parameter("nominal_damping_ratio").as_double();
    min_damping_ratio_ = this->get_parameter("min_damping_ratio").as_double();
    max_damping_ratio_ = this->get_parameter("max_damping_ratio").as_double();
    nullspace_stiffness_ = this->get_parameter("nullspace_stiffness").as_double();

    force_gain_ = this->get_parameter("force_gain").as_double();
    imbalance_gain_ = this->get_parameter("imbalance_gain").as_double();
    stiffness_update_epsilon_ = this->get_parameter("stiffness_update_epsilon").as_double();
    damping_update_epsilon_ = this->get_parameter("damping_update_epsilon").as_double();
    min_service_interval_sec_ = this->get_parameter("min_service_interval_sec").as_double();
    max_refresh_interval_sec_ = this->get_parameter("max_refresh_interval_sec").as_double();
    service_startup_wait_sec_ = this->get_parameter("service_startup_wait_sec").as_double();
    service_recheck_interval_sec_ = this->get_parameter("service_recheck_interval_sec").as_double();
    stop_on_done_ = this->get_parameter("stop_on_done").as_bool();

    if (update_rate_hz_ <= 0.0) {
      update_rate_hz_ = 20.0;
    }
    force_lpf_alpha_ = std::clamp(force_lpf_alpha_, 0.01, 1.0);
    force_deadband_n_ = std::max(0.0, force_deadband_n_);
    nominal_translation_stiffness_ = std::max(1.0, nominal_translation_stiffness_);
    min_translation_stiffness_ = std::max(1.0, min_translation_stiffness_);
    max_translation_stiffness_ = std::max(min_translation_stiffness_, max_translation_stiffness_);
    nominal_translation_stiffness_ = std::clamp(
      nominal_translation_stiffness_, min_translation_stiffness_, max_translation_stiffness_);
    nominal_damping_ratio_ = std::max(0.1, nominal_damping_ratio_);
    min_damping_ratio_ = std::max(0.1, min_damping_ratio_);
    max_damping_ratio_ = std::max(min_damping_ratio_, max_damping_ratio_);
    nominal_damping_ratio_ =
      std::clamp(nominal_damping_ratio_, min_damping_ratio_, max_damping_ratio_);
    min_service_interval_sec_ = std::max(0.01, min_service_interval_sec_);
    max_refresh_interval_sec_ = std::max(min_service_interval_sec_, max_refresh_interval_sec_);
    service_startup_wait_sec_ = std::max(0.0, service_startup_wait_sec_);
    service_recheck_interval_sec_ = std::max(0.2, service_recheck_interval_sec_);

    // NOTE:
    // get_parameter(...).as_string_array() returns a reference to data owned by
    // a temporary Parameter object. Iterating it directly can dangle and crash.
    const auto active_stages_param = this->get_parameter("active_stages").as_string_array();
    for (auto stage : active_stages_param) {
      std::transform(stage.begin(), stage.end(), stage.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
      });
      if (!stage.empty()) {
        active_stages_.insert(stage);
      }
    }

    left_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      left_wrench_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AdaptiveComplianceSupervisor::on_left_wrench, this, std::placeholders::_1));

    right_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      right_wrench_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AdaptiveComplianceSupervisor::on_right_wrench, this, std::placeholders::_1));

    stage_sub_ = this->create_subscription<std_msgs::msg::String>(
      task_stage_topic_, 20,
      std::bind(&AdaptiveComplianceSupervisor::on_stage, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      task_status_topic_, 10,
      std::bind(&AdaptiveComplianceSupervisor::on_task_status, this, std::placeholders::_1));

    adaptive_params_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      adaptive_params_topic_, 10);

    impedance_client_ = this->create_client<SetCartesianImpedance>(impedance_service_);

    const auto startup_wait_ms = static_cast<int>(std::round(service_startup_wait_sec_ * 1000.0));
    if (!impedance_client_->wait_for_service(std::chrono::milliseconds(std::max(startup_wait_ms, 0)))) {
      enter_degraded_mode("service_unavailable_at_startup");
    }

    last_command_time_ = this->now();
    last_sent_stiffness_ = nominal_translation_stiffness_;
    last_sent_damping_ = nominal_damping_ratio_;

    const auto period_ms = static_cast<int>(std::round(1000.0 / update_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(period_ms, 1)),
      std::bind(&AdaptiveComplianceSupervisor::on_control_tick, this));

    RCLCPP_INFO(this->get_logger(), "Adaptive compliance supervisor started.");
    RCLCPP_INFO(this->get_logger(), "  wrench topics: left=%s right=%s",
                left_wrench_topic_.c_str(), right_wrench_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  stage/status: %s | %s",
                task_stage_topic_.c_str(), task_status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  impedance service: %s", impedance_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "  adaptive params topic: %s", adaptive_params_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  translation stiffness range: [%.1f, %.1f], nominal=%.1f",
                min_translation_stiffness_, max_translation_stiffness_,
                nominal_translation_stiffness_);
  }

private:
  using SetCartesianImpedance = multi_mode_control_msgs::srv::SetCartesianImpedance;

  static std::string to_upper(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });
    return value;
  }

  static double apply_deadband(double value, double deadband)
  {
    return (std::abs(value) < deadband) ? 0.0 : value;
  }

  void on_left_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    const double force_z = apply_deadband(msg->wrench.force.z, force_deadband_n_);
    if (!have_left_) {
      filtered_left_force_z_ = force_z;
      have_left_ = true;
      return;
    }
    filtered_left_force_z_ =
      force_lpf_alpha_ * force_z + (1.0 - force_lpf_alpha_) * filtered_left_force_z_;
  }

  void on_right_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    const double force_z = apply_deadband(msg->wrench.force.z, force_deadband_n_);
    if (!have_right_) {
      filtered_right_force_z_ = force_z;
      have_right_ = true;
      return;
    }
    filtered_right_force_z_ =
      force_lpf_alpha_ * force_z + (1.0 - force_lpf_alpha_) * filtered_right_force_z_;
  }

  void on_stage(const std_msgs::msg::String::SharedPtr msg)
  {
    current_stage_ = to_upper(msg->data);
  }

  void on_task_status(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string status = to_upper(msg->data);
    if (status == "DONE" || status == "ERROR") {
      task_finished_ = true;
      if (stop_on_done_) {
        maybe_send_nominal(true);
      }
    }
  }

  bool is_active_stage() const
  {
    if (task_finished_ && stop_on_done_) {
      return false;
    }
    if (active_stages_.empty()) {
      return true;
    }
    return active_stages_.find(current_stage_) != active_stages_.end();
  }

  void on_control_tick()
  {
    if (!have_left_ || !have_right_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for both wrench streams before enabling adaptive loop.");
      return;
    }

    if (degraded_due_to_service_) {
      try_recover_from_degraded_mode();
      publish_adaptive_status(nominal_translation_stiffness_, nominal_damping_ratio_, false);
      return;
    }

    if (!is_active_stage()) {
      maybe_send_nominal(false);
      publish_adaptive_status(nominal_translation_stiffness_, nominal_damping_ratio_, false);
      return;
    }

    const double imbalance = filtered_left_force_z_ - filtered_right_force_z_;
    const double avg_abs_force =
      0.5 * (std::abs(filtered_left_force_z_) + std::abs(filtered_right_force_z_));

    const double reduction = force_gain_ * avg_abs_force + imbalance_gain_ * std::abs(imbalance);
    const double target_stiffness = std::clamp(
      nominal_translation_stiffness_ - reduction,
      min_translation_stiffness_,
      max_translation_stiffness_);

    const double stiffness_span = std::max(1.0, nominal_translation_stiffness_ - min_translation_stiffness_);
    const double normalized_softening =
      (nominal_translation_stiffness_ - target_stiffness) / stiffness_span;
    const double target_damping = std::clamp(
      nominal_damping_ratio_ + 0.25 * normalized_softening,
      min_damping_ratio_,
      max_damping_ratio_);

    publish_adaptive_status(target_stiffness, target_damping, true);

    const rclcpp::Time now = this->now();
    const double since_last = (now - last_command_time_).seconds();
    const bool changed =
      std::abs(target_stiffness - last_sent_stiffness_) >= stiffness_update_epsilon_ ||
      std::abs(target_damping - last_sent_damping_) >= damping_update_epsilon_;
    const bool refresh_due = since_last >= max_refresh_interval_sec_;
    const bool min_interval_ok = since_last >= min_service_interval_sec_;

    if ((changed || refresh_due) && min_interval_ok) {
      send_impedance_command(target_stiffness, target_damping);
    }
  }

  void maybe_send_nominal(bool force_send)
  {
    const rclcpp::Time now = this->now();
    const double since_last = (now - last_command_time_).seconds();
    if (!force_send && !adaptive_applied_) {
      return;
    }
    if (!force_send && since_last < min_service_interval_sec_) {
      return;
    }
    if (std::abs(last_sent_stiffness_ - nominal_translation_stiffness_) < stiffness_update_epsilon_ &&
        std::abs(last_sent_damping_ - nominal_damping_ratio_) < damping_update_epsilon_ &&
        !force_send) {
      adaptive_applied_ = false;
      return;
    }

    send_impedance_command(nominal_translation_stiffness_, nominal_damping_ratio_);
    adaptive_applied_ = false;
  }

  void send_impedance_command(double translation_stiffness, double damping_ratio)
  {
    if (request_in_flight_) {
      return;
    }
    if (!impedance_client_->service_is_ready()) {
      enter_degraded_mode("service_became_unavailable");
      return;
    }

    auto request = std::make_shared<SetCartesianImpedance::Request>();
    request->stiffness.fill(0.0);
    request->damping_ratio.fill(damping_ratio);

    for (size_t i = 0; i < 3; ++i) {
      request->stiffness[i * 7] = translation_stiffness;
    }
    for (size_t i = 3; i < 6; ++i) {
      request->stiffness[i * 7] = rotation_stiffness_;
    }
    request->nullspace_stiffness = nullspace_stiffness_;

    request_in_flight_ = true;
    impedance_client_->async_send_request(
      request,
      [this, translation_stiffness, damping_ratio](rclcpp::Client<SetCartesianImpedance>::SharedFuture future) {
        request_in_flight_ = false;
        (void)future;
        last_sent_stiffness_ = translation_stiffness;
        last_sent_damping_ = damping_ratio;
        last_command_time_ = this->now();
        adaptive_applied_ =
          (std::abs(last_sent_stiffness_ - nominal_translation_stiffness_) >= stiffness_update_epsilon_) ||
          (std::abs(last_sent_damping_ - nominal_damping_ratio_) >= damping_update_epsilon_);
      });
  }

  void enter_degraded_mode(const char * reason)
  {
    if (degraded_due_to_service_) {
      return;
    }
    degraded_due_to_service_ = true;
    request_in_flight_ = false;
    adaptive_applied_ = false;
    degraded_reason_ = reason ? reason : "unknown";
    last_service_recheck_time_ = this->now();

    RCLCPP_ERROR(
      this->get_logger(),
      "Adaptive impedance degraded (%s): service unavailable [%s]. "
      "Supervisor enters passive mode and will periodically probe for recovery.",
      degraded_reason_.c_str(), impedance_service_.c_str());
  }

  void try_recover_from_degraded_mode()
  {
    const rclcpp::Time now = this->now();
    const double since_last_probe = (now - last_service_recheck_time_).seconds();
    if (since_last_probe < service_recheck_interval_sec_) {
      return;
    }

    last_service_recheck_time_ = now;
    if (!impedance_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Adaptive impedance still degraded: waiting for service [%s]",
        impedance_service_.c_str());
      return;
    }

    degraded_due_to_service_ = false;
    degraded_reason_.clear();
    last_command_time_ = now;
    RCLCPP_WARN(
      this->get_logger(),
      "Adaptive impedance service recovered: [%s]. Supervisor resumes active adaptation.",
      impedance_service_.c_str());
  }

  void publish_adaptive_status(double stiffness, double damping, bool active)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(8);
    msg.data.push_back(filtered_left_force_z_);
    msg.data.push_back(filtered_right_force_z_);
    msg.data.push_back(filtered_left_force_z_ - filtered_right_force_z_);
    msg.data.push_back(0.5 * (std::abs(filtered_left_force_z_) + std::abs(filtered_right_force_z_)));
    msg.data.push_back(stiffness);
    msg.data.push_back(damping);
    msg.data.push_back(active ? 1.0 : 0.0);
    msg.data.push_back(request_in_flight_ ? 1.0 : 0.0);
    adaptive_params_pub_->publish(msg);
  }

  std::string left_wrench_topic_;
  std::string right_wrench_topic_;
  std::string task_stage_topic_;
  std::string task_status_topic_;
  std::string impedance_service_;
  std::string adaptive_params_topic_;

  double update_rate_hz_ {20.0};
  double force_lpf_alpha_ {0.25};
  double force_deadband_n_ {1.0};

  double nominal_translation_stiffness_ {400.0};
  double min_translation_stiffness_ {150.0};
  double max_translation_stiffness_ {700.0};
  double rotation_stiffness_ {20.0};
  double nominal_damping_ratio_ {0.8};
  double min_damping_ratio_ {0.6};
  double max_damping_ratio_ {1.2};
  double nullspace_stiffness_ {10.0};

  double force_gain_ {2.0};
  double imbalance_gain_ {8.0};
  double stiffness_update_epsilon_ {5.0};
  double damping_update_epsilon_ {0.02};
  double min_service_interval_sec_ {0.20};
  double max_refresh_interval_sec_ {1.0};
  double service_startup_wait_sec_ {2.0};
  double service_recheck_interval_sec_ {2.0};
  bool stop_on_done_ {true};

  std::unordered_set<std::string> active_stages_;

  bool have_left_ {false};
  bool have_right_ {false};
  bool task_finished_ {false};
  bool request_in_flight_ {false};
  bool adaptive_applied_ {false};
  bool degraded_due_to_service_ {false};

  double filtered_left_force_z_ {0.0};
  double filtered_right_force_z_ {0.0};
  double last_sent_stiffness_ {400.0};
  double last_sent_damping_ {0.8};

  std::string current_stage_ {"INIT"};
  std::string degraded_reason_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_service_recheck_time_ {0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr adaptive_params_pub_;
  rclcpp::Client<SetCartesianImpedance>::SharedPtr impedance_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveComplianceSupervisor>());
  rclcpp::shutdown();
  return 0;
}
