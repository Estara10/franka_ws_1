#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

class TaskMetricsMonitor : public rclcpp::Node
{
public:
  TaskMetricsMonitor()
  : Node("task_metrics_monitor"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("task_stage_topic", "/task_stage");
    this->declare_parameter<std::string>("task_status_topic", "/task_status");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("left_frame", "mj_left_hand");
    this->declare_parameter<std::string>("right_frame", "mj_right_hand");
    this->declare_parameter<double>("sync_target_mm", 5.0);
    this->declare_parameter<double>("ee_target_mm", 2.0);
    this->declare_parameter<bool>("enable_rotate", false);
    this->declare_parameter<double>("descend_place_z", 0.0);
    this->declare_parameter<double>("descend_shift_x", 999.0);
    this->declare_parameter<std::string>("result_dir", "");
    this->declare_parameter<std::string>("result_prefix", "task_metrics");
    this->declare_parameter<bool>("stop_on_done", true);
    this->declare_parameter<int>("sample_period_ms", 20);
    this->declare_parameter<int>("progress_log_interval_sec", 5);

    joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
    task_stage_topic_ = this->get_parameter("task_stage_topic").as_string();
    task_status_topic_ = this->get_parameter("task_status_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    left_frame_ = this->get_parameter("left_frame").as_string();
    right_frame_ = this->get_parameter("right_frame").as_string();
    sync_target_mm_ = this->get_parameter("sync_target_mm").as_double();
    ee_target_mm_ = this->get_parameter("ee_target_mm").as_double();
    enable_rotate_ = this->get_parameter("enable_rotate").as_bool();
    descend_place_z_ = this->get_parameter("descend_place_z").as_double();
    descend_shift_x_ = this->get_parameter("descend_shift_x").as_double();
    result_dir_ = this->get_parameter("result_dir").as_string();
    result_prefix_ = this->get_parameter("result_prefix").as_string();
    stop_on_done_ = this->get_parameter("stop_on_done").as_bool();
    sample_period_ms_ = this->get_parameter("sample_period_ms").as_int();
    progress_log_interval_sec_ = this->get_parameter("progress_log_interval_sec").as_int();

    if (descend_place_z_ <= 0.0) {
      descend_place_z_ = enable_rotate_ ? 0.18 : 0.16;
    }
    if (std::abs(descend_shift_x_) > 10.0) {
      descend_shift_x_ = enable_rotate_ ? -0.10 : 0.0;
    }
    if (result_dir_.empty()) {
      result_dir_ = (std::filesystem::current_path() / "launch_logs").string();
    }
    if (sample_period_ms_ <= 0) {
      sample_period_ms_ = 20;
    }
    if (progress_log_interval_sec_ <= 0) {
      progress_log_interval_sec_ = 5;
    }

    std::error_code ec;
    std::filesystem::create_directories(result_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create result dir '%s': %s",
                   result_dir_.c_str(), ec.message().c_str());
      throw std::runtime_error("failed to create metrics result directory");
    }

    const std::string stamp = make_wallclock_timestamp();
    const std::string base_name = result_prefix_ + "_" + stamp;
    sync_samples_csv_path_ = (std::filesystem::path(result_dir_) / (base_name + "_sync_samples.csv")).string();
    summary_csv_path_ = (std::filesystem::path(result_dir_) / (base_name + "_summary.csv")).string();
    summary_md_path_ = (std::filesystem::path(result_dir_) / (base_name + "_summary.md")).string();

    sync_samples_stream_.open(sync_samples_csv_path_, std::ios::out | std::ios::trunc);
    if (!sync_samples_stream_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open sync samples csv: %s", sync_samples_csv_path_.c_str());
      throw std::runtime_error("failed to open sync samples csv");
    }
    sync_samples_stream_ << "index,time_sec,stage,dt_sec,left_step_mm,right_step_mm,sync_error_mm\n";

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(200);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, sensor_qos,
      std::bind(&TaskMetricsMonitor::on_joint_state, this, std::placeholders::_1));

    stage_sub_ = this->create_subscription<std_msgs::msg::String>(
      task_stage_topic_, 20,
      std::bind(&TaskMetricsMonitor::on_stage, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      task_status_topic_, 10,
      std::bind(&TaskMetricsMonitor::on_task_status, this, std::placeholders::_1));

    sample_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(sample_period_ms_),
      std::bind(&TaskMetricsMonitor::on_sample_timer, this));

    progress_timer_ = this->create_wall_timer(
      std::chrono::seconds(progress_log_interval_sec_),
      std::bind(&TaskMetricsMonitor::log_progress, this));

    RCLCPP_INFO(this->get_logger(), "Task metrics monitor started.");
    RCLCPP_INFO(this->get_logger(), "  joint_state_topic: %s", joint_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  task_stage_topic: %s", task_stage_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  task_status_topic: %s", task_status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  frames: base=%s left=%s right=%s",
                base_frame_.c_str(), left_frame_.c_str(), right_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  thresholds: sync<=%.2fmm, ee<=%.2fmm",
                sync_target_mm_, ee_target_mm_);
    RCLCPP_INFO(this->get_logger(), "  descend target: place_z=%.3f, shift_x=%.3f",
                descend_place_z_, descend_shift_x_);
    RCLCPP_INFO(this->get_logger(), "  sync_samples_csv: %s", sync_samples_csv_path_.c_str());
  }

  ~TaskMetricsMonitor() override
  {
    finalize_and_write_report("node_shutdown");
  }

private:
  struct Vec3
  {
    double x {0.0};
    double y {0.0};
    double z {0.0};
  };

  struct Stats
  {
    size_t count {0};
    double min {0.0};
    double max {0.0};
    double mean {0.0};
    double p95 {0.0};
  };

  static std::string make_wallclock_timestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm {};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  static std::string normalize_stage(const std::string& s)
  {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });
    return out;
  }

  static double distance(const Vec3& a, const Vec3& b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static double percentile_from_sorted(const std::vector<double>& sorted_values, double p)
  {
    if (sorted_values.empty()) {
      return 0.0;
    }
    if (p <= 0.0) {
      return sorted_values.front();
    }
    if (p >= 100.0) {
      return sorted_values.back();
    }

    const double rank = (p / 100.0) * static_cast<double>(sorted_values.size() - 1);
    const auto low_idx = static_cast<size_t>(std::floor(rank));
    const auto high_idx = static_cast<size_t>(std::ceil(rank));
    const double frac = rank - static_cast<double>(low_idx);

    if (high_idx == low_idx) {
      return sorted_values[low_idx];
    }
    return sorted_values[low_idx] * (1.0 - frac) + sorted_values[high_idx] * frac;
  }

  static Stats compute_stats(const std::vector<double>& values)
  {
    Stats stats;
    stats.count = values.size();
    if (stats.count == 0) {
      return stats;
    }

    auto sorted = values;
    std::sort(sorted.begin(), sorted.end());
    stats.min = sorted.front();
    stats.max = sorted.back();
    stats.mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) /
                 static_cast<double>(sorted.size());
    stats.p95 = percentile_from_sorted(sorted, 95.0);
    return stats;
  }

  bool is_carry_stage(const std::string& stage) const
  {
    return stage == "TRANSPORT" || stage == "ROTATE" || stage == "DESCEND";
  }

  std::optional<Vec3> lookup_frame_position(const std::string& frame)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
      Vec3 p;
      p.x = tf.transform.translation.x;
      p.y = tf.transform.translation.y;
      p.z = tf.transform.translation.z;
      return p;
    } catch (const tf2::TransformException&) {
      return std::nullopt;
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    rclcpp::Time sample_time(msg->header.stamp);
    if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
      sample_time = this->now();
    }

    if (sample_time.nanoseconds() == 0) {
      ++joint_time_invalid_count_;
      return;
    }

    if (!have_last_joint_time_) {
      last_joint_time_ = sample_time;
      have_last_joint_time_ = true;
      return;
    }

    const double dt = (sample_time - last_joint_time_).seconds();
    last_joint_time_ = sample_time;

    if (dt <= 0.0 || dt > 1.0) {
      ++joint_dt_invalid_count_;
      return;
    }

    const size_t n = std::min(msg->velocity.size(), msg->effort.size());
    if (n == 0) {
      ++joint_payload_missing_count_;
      return;
    }

    double inst_power = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const double v = msg->velocity[i];
      const double tau = msg->effort[i];
      if (!std::isfinite(v) || !std::isfinite(tau)) {
        continue;
      }
      inst_power += std::abs(v * tau);
    }

    if (!std::isfinite(inst_power)) {
      ++joint_power_invalid_count_;
      return;
    }

    energy_proxy_j_ += inst_power * dt;
    power_samples_.push_back(inst_power);
  }

  void on_stage(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string new_stage = normalize_stage(msg->data);
    if (new_stage.empty()) {
      return;
    }

    if (new_stage == current_stage_) {
      return;
    }

    const std::string previous_stage = current_stage_;
    current_stage_ = new_stage;
    ++stage_transition_count_;

    if (previous_stage == "DESCEND" && current_stage_ == "PLACE") {
      finalize_descend_terminal_error();
    }

    if (current_stage_ == "DESCEND") {
      capture_descend_reference();
    }

    RCLCPP_INFO(this->get_logger(), "[metrics] stage transition: %s -> %s",
                previous_stage.empty() ? "(none)" : previous_stage.c_str(),
                current_stage_.c_str());
  }

  void on_task_status(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!stop_on_done_) {
      return;
    }

    if (msg->data == "DONE" || msg->data == "ERROR") {
      finalize_and_write_report("task_status_" + msg->data);
      rclcpp::shutdown();
    }
  }

  void on_sample_timer()
  {
    const auto left_opt = lookup_frame_position(left_frame_);
    const auto right_opt = lookup_frame_position(right_frame_);
    if (!left_opt || !right_opt) {
      ++tf_lookup_miss_count_;
      return;
    }

    const Vec3 left = *left_opt;
    const Vec3 right = *right_opt;
    const rclcpp::Time now_ros = this->now();

    latest_left_ = left;
    latest_right_ = right;
    have_latest_pose_ = true;

    if (is_carry_stage(current_stage_) && have_previous_pose_) {
      const double dt = (now_ros - previous_pose_time_).seconds();
      if (dt > 0.0 && dt < 1.0) {
        const double left_step_mm = distance(left, previous_left_) * 1000.0;
        const double right_step_mm = distance(right, previous_right_) * 1000.0;
        const double sync_error_mm = std::abs(left_step_mm - right_step_mm);

        sync_error_mm_samples_.push_back(sync_error_mm);
        ++sync_sample_index_;

        sync_samples_stream_ << sync_sample_index_ << ','
                             << std::fixed << std::setprecision(6) << now_ros.seconds() << ','
                             << current_stage_ << ','
                             << std::fixed << std::setprecision(6) << dt << ','
                             << std::fixed << std::setprecision(3) << left_step_mm << ','
                             << std::fixed << std::setprecision(3) << right_step_mm << ','
                             << std::fixed << std::setprecision(3) << sync_error_mm << '\n';
        if (sync_sample_index_ % 200 == 0) {
          sync_samples_stream_.flush();
        }
      }
    }

    previous_left_ = left;
    previous_right_ = right;
    previous_pose_time_ = now_ros;
    have_previous_pose_ = true;
  }

  void capture_descend_reference()
  {
    const auto left_opt = lookup_frame_position(left_frame_);
    const auto right_opt = lookup_frame_position(right_frame_);
    if (!left_opt || !right_opt) {
      ++descend_ref_capture_fail_count_;
      descend_reference_valid_ = false;
      RCLCPP_WARN(this->get_logger(),
                  "[metrics] unable to capture DESCEND reference (tf unavailable)");
      return;
    }

    const Vec3 left_start = *left_opt;
    const Vec3 right_start = *right_opt;

    expected_descend_end_left_ = left_start;
    expected_descend_end_right_ = right_start;

    expected_descend_end_left_.x += descend_shift_x_;
    expected_descend_end_right_.x += descend_shift_x_;
    expected_descend_end_left_.z = descend_place_z_;
    expected_descend_end_right_.z = descend_place_z_;

    descend_reference_valid_ = true;
    descend_reference_armed_ = true;

    RCLCPP_INFO(this->get_logger(),
                "[metrics] DESCEND reference captured: expected left=(%.3f,%.3f,%.3f) right=(%.3f,%.3f,%.3f)",
                expected_descend_end_left_.x, expected_descend_end_left_.y, expected_descend_end_left_.z,
                expected_descend_end_right_.x, expected_descend_end_right_.y, expected_descend_end_right_.z);
  }

  void finalize_descend_terminal_error()
  {
    if (!descend_reference_armed_) {
      return;
    }

    descend_reference_armed_ = false;

    if (!descend_reference_valid_ || !have_latest_pose_) {
      ++descend_terminal_eval_fail_count_;
      descend_terminal_error_valid_ = false;
      RCLCPP_WARN(this->get_logger(),
                  "[metrics] unable to evaluate DESCEND terminal ee error (missing reference or pose)");
      return;
    }

    const double left_error_mm = distance(latest_left_, expected_descend_end_left_) * 1000.0;
    const double right_error_mm = distance(latest_right_, expected_descend_end_right_) * 1000.0;
    descend_terminal_ee_error_mm_ = 0.5 * (left_error_mm + right_error_mm);
    descend_terminal_error_valid_ = true;

    RCLCPP_INFO(this->get_logger(),
                "[metrics] DESCEND terminal ee error: left=%.3fmm right=%.3fmm avg=%.3fmm",
                left_error_mm, right_error_mm, descend_terminal_ee_error_mm_);
  }

  void log_progress()
  {
    if (finalized_) {
      return;
    }

    const auto sync_stats = compute_stats(sync_error_mm_samples_);
    const auto power_stats = compute_stats(power_samples_);

    RCLCPP_INFO(this->get_logger(),
                "[metrics] stage=%s sync_n=%zu sync_p95=%.2fmm ee_terminal=%s energy_proxy=%.2fJ-like mean_power=%.2f",
                current_stage_.empty() ? "(none)" : current_stage_.c_str(),
                sync_stats.count,
                sync_stats.p95,
                descend_terminal_error_valid_ ? format_double(descend_terminal_ee_error_mm_).c_str() : "N/A",
                energy_proxy_j_,
                power_stats.mean);
  }

  static std::string format_double(double value, int precision = 6)
  {
    if (!std::isfinite(value)) {
      return "nan";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  void write_summary_files(const std::string& reason)
  {
    const auto sync_stats = compute_stats(sync_error_mm_samples_);
    const auto power_stats = compute_stats(power_samples_);

    const bool sync_pass = (sync_stats.count > 0) && (sync_stats.p95 <= sync_target_mm_);
    const bool ee_pass = descend_terminal_error_valid_ &&
                         (descend_terminal_ee_error_mm_ <= ee_target_mm_);

    {
      std::ofstream summary_csv(summary_csv_path_, std::ios::out | std::ios::trunc);
      summary_csv << "metric,value\n";
      summary_csv << "reason," << reason << "\n";
      summary_csv << "base_frame," << base_frame_ << "\n";
      summary_csv << "left_frame," << left_frame_ << "\n";
      summary_csv << "right_frame," << right_frame_ << "\n";
      summary_csv << "sync_target_mm," << format_double(sync_target_mm_) << "\n";
      summary_csv << "ee_target_mm," << format_double(ee_target_mm_) << "\n";
      summary_csv << "sync_sample_count," << sync_stats.count << "\n";
      summary_csv << "sync_error_min_mm," << format_double(sync_stats.min) << "\n";
      summary_csv << "sync_error_mean_mm," << format_double(sync_stats.mean) << "\n";
      summary_csv << "sync_error_p95_mm," << format_double(sync_stats.p95) << "\n";
      summary_csv << "sync_error_max_mm," << format_double(sync_stats.max) << "\n";
      summary_csv << "sync_within_target_pass," << (sync_pass ? "PASS" : "FAIL") << "\n";
      summary_csv << "energy_proxy_j," << format_double(energy_proxy_j_) << "\n";
      summary_csv << "power_sample_count," << power_stats.count << "\n";
      summary_csv << "power_mean_abs," << format_double(power_stats.mean) << "\n";
      summary_csv << "descend_terminal_error_valid," << (descend_terminal_error_valid_ ? "true" : "false") << "\n";
      summary_csv << "ee_terminal_error_mm," <<
        (descend_terminal_error_valid_ ? format_double(descend_terminal_ee_error_mm_) : "nan") << "\n";
      summary_csv << "ee_within_target_pass," << (ee_pass ? "PASS" : "FAIL") << "\n";
      summary_csv << "stage_transition_count," << stage_transition_count_ << "\n";
      summary_csv << "tf_lookup_miss_count," << tf_lookup_miss_count_ << "\n";
      summary_csv << "joint_payload_missing_count," << joint_payload_missing_count_ << "\n";
      summary_csv << "joint_dt_invalid_count," << joint_dt_invalid_count_ << "\n";
      summary_csv << "joint_time_invalid_count," << joint_time_invalid_count_ << "\n";
      summary_csv << "joint_power_invalid_count," << joint_power_invalid_count_ << "\n";
      summary_csv << "descend_ref_capture_fail_count," << descend_ref_capture_fail_count_ << "\n";
      summary_csv << "descend_terminal_eval_fail_count," << descend_terminal_eval_fail_count_ << "\n";
      summary_csv << "sync_samples_csv," << sync_samples_csv_path_ << "\n";
    }

    {
      std::ofstream summary_md(summary_md_path_, std::ios::out | std::ios::trunc);
      summary_md << "# Task Quantitative Metrics Report\n\n";
      summary_md << "- Reason: " << reason << "\n";
      summary_md << "- Stage topic: " << task_stage_topic_ << "\n";
      summary_md << "- Joint state topic: " << joint_state_topic_ << "\n";
      summary_md << "- Sync threshold: " << std::fixed << std::setprecision(2) << sync_target_mm_ << " mm\n";
      summary_md << "- EE threshold: " << std::fixed << std::setprecision(2) << ee_target_mm_ << " mm\n";
      summary_md << "- Sync samples csv: " << sync_samples_csv_path_ << "\n";
      summary_md << "- Summary csv: " << summary_csv_path_ << "\n\n";

      summary_md << "| Metric | Value |\n";
      summary_md << "|---|---:|\n";
      summary_md << "| sync_sample_count | " << sync_stats.count << " |\n";
      summary_md << "| sync_error_mean_mm | " << std::fixed << std::setprecision(3) << sync_stats.mean << " |\n";
      summary_md << "| sync_error_p95_mm | " << std::fixed << std::setprecision(3) << sync_stats.p95 << " |\n";
      summary_md << "| sync_error_max_mm | " << std::fixed << std::setprecision(3) << sync_stats.max << " |\n";
      summary_md << "| sync_within_target | " << (sync_pass ? "PASS" : "FAIL") << " |\n";
      summary_md << "| energy_proxy_j | " << std::fixed << std::setprecision(6) << energy_proxy_j_ << " |\n";
      summary_md << "| ee_terminal_error_mm | ";
      if (descend_terminal_error_valid_) {
        summary_md << std::fixed << std::setprecision(3) << descend_terminal_ee_error_mm_;
      } else {
        summary_md << "N/A";
      }
      summary_md << " |\n";
      summary_md << "| ee_within_target | " << (ee_pass ? "PASS" : "FAIL") << " |\n";
      summary_md << "| stage_transition_count | " << stage_transition_count_ << " |\n";
      summary_md << "| tf_lookup_miss_count | " << tf_lookup_miss_count_ << " |\n";
      summary_md << "| joint_payload_missing_count | " << joint_payload_missing_count_ << " |\n";
      summary_md << "| descend_ref_capture_fail_count | " << descend_ref_capture_fail_count_ << " |\n";
      summary_md << "| descend_terminal_eval_fail_count | " << descend_terminal_eval_fail_count_ << " |\n";
    }
  }

  void finalize_and_write_report(const std::string& reason)
  {
    if (finalized_) {
      return;
    }
    finalized_ = true;

    if (sync_samples_stream_.is_open()) {
      sync_samples_stream_.flush();
      sync_samples_stream_.close();
    }

    if (descend_reference_armed_) {
      finalize_descend_terminal_error();
    }

    write_summary_files(reason);

    const auto sync_stats = compute_stats(sync_error_mm_samples_);
    RCLCPP_INFO(this->get_logger(), "Task metrics report saved.");
    RCLCPP_INFO(this->get_logger(), "  sync_samples: %s", sync_samples_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  summary_csv: %s", summary_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  summary_md: %s", summary_md_path_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "  quick_result: sync_p95=%.2fmm (<=%.2f), ee_terminal=%smm (<=%.2f), energy_proxy=%.3f",
                sync_stats.p95,
                sync_target_mm_,
                descend_terminal_error_valid_ ? format_double(descend_terminal_ee_error_mm_, 3).c_str() : "N/A",
                ee_target_mm_,
                energy_proxy_j_);
  }

  std::string joint_state_topic_;
  std::string task_stage_topic_;
  std::string task_status_topic_;
  std::string base_frame_;
  std::string left_frame_;
  std::string right_frame_;

  double sync_target_mm_ {5.0};
  double ee_target_mm_ {2.0};
  bool enable_rotate_ {false};
  double descend_place_z_ {0.16};
  double descend_shift_x_ {0.0};

  std::string result_dir_;
  std::string result_prefix_;
  bool stop_on_done_ {true};
  int sample_period_ms_ {20};
  int progress_log_interval_sec_ {5};

  std::string current_stage_;
  size_t stage_transition_count_ {0};

  std::string sync_samples_csv_path_;
  std::string summary_csv_path_;
  std::string summary_md_path_;

  std::ofstream sync_samples_stream_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr progress_timer_;

  std::vector<double> sync_error_mm_samples_;
  size_t sync_sample_index_ {0};

  std::vector<double> power_samples_;
  double energy_proxy_j_ {0.0};
  rclcpp::Time last_joint_time_;
  bool have_last_joint_time_ {false};

  Vec3 previous_left_;
  Vec3 previous_right_;
  rclcpp::Time previous_pose_time_;
  bool have_previous_pose_ {false};

  Vec3 latest_left_;
  Vec3 latest_right_;
  bool have_latest_pose_ {false};

  Vec3 expected_descend_end_left_;
  Vec3 expected_descend_end_right_;
  bool descend_reference_valid_ {false};
  bool descend_reference_armed_ {false};
  double descend_terminal_ee_error_mm_ {0.0};
  bool descend_terminal_error_valid_ {false};

  size_t tf_lookup_miss_count_ {0};
  size_t joint_payload_missing_count_ {0};
  size_t joint_dt_invalid_count_ {0};
  size_t joint_time_invalid_count_ {0};
  size_t joint_power_invalid_count_ {0};
  size_t descend_ref_capture_fail_count_ {0};
  size_t descend_terminal_eval_fail_count_ {0};

  bool finalized_ {false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskMetricsMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
