#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

class CommunicationLatencyMonitor : public rclcpp::Node
{
public:
  CommunicationLatencyMonitor()
  : Node("communication_latency_monitor")
  {
    this->declare_parameter<std::string>("latency_topic", "/joint_states");
    this->declare_parameter<std::string>("task_status_topic", "/task_status");
    this->declare_parameter<double>("target_latency_ms", 100.0);
    this->declare_parameter<std::string>("result_dir", "");
    this->declare_parameter<std::string>("result_prefix", "comm_latency");
    this->declare_parameter<bool>("stop_on_done", true);
    this->declare_parameter<int>("progress_log_interval_sec", 5);

    latency_topic_ = this->get_parameter("latency_topic").as_string();
    task_status_topic_ = this->get_parameter("task_status_topic").as_string();
    target_latency_ms_ = this->get_parameter("target_latency_ms").as_double();
    result_dir_ = this->get_parameter("result_dir").as_string();
    result_prefix_ = this->get_parameter("result_prefix").as_string();
    stop_on_done_ = this->get_parameter("stop_on_done").as_bool();
    progress_log_interval_sec_ = this->get_parameter("progress_log_interval_sec").as_int();

    if (result_dir_.empty()) {
      result_dir_ = (std::filesystem::current_path() / "launch_logs").string();
    }
    if (progress_log_interval_sec_ <= 0) {
      progress_log_interval_sec_ = 5;
    }

    std::error_code ec;
    std::filesystem::create_directories(result_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create result dir '%s': %s",
                   result_dir_.c_str(), ec.message().c_str());
      throw std::runtime_error("failed to create result directory");
    }

    const std::string stamp = make_wallclock_timestamp();
    const std::string base_name = result_prefix_ + "_" + stamp;
    samples_csv_path_ = (std::filesystem::path(result_dir_) / (base_name + "_samples.csv")).string();
    summary_csv_path_ = (std::filesystem::path(result_dir_) / (base_name + "_summary.csv")).string();
    summary_md_path_ = (std::filesystem::path(result_dir_) / (base_name + "_summary.md")).string();

    samples_stream_.open(samples_csv_path_, std::ios::out | std::ios::trunc);
    if (!samples_stream_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open samples csv: %s", samples_csv_path_.c_str());
      throw std::runtime_error("failed to open sample csv");
    }
    samples_stream_ << "index,receive_time_sec,source_stamp_sec,latency_ms\n";

    auto latency_qos = rclcpp::SensorDataQoS().keep_last(200);
    latency_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      latency_topic_, latency_qos,
      std::bind(&CommunicationLatencyMonitor::on_latency_sample, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      task_status_topic_, 10,
      std::bind(&CommunicationLatencyMonitor::on_task_status, this, std::placeholders::_1));

    progress_timer_ = this->create_wall_timer(
      std::chrono::seconds(progress_log_interval_sec_),
      std::bind(&CommunicationLatencyMonitor::log_progress, this));

    RCLCPP_INFO(this->get_logger(), "Latency monitor started.");
    RCLCPP_INFO(this->get_logger(), "  latency_topic: %s", latency_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  task_status_topic: %s", task_status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  target_latency_ms: %.2f", target_latency_ms_);
    RCLCPP_INFO(this->get_logger(), "  samples_csv: %s", samples_csv_path_.c_str());
  }

  ~CommunicationLatencyMonitor() override
  {
    finalize_and_write_report("node_shutdown");
  }

private:
  struct Stats
  {
    size_t sample_count {0};
    double min_ms {0.0};
    double max_ms {0.0};
    double mean_ms {0.0};
    double std_ms {0.0};
    double p50_ms {0.0};
    double p90_ms {0.0};
    double p95_ms {0.0};
    double p99_ms {0.0};
    size_t over_target_count {0};
    double over_target_ratio {0.0};
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

  Stats compute_stats() const
  {
    Stats stats;
    stats.sample_count = latency_ms_samples_.size();
    if (stats.sample_count == 0) {
      return stats;
    }

    auto sorted = latency_ms_samples_;
    std::sort(sorted.begin(), sorted.end());

    stats.min_ms = sorted.front();
    stats.max_ms = sorted.back();
    stats.mean_ms = std::accumulate(sorted.begin(), sorted.end(), 0.0) /
                    static_cast<double>(sorted.size());

    double sum_sq = 0.0;
    for (double v : sorted) {
      const double d = v - stats.mean_ms;
      sum_sq += d * d;
    }
    stats.std_ms = std::sqrt(sum_sq / static_cast<double>(sorted.size()));

    stats.p50_ms = percentile_from_sorted(sorted, 50.0);
    stats.p90_ms = percentile_from_sorted(sorted, 90.0);
    stats.p95_ms = percentile_from_sorted(sorted, 95.0);
    stats.p99_ms = percentile_from_sorted(sorted, 99.0);

    stats.over_target_count = static_cast<size_t>(
      std::count_if(sorted.begin(), sorted.end(),
        [&](double v) { return v > target_latency_ms_; }));

    stats.over_target_ratio = static_cast<double>(stats.over_target_count) /
                              static_cast<double>(sorted.size());
    return stats;
  }

  void on_latency_sample(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const auto receive_time = this->now();
    if (receive_time.nanoseconds() == 0) {
      ++clock_not_ready_count_;
      return;
    }

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      ++zero_stamp_count_;
      return;
    }

    const rclcpp::Time source_time(msg->header.stamp);
    const double latency_ms = (receive_time - source_time).seconds() * 1000.0;
    if (!std::isfinite(latency_ms)) {
      ++invalid_sample_count_;
      return;
    }

    if (latency_ms < 0.0) {
      ++negative_latency_count_;
      return;
    }

    latency_ms_samples_.push_back(latency_ms);
    ++sample_index_;

    samples_stream_ << sample_index_ << ','
                    << std::fixed << std::setprecision(6) << receive_time.seconds() << ','
                    << std::fixed << std::setprecision(6) << source_time.seconds() << ','
                    << std::fixed << std::setprecision(3) << latency_ms << '\n';
    if (sample_index_ % 100 == 0) {
      samples_stream_.flush();
    }
  }

  void on_task_status(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!stop_on_done_) {
      return;
    }

    if (msg->data == "DONE" || msg->data == "ERROR") {
      RCLCPP_INFO(this->get_logger(),
                  "Received task terminal status '%s', finalizing latency report.",
                  msg->data.c_str());
      finalize_and_write_report("task_status_" + msg->data);
      rclcpp::shutdown();
    }
  }

  void log_progress()
  {
    if (finalized_) {
      return;
    }

    const auto stats = compute_stats();
    if (stats.sample_count == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[latency] waiting samples... zero_stamp=%zu invalid=%zu clock_not_ready=%zu",
                  zero_stamp_count_, invalid_sample_count_, clock_not_ready_count_);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "[latency] n=%zu mean=%.2fms p95=%.2fms max=%.2fms over(%.1fms)=%.2f%%",
                stats.sample_count,
                stats.mean_ms,
                stats.p95_ms,
                stats.max_ms,
                target_latency_ms_,
                stats.over_target_ratio * 100.0);
  }

  void write_summary_files(const Stats& stats, const std::string& reason)
  {
    {
      std::ofstream summary_csv(summary_csv_path_, std::ios::out | std::ios::trunc);
      summary_csv << "metric,value\n";
      summary_csv << "reason," << reason << "\n";
      summary_csv << "latency_topic," << latency_topic_ << "\n";
      summary_csv << "target_latency_ms," << target_latency_ms_ << "\n";
      summary_csv << "sample_count," << stats.sample_count << "\n";
      summary_csv << "min_ms," << stats.min_ms << "\n";
      summary_csv << "mean_ms," << stats.mean_ms << "\n";
      summary_csv << "std_ms," << stats.std_ms << "\n";
      summary_csv << "p50_ms," << stats.p50_ms << "\n";
      summary_csv << "p90_ms," << stats.p90_ms << "\n";
      summary_csv << "p95_ms," << stats.p95_ms << "\n";
      summary_csv << "p99_ms," << stats.p99_ms << "\n";
      summary_csv << "max_ms," << stats.max_ms << "\n";
      summary_csv << "over_target_count," << stats.over_target_count << "\n";
      summary_csv << "over_target_ratio," << stats.over_target_ratio << "\n";
      summary_csv << "zero_stamp_count," << zero_stamp_count_ << "\n";
      summary_csv << "invalid_sample_count," << invalid_sample_count_ << "\n";
      summary_csv << "negative_latency_count," << negative_latency_count_ << "\n";
      summary_csv << "clock_not_ready_count," << clock_not_ready_count_ << "\n";
    }

    {
      std::ofstream summary_md(summary_md_path_, std::ios::out | std::ios::trunc);
      summary_md << "# Communication Latency Report\n\n";
      summary_md << "- Reason: " << reason << "\n";
      summary_md << "- Topic: " << latency_topic_ << "\n";
      summary_md << "- Target threshold: " << std::fixed << std::setprecision(1)
                 << target_latency_ms_ << " ms\n";
      summary_md << "- Samples csv: " << samples_csv_path_ << "\n";
      summary_md << "- Summary csv: " << summary_csv_path_ << "\n\n";

      summary_md << "| Metric | Value |\n";
      summary_md << "|---|---:|\n";
      summary_md << "| sample_count | " << stats.sample_count << " |\n";
      summary_md << "| min_ms | " << std::fixed << std::setprecision(3) << stats.min_ms << " |\n";
      summary_md << "| mean_ms | " << std::fixed << std::setprecision(3) << stats.mean_ms << " |\n";
      summary_md << "| std_ms | " << std::fixed << std::setprecision(3) << stats.std_ms << " |\n";
      summary_md << "| p50_ms | " << std::fixed << std::setprecision(3) << stats.p50_ms << " |\n";
      summary_md << "| p90_ms | " << std::fixed << std::setprecision(3) << stats.p90_ms << " |\n";
      summary_md << "| p95_ms | " << std::fixed << std::setprecision(3) << stats.p95_ms << " |\n";
      summary_md << "| p99_ms | " << std::fixed << std::setprecision(3) << stats.p99_ms << " |\n";
      summary_md << "| max_ms | " << std::fixed << std::setprecision(3) << stats.max_ms << " |\n";
      summary_md << "| over_target_count | " << stats.over_target_count << " |\n";
      summary_md << "| over_target_ratio(%) | " << std::fixed << std::setprecision(2)
                 << stats.over_target_ratio * 100.0 << " |\n";
      summary_md << "| p95_within_target | "
                 << ((stats.p95_ms <= target_latency_ms_) ? "PASS" : "FAIL") << " |\n";
      summary_md << "| max_within_target | "
                 << ((stats.max_ms <= target_latency_ms_) ? "PASS" : "FAIL") << " |\n";
      summary_md << "| zero_stamp_count | " << zero_stamp_count_ << " |\n";
      summary_md << "| invalid_sample_count | " << invalid_sample_count_ << " |\n";
      summary_md << "| negative_latency_count | " << negative_latency_count_ << " |\n";
      summary_md << "| clock_not_ready_count | " << clock_not_ready_count_ << " |\n";
    }
  }

  void finalize_and_write_report(const std::string& reason)
  {
    if (finalized_) {
      return;
    }
    finalized_ = true;

    if (samples_stream_.is_open()) {
      samples_stream_.flush();
      samples_stream_.close();
    }

    const auto stats = compute_stats();
    write_summary_files(stats, reason);

    RCLCPP_INFO(this->get_logger(), "Latency report saved.");
    RCLCPP_INFO(this->get_logger(), "  samples: %s", samples_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  summary: %s", summary_csv_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  summary_table: %s", summary_md_path_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "  quick_result: n=%zu p95=%.2fms max=%.2fms target=%.2fms",
                stats.sample_count, stats.p95_ms, stats.max_ms, target_latency_ms_);
  }

  std::string latency_topic_;
  std::string task_status_topic_;
  double target_latency_ms_ {100.0};
  std::string result_dir_;
  std::string result_prefix_;
  bool stop_on_done_ {true};
  int progress_log_interval_sec_ {5};

  std::string samples_csv_path_;
  std::string summary_csv_path_;
  std::string summary_md_path_;

  std::ofstream samples_stream_;
  std::vector<double> latency_ms_samples_;

  size_t sample_index_ {0};
  size_t zero_stamp_count_ {0};
  size_t invalid_sample_count_ {0};
  size_t negative_latency_count_ {0};
  size_t clock_not_ready_count_ {0};

  bool finalized_ {false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr latency_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr progress_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommunicationLatencyMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
