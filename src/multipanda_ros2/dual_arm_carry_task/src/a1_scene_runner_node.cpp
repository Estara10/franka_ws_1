// A1 P1a: ROS side of the scene-application chain.
//
// Applies ONE sampled rod pose to a running MuJoCo instance via
// /set_body_state, waits for the simulation to settle, reads the pose back,
// and writes a manifest. It does not publish a truth topic, does not start the
// task, does not touch the planning scene, and never writes MuJoCo state
// directly.
//
// Scope and failure semantics: see
// docs/deliverables/A1_P0_接口核查与实现方案.md sections 4 and 10.
//
// Node name is `a1_scene_sampling` so that config/a1_scene_sampling.yaml loads
// with `--params-file` without any namespace juggling.

#include "dual_arm_carry_task/a1_sampling.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_ros_msgs/srv/get_body_state.hpp>
#include <mujoco_ros_msgs/srv/get_sim_info.hpp>
#include <mujoco_ros_msgs/srv/set_body_state.hpp>

#include <dual_arm_a1_msgs/msg/rod_pose_truth.hpp>

#include <rclcpp/rclcpp.hpp>

namespace {

// Marker file the runner script creates when it reserves a fresh run
// directory on the node's behalf.
constexpr const char* kScriptReservationMarker = ".reserved_by_a1_run_scene";

// Set from a signal handler so that an interrupted run still writes a manifest.
std::atomic<bool> g_abort_requested {false};

extern "C" void handleAbortSignal(int signum)
{
  static_cast<void>(signum);  // the signal number is not needed
  // Only an atomic store here: nothing else is async-signal-safe.
  g_abort_requested.store(true);
}

using SetBodyState = mujoco_ros_msgs::srv::SetBodyState;
using GetBodyState = mujoco_ros_msgs::srv::GetBodyState;
using GetSimInfo = mujoco_ros_msgs::srv::GetSimInfo;

// --- CLI ---------------------------------------------------------------------

struct Args {
  int32_t candidate_index {0};
  std::string run_id;
  bool dry_run {false};
  bool seed_given {false};
  int64_t seed {0};
  std::string manifest_dir;
  std::string models_dir;
  std::string srdf_path;
  std::string git_commit;
  std::string git_dirty;
  std::string domain_id;
  std::string localhost_only;
  // A1 P1b: publish the settled truth and stay alive so the latched sample can
  // be delivered to a task that starts afterwards.
  bool publish_truth {false};
  std::string truth_topic {"/a1_rod_pose_truth"};
  double hold_sec {0.0};
  std::vector<std::string> notes;
  // Explicit-pose override, used to apply a KNOWN test pose (yaw exactly 0,
  // positive and negative yaw) during verification. The uniform sampler cannot
  // produce an exact value. Recorded in the manifest as pose_source.
  int explicit_fields {0};
  double ex_x {0.0};
  double ex_y {0.0};
  double ex_yaw_deg {0.0};
  // Index of "--ros-args" in argv, or argc when absent. Everything from here on
  // is handed to rclcpp untouched; our own flags are stripped first so that
  // ROS argument parsing never has to interpret them.
  int ros_args_index {0};
};

void usage()
{
  std::cout
      << "usage: a1_scene_runner_node [--candidate-index N] [--run-id ID] [--dry-run]\n"
         "                              [--seed N] [--manifest-dir DIR] [--models-dir DIR]\n"
         "                              [--srdf-path PATH] [--git-commit SHA] [--git-dirty 0|1]\n"
         "                              [--domain-id N] [--localhost-only 0|1]\n"
         "                              [--publish-truth] [--truth-topic T] [--hold-sec S]\n"
         "                              [--x M --y M --yaw-deg DEG]  (known test pose)\n"
         "                              [--ros-args --params-file FILE]\n";
}

// Strict integer / double parsing: the whole token must be consumed, and the
// value must be finite and inside the requested range. std::stoll followed by a
// narrowing cast would silently accept overflow.
int64_t parseInteger(const char* flag, const std::string& text, int64_t lo, int64_t hi)
{
  std::size_t pos = 0;
  int64_t value = 0;
  try {
    value = static_cast<int64_t>(std::stoll(text, &pos, 10));
  } catch (const std::exception&) {
    throw std::runtime_error(std::string("invalid integer for ") + flag + ": " + text);
  }
  if (pos != text.size()) {
    throw std::runtime_error(std::string("trailing characters for ") + flag + ": " + text);
  }
  if (value < lo || value > hi) {
    throw std::runtime_error(std::string(flag) + " out of range: " + text);
  }
  return static_cast<int64_t>(value);
}

double parseNumber(const char* flag, const std::string& text)
{
  std::size_t pos = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &pos);
  } catch (const std::exception&) {
    throw std::runtime_error(std::string("invalid number for ") + flag + ": " + text);
  }
  if (pos != text.size()) {
    throw std::runtime_error(std::string("trailing characters for ") + flag + ": " + text);
  }
  if (!std::isfinite(value)) {
    throw std::runtime_error(std::string(flag) + " must be finite: " + text);
  }
  return value;
}

Args parseArgs(int argc, char** argv)
{
  Args a;
  a.ros_args_index = argc;
  std::set<std::string> seen;
  auto mark = [&seen](const std::string& flag) {
    if (!seen.insert(flag).second) {
      throw std::runtime_error("duplicate argument: " + flag);
    }
  };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return argv[++i];
    };
    if (arg == "--ros-args") {
      // Everything from here on belongs to rclcpp (e.g. --params-file).
      a.ros_args_index = i;
      break;
    } else if (arg == "--candidate-index") {
      mark(arg);
      a.candidate_index = static_cast<int32_t>(
          parseInteger("--candidate-index", next("--candidate-index"), 0, 100000));
    } else if (arg == "--run-id") {
      mark(arg);
      a.run_id = next("--run-id");
    } else if (arg == "--dry-run") {
      mark(arg);
      a.dry_run = true;
    } else if (arg == "--seed") {
      mark(arg);
      a.seed = parseInteger("--seed", next("--seed"), 0, 9007199254740992LL);
      a.seed_given = true;
    } else if (arg == "--manifest-dir") {
      mark(arg);
      a.manifest_dir = next("--manifest-dir");
    } else if (arg == "--models-dir") {
      mark(arg);
      a.models_dir = next("--models-dir");
    } else if (arg == "--srdf-path") {
      mark(arg);
      a.srdf_path = next("--srdf-path");
    } else if (arg == "--git-commit") {
      mark(arg);
      a.git_commit = next("--git-commit");
    } else if (arg == "--git-dirty") {
      mark(arg);
      a.git_dirty = next("--git-dirty");
    } else if (arg == "--domain-id") {
      mark(arg);
      a.domain_id = next("--domain-id");
    } else if (arg == "--localhost-only") {
      mark(arg);
      a.localhost_only = next("--localhost-only");
    } else if (arg == "--publish-truth") {
      mark(arg);
      a.publish_truth = true;
    } else if (arg == "--truth-topic") {
      mark(arg);
      a.truth_topic = next("--truth-topic");
    } else if (arg == "--hold-sec") {
      mark(arg);
      a.hold_sec = parseNumber("--hold-sec", next("--hold-sec"));
    } else if (arg == "--x") {
      mark(arg);
      a.ex_x = parseNumber("--x", next("--x"));
      ++a.explicit_fields;
    } else if (arg == "--y") {
      mark(arg);
      a.ex_y = parseNumber("--y", next("--y"));
      ++a.explicit_fields;
    } else if (arg == "--yaw-deg") {
      mark(arg);
      a.ex_yaw_deg = parseNumber("--yaw-deg", next("--yaw-deg"));
      ++a.explicit_fields;
    } else if (arg == "--help" || arg == "-h") {
      usage();
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  return a;
}

// --- JSON helpers ------------------------------------------------------------

std::string jsonEscape(const std::string& in)
{
  std::ostringstream os;
  for (unsigned char c : in) {
    switch (c) {
      case '"': os << "\\\""; break;
      case '\\': os << "\\\\"; break;
      case '\n': os << "\\n"; break;
      case '\r': os << "\\r"; break;
      case '\t': os << "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          os << buf;
        } else {
          os << static_cast<char>(c);
        }
    }
  }
  return os.str();
}

std::string jstr(const std::string& v) { return "\"" + jsonEscape(v) + "\""; }

std::string jnum(double v)
{
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}

std::string jbool(bool v) { return v ? "true" : "false"; }

std::string jnullable(const std::string& s) { return s.empty() ? "null" : jstr(s); }

std::string checksumHex(uint64_t v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "0x%016" PRIx64, v);
  return buf;
}

std::string utcNow()
{
  const std::time_t t = std::time(nullptr);
  std::tm tm {};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string envOrEmpty(const char* name)
{
  const char* v = std::getenv(name);
  return v != nullptr ? std::string(v) : std::string();
}

// Non-cryptographic content checksum. Returns false when the file is absent.
bool fileChecksum(const std::string& path, uint64_t& out)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  out = a1::fnv1a64(ss.str());
  return true;
}

// --- Clock -------------------------------------------------------------------

class RosClock : public a1::Clock {
 public:
  explicit RosClock(rclcpp::Node::SharedPtr node)
      : node_(std::move(node)), epoch_(std::chrono::steady_clock::now())
  {
  }

  double steadyNowSec() override
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch_).count();
  }

  double rosNowSec() override
  {
    return static_cast<double>(node_->get_clock()->now().nanoseconds()) * 1e-9;
  }

  bool rosTimeValid() override
  {
    auto clock = node_->get_clock();
    if (!clock->ros_time_is_active()) {
      return false;
    }
    return clock->now().nanoseconds() > 0;
  }

  void sleepUntil(double steady_deadline_sec) override
  {
    const auto tp = epoch_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(steady_deadline_sec));
    if (tp > std::chrono::steady_clock::now()) {
      std::this_thread::sleep_until(tp);
    }
  }

 private:
  rclcpp::Node::SharedPtr node_;
  std::chrono::steady_clock::time_point epoch_;
};

// --- Transport ---------------------------------------------------------------

class RosTransport : public a1::BodyStateTransport {
 public:
  RosTransport(rclcpp::Node::SharedPtr node,
               RosClock& clock,
               std::string set_service,
               std::string get_service,
               std::string sim_info_service,
               std::string body_name,
               double call_timeout_sec)
      : clock_(clock),
        body_name_(std::move(body_name)),
        call_timeout_(call_timeout_sec)
  {
    set_client_ = node->create_client<SetBodyState>(set_service);
    get_client_ = node->create_client<GetBodyState>(get_service);
    sim_info_client_ = node->create_client<GetSimInfo>(sim_info_service);
    set_service_name_ = std::move(set_service);
    get_service_name_ = std::move(get_service);
    sim_info_service_name_ = std::move(sim_info_service);
  }

  bool servicesReady() const
  {
    return set_client_->service_is_ready() && get_client_->service_is_ready() &&
           sim_info_client_->service_is_ready();
  }

  std::string missingService() const
  {
    std::string m;
    if (!set_client_->service_is_ready()) {
      m += set_service_name_ + " ";
    }
    if (!get_client_->service_is_ready()) {
      m += get_service_name_ + " ";
    }
    if (!sim_info_client_->service_is_ready()) {
      m += sim_info_service_name_ + " ";
    }
    return m;
  }

  bool waitForServices(double timeout_sec, std::string& message) override
  {
    const double deadline = clock_.steadyNowSec() + timeout_sec;
    while (clock_.steadyNowSec() < deadline) {
      if (servicesReady()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    message = "services not reachable within timeout; missing: " + missingService();
    return false;
  }

  a1::SimInfo getSimInfo(double budget_sec) override
  {
    a1::SimInfo out;
    const double timeout = std::min(call_timeout_, budget_sec);
    if (!(timeout > 0.0)) {
      out.status_message = "no time budget left for get_sim_info";
      return out;
    }
    auto req = std::make_shared<GetSimInfo::Request>();
    auto future = sim_info_client_->async_send_request(req);
    if (future.wait_for(std::chrono::duration<double>(timeout)) != std::future_status::ready) {
      out.status_message = "get_sim_info call timed out";
      return out;
    }
    auto resp = future.get();
    out.ok = true;
    out.model_valid = resp->state.model_valid;
    out.model_path = resp->state.model_path;
    out.load_count = resp->state.load_count;
    out.paused = resp->state.paused;
    return out;
  }

  a1::BodyState getState(double budget_sec, a1::PoseSample& timing_out) override
  {
    a1::BodyState out;
    const double timeout = std::min(call_timeout_, budget_sec);
    timing_out.request_ros_valid = clock_.rosTimeValid();
    timing_out.request_ros_sec = timing_out.request_ros_valid ? clock_.rosNowSec() : 0.0;
    timing_out.request_steady_sec = clock_.steadyNowSec();
    if (!(timeout > 0.0)) {
      out.status_message = "no time budget left for get_body_state";
      return out;
    }

    auto req = std::make_shared<GetBodyState::Request>();
    req->name = body_name_;
    req->admin_hash = admin_hash_;
    auto future = get_client_->async_send_request(req);
    const bool ready =
        future.wait_for(std::chrono::duration<double>(timeout)) == std::future_status::ready;
    // The response instant is the sample time: the value describes the body at
    // the moment the server answered, not when the request was sent.
    timing_out.ros_time_valid = clock_.rosTimeValid();
    timing_out.ros_time_sec = timing_out.ros_time_valid ? clock_.rosNowSec() : 0.0;
    timing_out.steady_time_sec = clock_.steadyNowSec();
    if (!ready) {
      timing_out.ros_time_valid = false;
      out.status_message = "get_body_state call timed out";
      return out;
    }
    auto resp = future.get();
    out.ok = resp->success;
    out.status_message = resp->status_message;
    if (!resp->success) {
      return out;
    }
    out.frame_id = resp->state.pose.header.frame_id;
    out.mass = static_cast<double>(resp->state.mass);
    out.pos = {resp->state.pose.pose.position.x,
               resp->state.pose.pose.position.y,
               resp->state.pose.pose.position.z};
    out.quat = {resp->state.pose.pose.orientation.x,
                resp->state.pose.pose.orientation.y,
                resp->state.pose.pose.orientation.z,
                resp->state.pose.pose.orientation.w};
    out.twist_lin = {resp->state.twist.twist.linear.x,
                     resp->state.twist.twist.linear.y,
                     resp->state.twist.twist.linear.z};
    out.twist_ang = {resp->state.twist.twist.angular.x,
                     resp->state.twist.twist.angular.y,
                     resp->state.twist.twist.angular.z};
    // GetBodyStateCB builds a default Header (ros_api.cpp:282,292), so the
    // returned stamp is always zero. Record that fact instead of pretending it
    // is a measurement time.
    out.pose_stamp_is_zero =
        (resp->state.pose.header.stamp.sec == 0 && resp->state.pose.header.stamp.nanosec == 0);
    return out;
  }

  a1::WriteOutcome setPose(const a1::Candidate& c,
                           double budget_sec,
                           a1::BodyStateTransport::WriteTiming& timing,
                           std::string& status_message) override
  {
    if (!set_client_->service_is_ready()) {
      status_message = "set_body_state service is not ready";
      return a1::WriteOutcome::UNAVAILABLE;
    }
    const double timeout = std::min(call_timeout_, budget_sec);
    if (!(timeout > 0.0)) {
      status_message = "no time budget left for set_body_state";
      return a1::WriteOutcome::UNAVAILABLE;
    }

    timing.request_ros_valid = clock_.rosTimeValid();
    timing.request_ros_sec = timing.request_ros_valid ? clock_.rosNowSec() : 0.0;
    timing.request_steady_sec = clock_.steadyNowSec();

    auto req = std::make_shared<SetBodyState::Request>();
    req->state.name = body_name_;
    req->state.pose.header.frame_id = "world";
    req->state.pose.pose.position.x = c.x;
    req->state.pose.pose.position.y = c.y;
    req->state.pose.pose.position.z = c.z;
    req->state.pose.pose.orientation.x = c.quat.x;
    req->state.pose.pose.orientation.y = c.quat.y;
    req->state.pose.pose.orientation.z = c.quat.z;
    req->state.pose.pose.orientation.w = c.quat.w;
    req->state.twist.header.frame_id = "world";
    req->state.twist.twist.linear.x = 0.0;
    req->state.twist.twist.linear.y = 0.0;
    req->state.twist.twist.linear.z = 0.0;
    req->state.twist.twist.angular.x = 0.0;
    req->state.twist.twist.angular.y = 0.0;
    req->state.twist.twist.angular.z = 0.0;
    req->set_pose = true;
    req->set_twist = true;   // explicitly zero the twist so no velocity carries over
    req->set_mass = false;
    req->reset_qpos = false;  // reset_qpos would override set_pose
    req->admin_hash = admin_hash_;

    auto future = set_client_->async_send_request(req);
    if (future.wait_for(std::chrono::duration<double>(timeout)) !=
        std::future_status::ready) {
      timing.response_steady_sec = clock_.steadyNowSec();
      timing.response_ros_valid = clock_.rosTimeValid();
      timing.response_ros_sec = timing.response_ros_valid ? clock_.rosNowSec() : 0.0;
      status_message = "set_body_state call timed out; server side may still apply it";
      return a1::WriteOutcome::TIMEOUT;
    }

    auto resp = future.get();
    timing.response_steady_sec = clock_.steadyNowSec();
    timing.response_ros_valid = clock_.rosTimeValid();
    timing.response_ros_sec = timing.response_ros_valid ? clock_.rosNowSec() : 0.0;
    status_message = resp->status_message;
    write_response_success_ = resp->success;
    write_response_success_valid_ = true;
    return resp->success ? a1::WriteOutcome::SUCCESS : a1::WriteOutcome::REJECTED;
  }

  bool writeResponseSuccess() const { return write_response_success_; }
  bool writeResponseSuccessValid() const { return write_response_success_valid_; }

 private:
  RosClock& clock_;
  std::string body_name_;
  std::string admin_hash_ {};  // empty is accepted unless eval_mode is on
  double call_timeout_ {2.0};

  std::string set_service_name_;
  std::string get_service_name_;
  std::string sim_info_service_name_;

  rclcpp::Client<SetBodyState>::SharedPtr set_client_;
  rclcpp::Client<GetBodyState>::SharedPtr get_client_;
  rclcpp::Client<GetSimInfo>::SharedPtr sim_info_client_;

  bool write_response_success_ {false};
  bool write_response_success_valid_ {false};
};

// Declares every parameter this node reads, mirroring
// config/a1_scene_sampling.yaml. Declaring them up front means a config file
// entry is never silently ignored.
void declareA1Parameters(const rclcpp::Node::SharedPtr& node)
{
  // use_sim_time may already be declared by rclcpp's TimeSource; it must be
  // supplied through the parameter file or an override so that it takes effect
  // before the clock is configured.
  if (!node->has_parameter("use_sim_time")) {
    node->declare_parameter("use_sim_time", false);
  }
  node->declare_parameter("algorithm_version", std::string(""));
  node->declare_parameter("sampling_range_version", std::string(""));
  node->declare_parameter("range_is_reachability_verified", false);
  node->declare_parameter("seed", 0);
  node->declare_parameter("x_min", 0.0);
  node->declare_parameter("x_max", 0.0);
  node->declare_parameter("y_min", 0.0);
  node->declare_parameter("y_max", 0.0);
  node->declare_parameter("yaw_deg_min", 0.0);
  node->declare_parameter("yaw_deg_max", 0.0);
  node->declare_parameter("roll_deg", 0.0);
  node->declare_parameter("pitch_deg", 0.0);
  node->declare_parameter("rod_half_length", 0.0);
  node->declare_parameter("rod_half_width", 0.0);
  node->declare_parameter("rod_half_height", 0.0);
  node->declare_parameter("support_surface_z", 0.0);
  node->declare_parameter("apply_clearance_m", 0.0);
  node->declare_parameter("max_total_candidates", 0);
  node->declare_parameter("max_write_attempts_per_run", 1);
  node->declare_parameter("poll_rate_hz", 50.0);
  node->declare_parameter("min_sim_window_sec", 0.5);
  node->declare_parameter("max_window_pos_tol", 0.0005);
  node->declare_parameter("max_window_angle_tol_deg", 0.2);
  node->declare_parameter("max_twist_linear_tol", 0.01);
  node->declare_parameter("max_twist_angular_tol", 0.05);
  node->declare_parameter("resting_z_tol", 0.003);
  node->declare_parameter("resting_tilt_tol_deg", 2.0);
  node->declare_parameter("min_clock_advance_ratio", 0.5);
  node->declare_parameter("stability_timeout_sec", 5.0);
  node->declare_parameter("set_body_state_service", std::string("/set_body_state"));
  node->declare_parameter("get_body_state_service", std::string("/get_body_state"));
  node->declare_parameter("get_sim_info_service", std::string("/get_sim_info"));
  node->declare_parameter("body_name", std::string("obj_rod_01"));
  node->declare_parameter("service_wait_timeout_sec", 10.0);
  node->declare_parameter("service_call_timeout_sec", 2.0);
  node->declare_parameter("sim_info_check_every", 10);
  node->declare_parameter("max_window_samples", 200);
  node->declare_parameter("run_manifest_dir", std::string("experiment_results/a1_runs"));
}

// --- Parameter plumbing ------------------------------------------------------

double getD(const rclcpp::Node::SharedPtr& n, const std::string& name)
{
  return n->get_parameter(name).as_double();
}
int64_t getI(const rclcpp::Node::SharedPtr& n, const std::string& name)
{
  return n->get_parameter(name).as_int();
}
bool getB(const rclcpp::Node::SharedPtr& n, const std::string& name)
{
  return n->get_parameter(name).as_bool();
}
std::string getS(const rclcpp::Node::SharedPtr& n, const std::string& name)
{
  return n->get_parameter(name).as_string();
}

}  // namespace

int main(int argc, char** argv)
{
  Args args;
  try {
    args = parseArgs(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "argument error: " << e.what() << "\n";
    usage();
    return 3;
  }
  if (args.explicit_fields != 0 && args.explicit_fields != 3) {
    std::cerr << "error: --x, --y and --yaw-deg must be supplied together\n";
    return 3;
  }

  // Hand rclcpp only its own arguments (everything from --ros-args onward), so
  // our flags are never interpreted as ROS arguments or remappings.
  std::vector<std::string> ros_arg_storage;
  ros_arg_storage.reserve(static_cast<std::size_t>(argc - args.ros_args_index + 1));
  ros_arg_storage.emplace_back(argv[0]);
  for (int i = args.ros_args_index; i < argc; ++i) {
    ros_arg_storage.emplace_back(argv[i]);
  }
  std::vector<char*> ros_argv;
  ros_argv.reserve(ros_arg_storage.size());
  for (auto& s : ros_arg_storage) {
    ros_argv.push_back(s.data());
  }
  int ros_argc = static_cast<int>(ros_argv.size());
  rclcpp::init(ros_argc, ros_argv.data());

  auto node = std::make_shared<rclcpp::Node>("a1_scene_sampling");

  // Installed after rclcpp::init so it takes precedence: an interrupted run
  // must still write a manifest rather than exit silently.
  std::signal(SIGINT, handleAbortSignal);
  std::signal(SIGTERM, handleAbortSignal);

  declareA1Parameters(node);
  const bool use_sim_time = node->get_parameter("use_sim_time").as_bool();

  // --- Build the pure-logic configuration -----------------------------------
  a1::SamplingConfig cfg;
  cfg.algorithm_version = getS(node, "algorithm_version");
  cfg.sampling_range_version = getS(node, "sampling_range_version");
  cfg.range_is_reachability_verified = getB(node, "range_is_reachability_verified");
  cfg.seed_raw = args.seed_given ? args.seed : getI(node, "seed");
  cfg.x_min = getD(node, "x_min");
  cfg.x_max = getD(node, "x_max");
  cfg.y_min = getD(node, "y_min");
  cfg.y_max = getD(node, "y_max");
  cfg.yaw_deg_min = getD(node, "yaw_deg_min");
  cfg.yaw_deg_max = getD(node, "yaw_deg_max");
  cfg.roll_deg = getD(node, "roll_deg");
  cfg.pitch_deg = getD(node, "pitch_deg");
  cfg.rod_half_length = getD(node, "rod_half_length");
  cfg.rod_half_width = getD(node, "rod_half_width");
  cfg.rod_half_height = getD(node, "rod_half_height");
  cfg.support_surface_z = getD(node, "support_surface_z");
  cfg.apply_clearance_m = getD(node, "apply_clearance_m");
  cfg.max_total_candidates = static_cast<int32_t>(getI(node, "max_total_candidates"));

  a1::StabilityCriteria crit;
  crit.min_sim_window_sec = getD(node, "min_sim_window_sec");
  crit.max_window_pos_tol = getD(node, "max_window_pos_tol");
  crit.max_window_angle_tol_deg = getD(node, "max_window_angle_tol_deg");
  crit.max_twist_linear_tol = getD(node, "max_twist_linear_tol");
  crit.max_twist_angular_tol = getD(node, "max_twist_angular_tol");
  crit.resting_z_tol = getD(node, "resting_z_tol");
  crit.resting_tilt_tol_deg = getD(node, "resting_tilt_tol_deg");
  crit.min_clock_advance_ratio = getD(node, "min_clock_advance_ratio");

  a1::ApplyPolicy policy;
  policy.abort_flag = &g_abort_requested;
  policy.poll_rate_hz = getD(node, "poll_rate_hz");
  policy.service_wait_timeout_sec = getD(node, "service_wait_timeout_sec");
  policy.stability_timeout_sec = getD(node, "stability_timeout_sec");
  policy.service_call_timeout_sec = getD(node, "service_call_timeout_sec");
  policy.max_write_attempts_per_run =
      static_cast<int32_t>(getI(node, "max_write_attempts_per_run"));
  policy.sim_info_check_every = static_cast<int32_t>(getI(node, "sim_info_check_every"));
  policy.max_window_samples = static_cast<int32_t>(getI(node, "max_window_samples"));

  const std::string set_service = getS(node, "set_body_state_service");
  const std::string get_service = getS(node, "get_body_state_service");
  const std::string sim_info_service = getS(node, "get_sim_info_service");
  const std::string body_name = getS(node, "body_name");

  const std::string manifest_root =
      args.manifest_dir.empty() ? getS(node, "run_manifest_dir") : args.manifest_dir;
  std::string run_id = args.run_id;
  if (run_id.empty()) {
    std::ostringstream os;
    const std::time_t t = std::time(nullptr);
    std::tm tm {};
    gmtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    os << "a1_" << buf;
    run_id = os.str();
  }
  const a1::ValidationResult run_id_check = a1::validateRunId(run_id);
  if (!run_id_check.ok) {
    std::cout << "A1_RESULT {\"scene_state\": \"NOT_RUN\", \"outcome_code\": \""
              << run_id_check.reason_code << "\", \"detail\": \"" << jsonEscape(run_id_check.detail)
              << "\", \"manifest\": null}\n";
    std::cerr << "run_id rejected: " << run_id_check.reason_code << " (" << run_id_check.detail
              << ")\n";
    rclcpp::shutdown();
    return 3;
  }

  const std::filesystem::path run_dir = std::filesystem::path(manifest_root) / run_id;

  // Reserve the run directory EXCLUSIVELY before anything else can act. This
  // happens before the first service call, so a run can never truncate or
  // overwrite the logs and raw data of an earlier run, whether it is started
  // through the script or invoked directly.
  {
    std::error_code dir_ec;
    std::filesystem::create_directories(manifest_root, dir_ec);
    if (!std::filesystem::create_directory(run_dir, dir_ec)) {
      // The directory already existed. That is acceptable only when the runner
      // script reserved this exact fresh directory for us; anything else (an
      // earlier attempt, a previous run, a direct invocation) is refused so no
      // existing record can be truncated or overwritten.
      const bool script_reserved = std::filesystem::exists(run_dir / kScriptReservationMarker);
      const bool already_written = std::filesystem::exists(run_dir / "manifest.json");
      if (!script_reserved || already_written) {
        std::cout << "A1_RESULT {\"scene_state\": \"NOT_RUN\", \"outcome_code\": "
                     "\"E_RUN_DIR_EXISTS\", \"detail\": \"run directory already exists or could "
                     "not be reserved\", \"manifest\": null}\n";
        std::cerr << "refusing to reuse run directory: " << run_dir << "\n";
        rclcpp::shutdown();
        return 3;
      }
    }
  }
  // The note about the controller inventory is fixed for the run; it must not
  // be appended once per manifest build.
  args.notes.push_back(
      "controllers not queried by this node (NOT_RUN); the runner script captures "
      "`ros2 control list_controllers` into the run directory");

  // --- Manifest assembly ----------------------------------------------------
  std::vector<std::string> notes = args.notes;
  std::string scene_state = "NOT_RUN";
  std::string outcome_code = "NOT_RUN";
  std::string detail;
  // "sampled" for the normal path, "explicit_override" when a known test pose
  // was supplied via --x/--y/--yaw-deg.
  const bool explicit_pose = args.explicit_fields > 0;
  std::string pose_source = explicit_pose ? "explicit_override" : "sampled";
  const std::string history_csv_name = "settle_samples.csv";

  // A1 P1b truth publication state (declared before the manifest builder, which
  // records it).
  bool truth_published = false;
  std::string truth_publish_error;
  rclcpp::Publisher<dual_arm_a1_msgs::msg::RodPoseTruth>::SharedPtr truth_pub;

  auto buildManifest = [&](const a1::RunResult* run,
                           const RosTransport* transport,
                           bool write_success_valid,
                           bool write_success,
                           const std::string& wall_start,
                           const std::string& wall_end,
                           const std::vector<a1::Candidate>& preview) {
    std::ostringstream os;
    os << "{\n";
    os << "  \"run_id\": " << jstr(run_id) << ",\n";
    os << "  \"wall_start_utc\": " << jstr(wall_start) << ",\n";
    os << "  \"wall_end_utc\": " << jstr(wall_end) << ",\n";
    os << "  \"scene_state\": " << jstr(scene_state) << ",\n";
    os << "  \"outcome_code\": " << jstr(outcome_code) << ",\n";
    os << "  \"detail\": " << jstr(detail) << ",\n";

    os << "  \"ros_session\": {\"ros_domain_id\": " << jnullable(args.domain_id)
       << ", \"ros_localhost_only\": " << jnullable(args.localhost_only)
       << ", \"use_sim_time\": " << jbool(use_sim_time) << "},\n";

    os << "  \"config\": {\n";
    os << "    \"canonical_string\": " << jstr(cfg.canonicalString()) << ",\n";
    os << "    \"fnv1a64\": " << jstr(checksumHex(cfg.checksum())) << ",\n";
    os << "    \"note\": \"change-detection checksum over the sampling config, "
          "not cryptographic\"\n";
    os << "  },\n";
    // The effective values actually used, so a run is interpretable without
    // recomputing anything from the checksum.
    os << "  \"effective_parameters\": {\n";
    os << "    \"set_body_state_service\": " << jstr(set_service) << ",\n";
    os << "    \"get_body_state_service\": " << jstr(get_service) << ",\n";
    os << "    \"get_sim_info_service\": " << jstr(sim_info_service) << ",\n";
    os << "    \"body_name\": " << jstr(body_name) << ",\n";
    os << "    \"candidate_index\": " << args.candidate_index << ",\n";
    os << "    \"dry_run\": " << jbool(args.dry_run) << ",\n";
    os << "    \"poll_rate_hz\": " << jnum(policy.poll_rate_hz) << ",\n";
    os << "    \"service_wait_timeout_sec\": " << jnum(policy.service_wait_timeout_sec) << ",\n";
    os << "    \"service_call_timeout_sec\": " << jnum(policy.service_call_timeout_sec) << ",\n";
    os << "    \"stability_timeout_sec\": " << jnum(policy.stability_timeout_sec) << ",\n";
    os << "    \"min_sim_window_sec\": " << jnum(crit.min_sim_window_sec) << ",\n";
    os << "    \"max_window_pos_tol\": " << jnum(crit.max_window_pos_tol) << ",\n";
    os << "    \"max_window_angle_tol_deg\": " << jnum(crit.max_window_angle_tol_deg) << ",\n";
    os << "    \"max_twist_linear_tol\": " << jnum(crit.max_twist_linear_tol) << ",\n";
    os << "    \"max_twist_angular_tol\": " << jnum(crit.max_twist_angular_tol) << ",\n";
    os << "    \"resting_z_tol\": " << jnum(crit.resting_z_tol) << ",\n";
    os << "    \"resting_tilt_tol_deg\": " << jnum(crit.resting_tilt_tol_deg) << ",\n";
    os << "    \"min_clock_advance_ratio\": " << jnum(crit.min_clock_advance_ratio) << ",\n";
    os << "    \"sim_info_check_every\": " << policy.sim_info_check_every << ",\n";
    os << "    \"max_window_samples\": " << policy.max_window_samples << ",\n";
    os << "    \"max_write_attempts_per_run\": " << policy.max_write_attempts_per_run << ",\n";
    os << "    \"roll_deg\": " << jnum(cfg.roll_deg) << ",\n";
    os << "    \"pitch_deg\": " << jnum(cfg.pitch_deg) << "\n";
    os << "  },\n";
    os << "  \"requested_seed\": " << cfg.seed_raw << ",\n";
    os << "  \"algorithm_version\": " << jstr(cfg.algorithm_version) << ",\n";
    os << "  \"sampling_range_version\": " << jstr(cfg.sampling_range_version) << ",\n";
    os << "  \"range_is_reachability_verified\": " << jbool(cfg.range_is_reachability_verified)
       << ",\n";
    os << "  \"pose_source\": " << jstr(pose_source) << ",\n";
    os << "  \"rod_center_z_derivation\": {\"support_surface_z\": " << jnum(cfg.support_surface_z)
       << ", \"rod_half_height\": " << jnum(cfg.rod_half_height)
       << ", \"apply_clearance_m\": " << jnum(cfg.apply_clearance_m)
       << ", \"rod_center_z\": " << jnum(a1::rodCenterZ(cfg)) << "},\n";

    os << "  \"git_commit\": " << jnullable(args.git_commit) << ",\n";
    os << "  \"git_dirty\": ";
    if (args.git_dirty == "1") {
      os << "true";
    } else if (args.git_dirty == "0") {
      os << "false";
    } else {
      os << "null";
    }
    os << ",\n";

    // Model checksums (null when a file is absent).
    os << "  \"model_checksums\": {";
    bool first = true;
    auto emit_checksum = [&](const std::string& label, const std::string& path) {
      uint64_t sum = 0;
      const bool ok = !path.empty() && fileChecksum(path, sum);
      if (!first) {
        os << ", ";
      }
      first = false;
      os << jstr(label) << ": " << (ok ? jstr(checksumHex(sum)) : std::string("null"));
    };
    const std::string models_dir = args.models_dir;
    emit_checksum("dual_scene.xml",
                  models_dir.empty() ? "" : models_dir + "/dual_scene.xml");
    emit_checksum("mj_dual.xml", models_dir.empty() ? "" : models_dir + "/mj_dual.xml");
    emit_checksum("objects.xml", models_dir.empty() ? "" : models_dir + "/objects.xml");
    emit_checksum("dual_panda_generated.srdf", args.srdf_path);
    os << "},\n";

    // Compliance environment actually in effect for this process.
    os << "  \"compliance_env\": {";
    const char* names[] = {"MJ_INTERNAL_COMPLIANCE_ENABLED",
                           "MJ_INTERNAL_COMPLIANCE_STRESS_THRESHOLD",
                           "MJ_INTERNAL_COMPLIANCE_K_GAIN",
                           "MJ_INTERNAL_COMPLIANCE_D_GAIN",
                           "MJ_INTERNAL_COMPLIANCE_MIN_K",
                           "MJ_INTERNAL_COMPLIANCE_MAX_D"};
    first = true;
    for (const char* n : names) {
      if (!first) {
        os << ", ";
      }
      first = false;
      const std::string v = envOrEmpty(n);
      os << jstr(n) << ": " << (v.empty() ? std::string("null") : jstr(v));
    }
    os << "},\n";

    os << "  \"controllers\": null,\n";

    if (run == nullptr) {
      // Dry run / configuration failure: preview candidates only.
      os << "  \"candidate_preview\": [";
      for (std::size_t i = 0; i < preview.size(); ++i) {
        const auto& c = preview[i];
        if (i > 0) {
          os << ", ";
        }
        os << "{\"index\": " << c.index << ", \"x\": " << jnum(c.x) << ", \"y\": " << jnum(c.y)
           << ", \"z\": " << jnum(c.z) << ", \"yaw_rad\": " << jnum(c.yaw_rad) << "}";
      }
      os << "],\n";
      os << "  \"write\": null,\n";
      os << "  \"readback\": null,\n";
      os << "  \"readback_pose\": null,\n";
      os << "  \"settle\": null,\n";
      os << "  \"sim_info_before\": null,\n";
      os << "  \"sim_info_after\": null,\n";
      os << "  \"sim_info_final\": null,\n";
      os << "  \"health_confirmed\": null,\n";
      os << "  \"health_detail\": null,\n";
    } else {
      const auto& r = *run;
      os << "  \"candidate\": {\n";
      os << "    \"index\": " << r.candidate.index << ",\n";
      os << "    \"generated\": " << jbool(r.candidate_generated) << ",\n";
      os << "    \"pose_candidate\": {\"x\": " << jnum(r.candidate.x) << ", \"y\": "
         << jnum(r.candidate.y) << ", \"z\": " << jnum(r.candidate.z)
         << ", \"yaw_rad\": " << jnum(r.candidate.yaw_rad) << "},\n";
      os << "    \"quat_candidate\": {\"x\": " << jnum(r.candidate.quat.x) << ", \"y\": "
         << jnum(r.candidate.quat.y) << ", \"z\": " << jnum(r.candidate.quat.z)
         << ", \"w\": " << jnum(r.candidate.quat.w) << "}\n";
      os << "  },\n";

      os << "  \"write\": {\n";
      os << "    \"attempted\": " << jbool(r.write_attempted) << ",\n";
      os << "    \"outcome\": " << jstr(a1::toString(r.write_outcome)) << ",\n";
      os << "    \"request_ros_time\": "
         << (r.write_timing.request_ros_valid ? jnum(r.write_timing.request_ros_sec)
                                              : std::string("null"))
         << ",\n";
      os << "    \"request_steady\": " << jnum(r.write_timing.request_steady_sec) << ",\n";
      os << "    \"response_ros_time\": "
         << (r.write_timing.response_ros_valid ? jnum(r.write_timing.response_ros_sec)
                                               : std::string("null"))
         << ",\n";
      os << "    \"response_steady\": " << jnum(r.write_timing.response_steady_sec) << ",\n";
      os << "    \"response_success\": "
         << (write_success_valid ? jbool(write_success) : std::string("null")) << ",\n";
      os << "    \"response_status_message\": " << jstr(r.write_status_message) << "\n";
      os << "  },\n";

      os << "  \"sim_info_before\": {\"ok\": " << jbool(r.sim_info_before.ok)
         << ", \"model_valid\": " << jbool(r.sim_info_before.model_valid)
         << ", \"model_path\": " << jstr(r.sim_info_before.model_path)
         << ", \"load_count\": " << r.sim_info_before.load_count
         << ", \"paused\": " << jbool(r.sim_info_before.paused) << "},\n";
      os << "  \"sim_info_after\": ";
      if (r.sim_info_after.ok) {
        os << "{\"ok\": true, \"model_valid\": " << jbool(r.sim_info_after.model_valid)
           << ", \"model_path\": " << jstr(r.sim_info_after.model_path)
           << ", \"load_count\": " << r.sim_info_after.load_count
           << ", \"paused\": " << jbool(r.sim_info_after.paused) << "},\n";
      } else {
        os << "null,\n";
      }

      os << "  \"settle\": {\"sim_window_sec\": " << jnum(r.stability.sim_window_sec)
         << ", \"steady_window_sec\": " << jnum(r.stability.steady_window_sec)
         << ", \"clock_advance_ratio\": " << jnum(r.stability.clock_advance_ratio)
         << ", \"max_window_pos_delta\": " << jnum(r.stability.max_window_pos_delta)
         << ", \"max_window_angle_delta_deg\": " << jnum(r.stability.max_window_angle_delta_deg)
         << ", \"max_twist_lin\": " << jnum(r.stability.max_twist_lin)
         << ", \"max_twist_ang\": " << jnum(r.stability.max_twist_ang)
         << ", \"converged\": "
         << jbool(r.stability.outcome == a1::StabilityOutcome::STABLE)
         << ", \"reason_code\": " << jstr(r.stability.reason_code)
         << ", \"detail\": " << jstr(r.stability.detail) << "},\n";

      os << "  \"settle_reason_trace\": [";
      for (std::size_t i = 0; i < r.settle_trace.size(); ++i) {
        if (i > 0) {
          os << ", ";
        }
        os << "{\"poll\": " << r.settle_trace[i].poll
           << ", \"reason_code\": " << jstr(r.settle_trace[i].reason_code) << "}";
      }
      os << "],\n";

      os << "  \"readback\": {\"count\": " << r.samples.size()
         << ", \"poll_count\": " << r.history.size()
         << ", \"history_csv\": " << jstr(history_csv_name)
         << ", \"rod_mass_kg\": " << jnum(r.baseline_state.mass)
         << ", \"baseline_pose\": {\"x\": " << jnum(r.baseline_state.pos.x)
         << ", \"y\": " << jnum(r.baseline_state.pos.y)
         << ", \"z\": " << jnum(r.baseline_state.pos.z) << "}"
         << ", \"service_pose_stamp_is_zero\": "
         << jbool(r.readback.ok ? r.readback.pose_stamp_is_zero : true) << ",\n";
      os << "    \"samples\": [";
      for (std::size_t i = 0; i < r.samples.size(); ++i) {
        const auto& s = r.samples[i];
        if (i > 0) {
          os << ",";
        }
        os << "\n      {\"ros_time\": "
           << (s.ros_time_valid ? jnum(s.ros_time_sec) : std::string("null"))
           << ", \"steady\": " << jnum(s.steady_time_sec) << ", \"pos\": {\"x\": " << jnum(s.pos.x)
           << ", \"y\": " << jnum(s.pos.y) << ", \"z\": " << jnum(s.pos.z)
           << "}, \"quat\": {\"x\": " << jnum(s.quat.x) << ", \"y\": " << jnum(s.quat.y)
           << ", \"z\": " << jnum(s.quat.z) << ", \"w\": " << jnum(s.quat.w)
           << "}, \"twist_lin_norm\": " << jnum(s.twist_lin_norm)
           << ", \"twist_ang_norm\": " << jnum(s.twist_ang_norm)
           << ", \"twist_valid\": " << jbool(s.twist_valid) << "}";
      }
      os << "\n    ]},\n";

      os << "  \"readback_pose\": ";
      if (r.readback.ok) {
        os << "{\"pos\": {\"x\": " << jnum(r.readback.pos.x) << ", \"y\": "
           << jnum(r.readback.pos.y) << ", \"z\": " << jnum(r.readback.pos.z)
           << "}, \"quat\": {\"x\": "
           << jnum(r.readback.quat.x) << ", \"y\": " << jnum(r.readback.quat.y)
           << ", \"z\": " << jnum(r.readback.quat.z) << ", \"w\": " << jnum(r.readback.quat.w)
           << "}},\n";
      } else {
        os << "null,\n";
      }

      os << "  \"sim_info_final\": ";
      if (r.sim_info_final.ok) {
        os << "{\"ok\": true, \"model_valid\": " << jbool(r.sim_info_final.model_valid)
           << ", \"model_path\": " << jstr(r.sim_info_final.model_path)
           << ", \"load_count\": " << r.sim_info_final.load_count
           << ", \"paused\": " << jbool(r.sim_info_final.paused) << "},\n";
      } else {
        os << "null,\n";
      }
      os << "  \"health_confirmed\": " << jbool(r.health_confirmed) << ",\n";
      os << "  \"health_detail\": " << jstr(r.health_detail) << ",\n";
      os << "  \"steady_clock_span_sec\": " << jnum(r.steady_span_sec) << ",\n";
      os << "  \"ros_time_span_sec\": "
         << (r.ros_span_valid ? jnum(r.ros_span_sec) : std::string("null")) << ",\n";
    }

    os << "  \"truth\": ";
    if (run == nullptr || !truth_pub) {
      os << "null,\n";
    } else {
      os << "{\"published\": " << jbool(truth_published) << ", \"topic\": "
         << jstr(args.truth_topic) << ", \"hold_sec\": " << jnum(args.hold_sec)
         << ", \"publish_error\": " << jstr(truth_publish_error) << "},\n";
    }
    os << "  \"p2_acceptance\": null,\n";
    os << "  \"task_outcome\": null,\n";
    os << "  \"placement_error\": null,\n";
    os << "  \"notes\": [";
    for (std::size_t i = 0; i < notes.size(); ++i) {
      if (i > 0) {
        os << ", ";
      }
      os << jstr(notes[i]);
    }
    os << "]\n";
    os << "}\n";
    (void)transport;
    return os.str();
  };

  // Raw per-poll data, written next to the manifest. Keeping this separate
  // stops the JSON from growing with every poll while still preserving the
  // original measurements.
  auto writeHistoryCsv = [&](const std::vector<a1::PoseSample>& history) -> bool {
    if (history.empty()) {
      return true;  // nothing to persist
    }
    // Write to a temporary file first and rename, so a partial write never
    // replaces a complete one.
    const std::filesystem::path target = run_dir / history_csv_name;
    const std::filesystem::path temp = run_dir / (history_csv_name + ".part");
    std::ofstream out(temp);
    if (!out) {
      std::cerr << "cannot write history csv " << temp << "\n";
      return false;
    }
    out << "poll,ros_time,ros_time_valid,steady,pos_x,pos_y,pos_z,"
           "quat_x,quat_y,quat_z,quat_w,twist_lin_norm,twist_ang_norm,twist_valid\n";
    out << std::setprecision(17);
    for (std::size_t i = 0; i < history.size(); ++i) {
      const auto& s = history[i];
      out << i << ',';
      if (s.ros_time_valid) {
        out << s.ros_time_sec;
      } else {
        out << "null";
      }
      out << ',' << (s.ros_time_valid ? 1 : 0) << ',' << s.steady_time_sec << ',' << s.pos.x << ','
          << s.pos.y << ',' << s.pos.z << ',' << s.quat.x << ',' << s.quat.y << ',' << s.quat.z
          << ',' << s.quat.w << ',' << s.twist_lin_norm << ',' << s.twist_ang_norm << ','
          << (s.twist_valid ? 1 : 0) << '\n';
    }
    out.flush();
    if (!out) {
      std::cerr << "history csv write failed; leaving " << temp << " for inspection\n";
      return false;
    }
    out.close();
    std::error_code rename_ec;
    std::filesystem::rename(temp, target, rename_ec);
    if (rename_ec) {
      std::cerr << "history csv rename failed; leaving " << temp << " for inspection\n";
      return false;
    }
    return true;
  };

  // The run directory was reserved up front, so this only ever creates the
  // manifest inside it. The return value is not advisory: a failure to persist
  // evidence must surface as a non-zero exit.
  auto writeManifest = [&](const std::string& text, std::string& path_out) -> bool {
    const std::filesystem::path file = run_dir / "manifest.json";
    if (std::filesystem::exists(file)) {
      std::cerr << "refusing to overwrite existing manifest: " << file << "\n";
      return false;
    }
    std::ofstream out(file);
    if (!out) {
      std::cerr << "cannot write manifest " << file << "\n";
      return false;
    }
    out << text;
    out.flush();
    if (!out) {
      std::cerr << "manifest write failed: " << file << "\n";
      return false;
    }
    path_out = file.string();
    return true;
  };

  // --- A1 P1b truth publication --------------------------------------------
  // Published once, after the pose has settled and been read back. The
  // publisher is latched (transient_local) so a task that starts later still
  // receives it, and the node stays alive for `hold_sec` so that sample remains
  // servable.
  if (args.publish_truth) {
    if (args.truth_topic.empty() || args.truth_topic.front() != '/') {
      std::cerr << "error: --truth-topic must be a non-empty absolute name\n";
      rclcpp::shutdown();
      return 3;
    }
    rclcpp::QoS truth_qos(1);
    truth_qos.transient_local().reliable();
    truth_pub = node->create_publisher<dual_arm_a1_msgs::msg::RodPoseTruth>(args.truth_topic,
                                                                           truth_qos);
    std::cout << "truth publication enabled on " << args.truth_topic
              << " (hold " << args.hold_sec << "s)\n";
  }

  const std::string wall_start = utcNow();
  int rc = 0;

  // --- Validate every configuration value actually used ---------------------
  auto reject = [&](const a1::ValidationResult& v) {
    scene_state = "NOT_RUN";
    outcome_code = v.reason_code;
    detail = v.detail;
    std::cerr << "configuration rejected: " << v.reason_code << " (" << v.detail << ")\n";
    std::string path;
    if (!writeManifest(buildManifest(nullptr, nullptr, false, false, wall_start, utcNow(), {}),
                       path)) {
      std::cout << "A1_RESULT {\"scene_state\": \"NOT_RUN\", \"outcome_code\": "
                   "\"E_MANIFEST_WRITE_FAILED\", \"evidence_written\": false, "
                   "\"manifest\": null}\n";
      rclcpp::shutdown();
      return 5;
    }
    std::cerr << "manifest: " << path << "\n";
    std::cout << "A1_RESULT {\"scene_state\": \"" << scene_state << "\", \"outcome_code\": \""
              << outcome_code << "\", \"evidence_written\": true, \"manifest\": \"" << path
              << "\"}\n";
    rclcpp::shutdown();
    return 2;
  };

  const a1::ValidationResult config_check = a1::validateSamplingConfig(cfg);
  if (!config_check.ok) {
    return reject(config_check);
  }
  const a1::ValidationResult policy_check = a1::validateApplyParameters(policy, crit);
  if (!policy_check.ok) {
    return reject(policy_check);
  }
  if (set_service.empty() || set_service.front() != '/') {
    return reject(a1::ValidationResult{false, "E_SERVICE_NAME_INVALID",
                                       "set_body_state_service must be a non-empty absolute name"});
  }
  if (get_service.empty() || get_service.front() != '/') {
    return reject(a1::ValidationResult{false, "E_SERVICE_NAME_INVALID",
                                       "get_body_state_service must be a non-empty absolute name"});
  }
  if (sim_info_service.empty() || sim_info_service.front() != '/') {
    return reject(a1::ValidationResult{false, "E_SERVICE_NAME_INVALID",
                                       "get_sim_info_service must be a non-empty absolute name"});
  }
  if (body_name.empty()) {
    return reject(
        a1::ValidationResult{false, "E_BODY_NAME_EMPTY", "body_name must not be empty"});
  }

  const a1::Sampler sampler(cfg);

  // --- Dry run: no service interaction at all -------------------------------
  if (args.dry_run) {
    std::vector<a1::Candidate> preview =
        sampler.firstN(std::min<int32_t>(5, cfg.max_total_candidates));
    scene_state = "NOT_RUN";
    outcome_code = "DRY_RUN";
    detail = "dry run: no service call, no simulation interaction";
    std::cout << "dry-run candidates (algorithm=" << cfg.algorithm_version
              << ", range=" << cfg.sampling_range_version
              << ", seed=" << cfg.seed_raw << "):\n";
    for (std::size_t i = 0; i < preview.size(); ++i) {
      const auto& c = preview[i];
      std::cout << "  [" << c.index << "] x=" << c.x << " y=" << c.y << " z=" << c.z
                << " yaw_rad=" << c.yaw_rad << "\n";
    }
    if (args.candidate_index >= 0 && args.candidate_index < cfg.max_total_candidates) {
      const auto c = sampler.candidate(args.candidate_index);
      std::cout << "selected candidate " << args.candidate_index << ": x=" << c.x << " y=" << c.y
                << " z=" << c.z << " yaw_rad=" << c.yaw_rad << " quat=(" << c.quat.x << ", "
                << c.quat.y << ", " << c.quat.z << ", " << c.quat.w << ")\n";
    }
    std::string path;
    if (!writeManifest(buildManifest(nullptr, nullptr, false, false, wall_start, utcNow(), preview),
                       path)) {
      std::cout << "A1_RESULT {\"scene_state\": \"NOT_RUN\", \"outcome_code\": "
                   "\"E_MANIFEST_WRITE_FAILED\", \"evidence_written\": false, "
                   "\"manifest\": null}\n";
      rclcpp::shutdown();
      return 5;
    }
    std::cout << "manifest: " << path << "\n";
    std::cout << "A1_RESULT {\"scene_state\": \"" << scene_state << "\", \"outcome_code\": \""
              << outcome_code << "\", \"evidence_written\": true, \"manifest\": \"" << path
              << "\"}\n";
    rclcpp::shutdown();
    return 0;
  }

  // --- Require simulated time ----------------------------------------------
  if (!use_sim_time) {
    scene_state = "NOT_RUN";
    outcome_code = "E_REQUIRES_SIM_TIME";
    detail = "the settling criterion needs simulated time; enable use_sim_time";
    std::cerr << detail << "\n";
    std::string path;
    writeManifest(buildManifest(nullptr, nullptr, false, false, wall_start, utcNow(), {}), path);
    std::cerr << "manifest: " << path << "\n";
    rclcpp::shutdown();
    return 2;
  }

  RosClock clock(node);
  RosTransport transport(node, clock, set_service, get_service, sim_info_service, body_name,
                        policy.service_call_timeout_sec);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  a1::RunResult run;
  auto finish = [&](const std::string& state,
                    const std::string& code,
                    const std::string& why,
                    int exit_code) {
    scene_state = state;
    outcome_code = code;
    detail = why;

    // Publish the settled truth only for a successful preparation, only once,
    // and only from the read-back actually measured this run.
    if (truth_pub && scene_state == "SCENE_READY_UNVALIDATED" && run.readback.ok) {
      if (run.readback.frame_id.empty()) {
        truth_publish_error = "E_TRUTH_FRAME_MISSING: service reported no pose frame";
      } else {
        dual_arm_a1_msgs::msg::RodPoseTruth msg;
        msg.header.stamp = node->now();  // publisher-local time, not a freshness token
        msg.header.frame_id = run.readback.frame_id;
        msg.run_id = run_id;
        msg.pose.position.x = run.readback.pos.x;
        msg.pose.position.y = run.readback.pos.y;
        msg.pose.position.z = run.readback.pos.z;
        msg.pose.orientation.x = run.readback.quat.x;
        msg.pose.orientation.y = run.readback.quat.y;
        msg.pose.orientation.z = run.readback.quat.z;
        msg.pose.orientation.w = run.readback.quat.w;
        // Full box size, derived from the configured half extents.
        msg.rod_size.x = cfg.rod_half_width * 2.0;
        msg.rod_size.y = cfg.rod_half_length * 2.0;
        msg.rod_size.z = cfg.rod_half_height * 2.0;
        msg.mass_kg = run.readback.mass;
        truth_pub->publish(msg);
        truth_published = true;
        std::cout << "truth published: run_id=" << run_id << " frame=" << msg.header.frame_id
                  << " pos=(" << msg.pose.position.x << ", " << msg.pose.position.y << ", "
                  << msg.pose.position.z << ")" << std::endl;
      }
    }

    // "scene action succeeded" and "evidence persisted" are separate outcomes.
    const bool csv_ok = writeHistoryCsv(run.history);
    std::string path;
    const bool manifest_ok =
        writeManifest(buildManifest(&run, &transport, transport.writeResponseSuccessValid(),
                                    transport.writeResponseSuccess(), wall_start, utcNow(), {}),
                      path);
    const bool evidence_ok = csv_ok && manifest_ok;
    std::cout << "scene_state=" << scene_state << " outcome_code=" << outcome_code << "\n";
    if (manifest_ok) {
      std::cout << "manifest: " << path << "\n";
    }
    std::cout << "A1_RESULT {\"scene_state\": \"" << scene_state << "\", \"outcome_code\": \""
              << outcome_code << "\", \"evidence_written\": " << (evidence_ok ? "true" : "false")
              << ", \"truth_published\": " << (truth_published ? "true" : "false")
              << ", \"manifest\": " << (manifest_ok ? "\"" + path + "\"" : "null") << "}\n";

    // Stay alive so the latched truth remains available to a task that starts
    // after this node finished settling.
    if (truth_published && args.hold_sec > 0.0) {
      std::cout << "holding for up to " << args.hold_sec
                << "s so the latched truth can be received" << std::endl;
      const auto hold_deadline = std::chrono::steady_clock::now() +
                                 std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>(args.hold_sec));
      while (rclcpp::ok() && !g_abort_requested.load() &&
             std::chrono::steady_clock::now() < hold_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::cout << "hold finished" << std::endl;
    }

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    if (!evidence_ok) {
      // The pose may well have been applied; what failed is the record of it.
      return 5;
    }
    return exit_code;
  };

  std::string wait_message;
  if (!transport.waitForServices(policy.service_wait_timeout_sec, wait_message)) {
    return finish("SCENE_APPLY_FAILED", "E_SERVICES_UNAVAILABLE", wait_message, 2);
  }

  // The settling criterion depends on simulated time actually flowing. Without
  // an active ROS clock (no /clock received yet) no sample can ever be valid,
  // so fail clearly instead of reporting a clock-related settle failure later.
  if (!node->get_clock()->ros_time_is_active()) {
    return finish("SCENE_APPLY_FAILED", "E_SIM_CLOCK_NOT_ACTIVE",
                  "ROS clock is not active; /clock has not been received from the simulator",
                  2);
  }

  // Model readiness and rod identity are checked before any write.
  const a1::SimInfo info = transport.getSimInfo(policy.service_call_timeout_sec);
  if (!info.ok) {
    return finish("SCENE_APPLY_FAILED", "E_SIM_INFO_UNAVAILABLE", info.status_message, 2);
  }
  if (!info.model_valid) {
    return finish("SCENE_APPLY_FAILED", "E_MODEL_INVALID",
                  "simulation reports model_valid == false (path=" + info.model_path + ")", 2);
  }
  a1::PoseSample probe_timing;
  const a1::BodyState probe = transport.getState(policy.service_call_timeout_sec, probe_timing);
  if (!probe.ok) {
    return finish("SCENE_APPLY_FAILED", "E_BASELINE_READ_FAILED", probe.status_message, 2);
  }
  std::cout << "preconditions ok: model_valid=1 load_count=" << info.load_count
            << " model_path=" << info.model_path << "\n";
  std::cout << "baseline rod pose: (" << probe.pos.x << ", " << probe.pos.y << ", " << probe.pos.z
            << ")\n";

  if (explicit_pose) {
    a1::Candidate explicit_candidate;
    if (!a1::makeExplicitCandidate(cfg, args.candidate_index, args.ex_x, args.ex_y,
                                  args.ex_yaw_deg * M_PI / 180.0, explicit_candidate)) {
      return finish("NOT_RUN", "E_BAD_EXPLICIT_POSE",
                    "explicit pose contains non-finite values", 3);
    }
    run = a1::applyCandidateAndSettle(cfg, crit, policy, explicit_candidate, transport, clock);
  } else {
    run = a1::applyCandidateAndSettle(cfg, crit, policy, args.candidate_index, transport, clock);
  }


  if (run.scene_state == "SCENE_READY_UNVALIDATED") {
    rc = 0;
  } else if (run.scene_state == "INTERRUPTED") {
    rc = 130;
  } else {
    rc = 2;
  }
  return finish(run.scene_state, run.outcome_code, run.detail, rc);
}
