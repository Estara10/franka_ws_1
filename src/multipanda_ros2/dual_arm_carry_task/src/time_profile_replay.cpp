// Offline replay of the constrained time-profile optimisation.
//
// Reads a saved transport sample (long-format JointTrajectoryController points
// plus explicit limits), optimises only the monotone time map s(tau), and
// writes the baseline, raw candidate, selected faithful export and a comparison
// report. `--duration-scale` changes only the target clock, preserving the
// frozen geometry. No ROS, no simulation.

#include "dual_arm_carry_task/time_profile_optimizer.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{
struct SavedSample
{
  std::vector<std::string> joint_names;
  std::vector<tpo::TrajectoryPoint> points;
  std::vector<tpo::JointLimits> limits;
  std::string source = "unknown";
  std::string method;
};

bool readTrajectory(const std::string& path, SavedSample& out, std::string& error)
{
  std::ifstream in(path);
  if (!in)
  {
    error = "cannot open " + path;
    return false;
  }
  std::string line;
  if (!std::getline(in, line))
  {
    error = "empty trajectory file";
    return false;
  }
  struct Row
  {
    double t, q, v, a;
  };
  std::map<int, std::map<std::string, Row>> by_point;
  std::vector<std::string> order;
  while (std::getline(in, line))
  {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string name, idx, t, q, v, a;
    if (!std::getline(ss, name, ',') || !std::getline(ss, idx, ',') || !std::getline(ss, t, ',') ||
        !std::getline(ss, q, ',') || !std::getline(ss, v, ',') || !std::getline(ss, a, ','))
    {
      error = "malformed trajectory row: " + line;
      return false;
    }
    if (std::find(order.begin(), order.end(), name) == order.end()) order.push_back(name);
    by_point[std::stoi(idx)][name] = Row{std::stod(t), std::stod(q), std::stod(v), std::stod(a)};
  }
  if (by_point.size() < 2 || order.empty())
  {
    error = "trajectory needs at least two points and one joint";
    return false;
  }
  out.joint_names = order;
  for (const auto& entry : by_point)
  {
    tpo::TrajectoryPoint p;
    p.q.resize(static_cast<Eigen::Index>(order.size()));
    p.v.resize(static_cast<Eigen::Index>(order.size()));
    p.a.resize(static_cast<Eigen::Index>(order.size()));
    for (std::size_t j = 0; j < order.size(); ++j)
    {
      const auto it = entry.second.find(order[j]);
      if (it == entry.second.end())
      {
        error = "missing joint " + order[j] + " at point " + std::to_string(entry.first);
        return false;
      }
      if (j == 0) p.time = it->second.t;
      p.q[static_cast<Eigen::Index>(j)] = it->second.q;
      p.v[static_cast<Eigen::Index>(j)] = it->second.v;
      p.a[static_cast<Eigen::Index>(j)] = it->second.a;
    }
    out.points.push_back(std::move(p));
  }
  return true;
}

bool setLimitsFromMap(const std::map<std::string, tpo::JointLimits>& limits,
                      SavedSample& out, std::string& error)
{
  if (limits.empty())
  {
    error = "no limits available";
    return false;
  }
  out.limits.clear();
  for (const auto& name : out.joint_names)
  {
    const auto it = limits.find(name);
    if (it == limits.end())
    {
      error = "no limits for joint " + name;
      return false;
    }
    out.limits.push_back(it->second);
  }
  return true;
}

bool readMeta(const std::string& path, SavedSample& out, std::string& error)
{
  std::ifstream in(path);
  if (!in)
  {
    error = "cannot open " + path;
    return false;
  }
  std::map<std::string, tpo::JointLimits> limits;
  std::string line;
  while (std::getline(in, line))
  {
    if (line.rfind("limit,", 0) == 0)
    {
      std::stringstream ss(line);
      std::string tag, name, minp, maxp, minv, maxv, mina, maxa;
      if (!std::getline(ss, tag, ',') || !std::getline(ss, name, ',') ||
          !std::getline(ss, minp, ',') || !std::getline(ss, maxp, ',') ||
          !std::getline(ss, minv, ',') || !std::getline(ss, maxv, ',') ||
          !std::getline(ss, mina, ',') || !std::getline(ss, maxa, ','))
      {
        error = "malformed limit line: " + line;
        return false;
      }
      limits[name] = tpo::JointLimits{std::stod(minp), std::stod(maxp), std::stod(minv),
                                      std::stod(maxv), std::stod(mina), std::stod(maxa)};
    }
    else if (line.rfind("source=", 0) == 0)
    {
      out.source = line.substr(7);
    }
    else if (line.rfind("method=", 0) == 0)
    {
      out.method = line.substr(7);
    }
  }
  return setLimitsFromMap(limits, out, error);
}

bool readLimitsCsv(const std::string& path, SavedSample& out, std::string& error)
{
  std::ifstream in(path);
  if (!in)
  {
    error = "cannot open limit file " + path;
    return false;
  }
  std::string line;
  if (!std::getline(in, line))
  {
    error = "empty limit file";
    return false;
  }
  std::map<std::string, tpo::JointLimits> limits;
  while (std::getline(in, line))
  {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::vector<std::string> columns;
    std::string field;
    while (std::getline(ss, field, ',')) columns.push_back(field);
    if (columns.size() < 7)
    {
      error = "malformed limit row: " + line;
      return false;
    }
    limits[columns[0]] = tpo::JointLimits{std::stod(columns[1]), std::stod(columns[2]),
                                          std::stod(columns[3]), std::stod(columns[4]),
                                          std::stod(columns[5]), std::stod(columns[6])};
  }
  return setLimitsFromMap(limits, out, error);
}

void writeTrajectory(const std::string& path, const std::vector<std::string>& names,
                     const std::vector<tpo::TrajectoryPoint>& points)
{
  std::ofstream out(path);
  out << std::setprecision(17)
      << "joint_name,point_index,time_from_start,position,velocity,acceleration\n";
  for (std::size_t j = 0; j < names.size(); ++j)
  {
    for (std::size_t k = 0; k < points.size(); ++k)
    {
      out << names[j] << ',' << k << ',' << points[k].time << ','
          << points[k].q[static_cast<Eigen::Index>(j)] << ','
          << points[k].v[static_cast<Eigen::Index>(j)] << ','
          << points[k].a[static_cast<Eigen::Index>(j)] << '\n';
    }
  }
}

const char* boolName(bool value) { return value ? "1" : "0"; }
}  // namespace

int main(int argc, char** argv)
{
  std::string run_dir;
  std::string trajectory_path;
  std::string limits_csv_path;
  std::string meta_path;
  std::string out_dir;
  double duration_scale = 1.0;
  int evaluations = 100;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--run-dir" && i + 1 < argc)
      run_dir = argv[++i];
    else if (arg == "--trajectory" && i + 1 < argc)
      trajectory_path = argv[++i];
    else if (arg == "--limits-csv" && i + 1 < argc)
      limits_csv_path = argv[++i];
    else if (arg == "--meta" && i + 1 < argc)
      meta_path = argv[++i];
    else if (arg == "--out-dir" && i + 1 < argc)
      out_dir = argv[++i];
    else if (arg == "--max-evaluations" && i + 1 < argc)
      evaluations = std::stoi(argv[++i]);
    else if (arg == "--duration-scale" && i + 1 < argc)
      duration_scale = std::stod(argv[++i]);
    else
    {
      std::cerr
          << "usage: time_profile_replay [--run-dir <dir>] [--trajectory <csv>] "
             "[--limits-csv <csv>] [--meta <txt>] [--out-dir <dir>] "
             "[--duration-scale S] [--max-evaluations N]\n";
      return 2;
    }
  }
  if (trajectory_path.empty())
  {
    if (run_dir.empty())
    {
      std::cerr << "error: --run-dir or --trajectory is required\n";
      return 2;
    }
    trajectory_path = run_dir + "/transport_baseline_trajectory.csv";
  }
  if (out_dir.empty()) out_dir = run_dir.empty() ? "." : run_dir;

  SavedSample sample;
  std::string error;
  if (!readTrajectory(trajectory_path, sample, error))
  {
    std::cerr << "error: " << error << "\n";
    return 1;
  }
  if (!limits_csv_path.empty())
  {
    if (!readLimitsCsv(limits_csv_path, sample, error))
    {
      std::cerr << "error: " << error << "\n";
      return 1;
    }
  }
  else
  {
    if (meta_path.empty() && !run_dir.empty())
      meta_path = run_dir + "/transport_sample_meta.txt";
    if (meta_path.empty())
    {
      std::cerr << "error: --limits-csv or --meta is required\n";
      return 2;
    }
    if (!readMeta(meta_path, sample, error))
    {
      std::cerr << "error: " << error << "\n";
      return 1;
    }
  }
  if (sample.source == "unknown") sample.source = trajectory_path;

  tpo::ContinuousPath path;
  if (!path.build(sample.joint_names, sample.points, error))
  {
    std::cerr << "error: " << error << "\n";
    return 1;
  }

  if (!(duration_scale > 0.0) || !std::isfinite(duration_scale))
  {
    std::cerr << "error: --duration-scale must be positive and finite\n";
    return 2;
  }
  const double target_duration = path.duration() * duration_scale;

  tpo::TimeProfileOptions options;
  options.max_evaluations = evaluations;
  const auto solve_start = std::chrono::steady_clock::now();
  const auto result = tpo::optimizeTimeProfile(path, sample.limits, target_duration, options);
  const double solve_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - solve_start).count();
  if (!result.ok)
  {
    std::cerr << "error: " << result.status << ": " << result.detail << "\n";
    return 1;
  }

  writeTrajectory(out_dir + "/time_profile_candidate.csv", sample.joint_names,
                  result.candidate_export);
  writeTrajectory(out_dir + "/time_profile_baseline.csv", sample.joint_names,
                  result.baseline_export);
  writeTrajectory(out_dir + "/time_profile_selected.csv", sample.joint_names,
                  result.selected_export);

  std::ofstream report(out_dir + "/time_profile_report.txt");
  report << std::setprecision(10)
         << "sample=" << trajectory_path << "\n"
         << "source=" << sample.source << "\n"
         << "method=" << sample.method << "\n"
         << "joints=" << sample.joint_names.size() << "\n"
         << "points=" << sample.points.size() << "\n"
         << "duration=" << path.duration() << "\n"
         << "duration_scale=" << duration_scale << "\n"
         << "target_duration_s=" << target_duration << "\n"
         << "degree=" << options.degree << "\n"
         << "parameters=" << result.parameters.size() << "\n"
         << "max_evaluations=" << options.max_evaluations << "\n"
         << "evaluations=" << result.evaluations << "\n"
         << "solve_sec=" << solve_sec << "\n"
         << "accepted=" << boolName(result.accepted) << "\n"
         << "selected_plan=" << (result.accepted ? "candidate" : "baseline") << "\n"
         << "fallback_reason=" << (result.fallback_reason.empty() ? "none" : result.fallback_reason)
         << "\n"
         << "constraints_ok=" << boolName(result.constraints_ok) << "\n"
         << "rms_improved=" << boolName(result.rms_improved) << "\n"
         << "peak_ok=" << boolName(result.peak_ok) << "\n"
         << "export_fidelity_ok=" << boolName(result.export_fidelity_ok) << "\n"
         << "export_spacing_used=" << result.export_spacing_used << "\n"
         << "baseline_export_deviation_rad=" << result.baseline_export_deviation_rad << "\n"
         << "candidate_export_deviation_rad=" << result.max_export_deviation_rad << "\n"
         << "parameter_values=";
  for (Eigen::Index i = 0; i < result.parameters.size(); ++i)
    report << (i == 0 ? "" : ",") << result.parameters[i];
  report << "\n";
  auto row = [&](const char* name, const tpo::TrajectoryMetrics& baseline,
                 const tpo::TrajectoryMetrics& candidate) {
    report << name << ",jerk_rms," << baseline.jerk_rms << "," << candidate.jerk_rms << ","
           << (baseline.jerk_rms > 0.0
                   ? 100.0 * (candidate.jerk_rms - baseline.jerk_rms) / baseline.jerk_rms
                   : 0.0)
           << "\n";
    report << name << ",jerk_max," << baseline.jerk_max << "," << candidate.jerk_max << ","
           << (baseline.jerk_max > 0.0
                   ? 100.0 * (candidate.jerk_max - baseline.jerk_max) / baseline.jerk_max
                   : 0.0)
           << "\n";
    report << name << ",acc_rms," << baseline.acceleration_rms << ","
           << candidate.acceleration_rms << ","
           << (baseline.acceleration_rms > 0.0
                   ? 100.0 * (candidate.acceleration_rms - baseline.acceleration_rms) /
                         baseline.acceleration_rms
                   : 0.0)
           << "\n";
    report << name << ",max_vel_ratio," << baseline.max_velocity_ratio << ","
           << candidate.max_velocity_ratio << ",0\n";
    report << name << ",max_acc_ratio," << baseline.max_acceleration_ratio << ","
           << candidate.max_acceleration_ratio << ",0\n";
    report << name << ",vel_violation," << boolName(baseline.velocity_violation) << ","
           << boolName(candidate.velocity_violation) << ",0\n";
    report << name << ",acc_violation," << boolName(baseline.acceleration_violation) << ","
           << boolName(candidate.acceleration_violation) << ",0\n";
    report << name << ",pos_violation," << boolName(baseline.position_violation) << ","
           << boolName(candidate.position_violation) << ",0\n";
    report << name << ",samples," << baseline.samples << "," << candidate.samples << ",0\n";
  };
  report << "category,metric,baseline,candidate,change_percent\n";
  row("continuous", result.baseline_continuous, result.candidate_continuous);
  row("command_interp", result.baseline_metrics, result.candidate_metrics);
  report << "selected_command_interp_rms," << (result.accepted ? "candidate" : "baseline") << ","
         << result.selected_metrics.jerk_rms << "\n";
  report.close();

  std::ofstream summary(out_dir + "/time_profile_summary.csv");
  summary << std::setprecision(12)
          << "sample,accepted,selected_plan,baseline_rms,candidate_rms,selected_rms,"
             "baseline_peak,candidate_peak,selected_peak,selected_command_interp_rms,"
             "selected_command_interp_peak,baseline_command_interp_rms,"
             "baseline_command_interp_peak,candidate_command_interp_rms,"
             "candidate_command_interp_peak,constraints_ok,rms_improved,"
             "peak_ok,export_fidelity_ok,export_spacing_used,"
             "baseline_export_deviation_rad,candidate_export_deviation_rad,evaluations,solve_sec,"
             "duration_scale,target_duration_s,"
             "fallback_reason\n"
          << trajectory_path << ',' << boolName(result.accepted) << ','
          << (result.accepted ? "candidate" : "baseline") << ','
          << result.baseline_continuous.jerk_rms << ','
          << result.candidate_continuous.jerk_rms << ','
          << (result.accepted ? result.candidate_continuous.jerk_rms
                              : result.baseline_continuous.jerk_rms)
          << ','
          << result.baseline_continuous.jerk_max << ','
          << result.candidate_continuous.jerk_max << ','
          << (result.accepted ? result.candidate_continuous.jerk_max
                              : result.baseline_continuous.jerk_max)
          << ',' << result.selected_metrics.jerk_rms << ',' << result.selected_metrics.jerk_max << ','
          << result.baseline_metrics.jerk_rms << ',' << result.baseline_metrics.jerk_max << ','
          << result.candidate_metrics.jerk_rms << ',' << result.candidate_metrics.jerk_max << ','
          << boolName(result.constraints_ok) << ',' << boolName(result.rms_improved) << ','
          << boolName(result.peak_ok) << ',' << boolName(result.export_fidelity_ok) << ','
          << result.export_spacing_used << ',' << result.baseline_export_deviation_rad << ','
          << result.max_export_deviation_rad << ',' << result.evaluations << ','
          << solve_sec << ',' << duration_scale << ',' << target_duration << ','
          << (result.fallback_reason.empty() ? "none" : result.fallback_reason) << '\n';

  std::cout << "time profile optimisation (fixed geometry, fixed duration)\n"
            << "  source            : " << sample.source << "\n"
            << "  duration          : " << path.duration() << " s\n"
            << "  parameters        : " << result.parameters.size() << "\n"
            << "  evaluations       : " << result.evaluations << " / " << options.max_evaluations
            << "\n"
            << "  continuous jerk   : " << result.baseline_continuous.jerk_rms << " -> "
            << result.candidate_continuous.jerk_rms << " ("
            << 100.0 * (result.candidate_continuous.jerk_rms - result.baseline_continuous.jerk_rms) /
                   std::max(1e-12, result.baseline_continuous.jerk_rms)
            << " %)\n"
            << "  command interp RMS: " << result.baseline_metrics.jerk_rms << " -> "
            << result.candidate_metrics.jerk_rms << "\n"
            << "  selected plan     : " << (result.accepted ? "candidate" : "baseline") << "\n"
            << "  fallback reason   : "
            << (result.fallback_reason.empty() ? "none" : result.fallback_reason) << "\n"
            << "  export spacing    : " << result.export_spacing_used << " s\n"
            << "  export deviation  : " << result.baseline_export_deviation_rad << " / "
            << result.max_export_deviation_rad << " rad (baseline/candidate)\n";
  return 0;
}
