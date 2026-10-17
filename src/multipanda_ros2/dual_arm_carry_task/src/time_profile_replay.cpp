// Offline replay of the constrained time-profile optimisation.
//
// Reads the saved transport sample (long-format JointTrajectoryController
// points + provenance/limits), optimises only the monotone time map s(tau),
// and writes the candidate trajectory plus a comparison report. No ROS, no
// simulation: the geometry is frozen, so iterating here never needs a new run.

#include "dual_arm_carry_task/time_profile_optimizer.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cstdio>
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
      if (!std::getline(ss, tag, ',') || !std::getline(ss, name, ',') || !std::getline(ss, minp, ',') ||
          !std::getline(ss, maxp, ',') || !std::getline(ss, minv, ',') || !std::getline(ss, maxv, ',') ||
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
  if (limits.empty())
  {
    error = "no limit lines in the meta file";
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

void writeTrajectory(const std::string& path, const std::vector<std::string>& names,
                     const std::vector<tpo::TrajectoryPoint>& points)
{
  std::ofstream out(path);
  out << std::setprecision(17) << "joint_name,point_index,time_from_start,position,velocity,acceleration\n";
  for (std::size_t j = 0; j < names.size(); ++j)
  {
    for (std::size_t k = 0; k < points.size(); ++k)
    {
      out << names[j] << ',' << k << ',' << points[k].time << ',' << points[k].q[static_cast<Eigen::Index>(j)]
          << ',' << points[k].v[static_cast<Eigen::Index>(j)] << ','
          << points[k].a[static_cast<Eigen::Index>(j)] << '\n';
    }
  }
}

const char* boolName(bool v) { return v ? "1" : "0"; }
}  // namespace

int main(int argc, char** argv)
{
  std::string run_dir;
  std::string out_dir;
  int evaluations = 100;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--run-dir" && i + 1 < argc)
      run_dir = argv[++i];
    else if (arg == "--out-dir" && i + 1 < argc)
      out_dir = argv[++i];
    else if (arg == "--max-evaluations" && i + 1 < argc)
      evaluations = std::stoi(argv[++i]);
    else
    {
      std::cerr << "usage: time_profile_replay --run-dir <dir> [--out-dir <dir>] "
                   "[--max-evaluations N]\n";
      return 2;
    }
  }
  if (run_dir.empty())
  {
    std::cerr << "error: --run-dir is required\n";
    return 2;
  }
  if (out_dir.empty()) out_dir = run_dir;

  SavedSample sample;
  std::string error;
  if (!readTrajectory(run_dir + "/transport_baseline_trajectory.csv", sample, error) ||
      !readMeta(run_dir + "/transport_sample_meta.txt", sample, error))
  {
    std::cerr << "error: " << error << "\n";
    return 1;
  }

  tpo::ContinuousPath path;
  if (!path.build(sample.joint_names, sample.points, error))
  {
    std::cerr << "error: " << error << "\n";
    return 1;
  }

  tpo::TimeProfileOptions options;
  options.max_evaluations = evaluations;
  const auto result = tpo::optimizeTimeProfile(path, sample.limits, path.duration(), options);
  if (!result.ok)
  {
    std::cerr << "error: " << result.status << ": " << result.detail << "\n";
    return 1;
  }

  writeTrajectory(out_dir + "/time_profile_candidate.csv", sample.joint_names, result.candidate_export);

  std::ofstream report(out_dir + "/time_profile_report.txt");
  report << std::setprecision(10);
  report << "sample=" << run_dir << "\n"
         << "source=" << sample.source << "\n"
         << "method=" << sample.method << "\n"
         << "joints=" << sample.joint_names.size() << "\n"
         << "points=" << sample.points.size() << "\n"
         << "duration=" << path.duration() << "\n"
         << "degree=" << options.degree << "\n"
         << "parameters=" << result.parameters.size() << "\n"
         << "max_evaluations=" << options.max_evaluations << "\n"
         << "evaluations=" << result.evaluations << "\n"
         << "parameter_values=";
  for (Eigen::Index i = 0; i < result.parameters.size(); ++i)
    report << (i == 0 ? "" : ",") << result.parameters[i];
  report << "\n";
  auto row = [&](const char* name, const tpo::TrajectoryMetrics& b, const tpo::TrajectoryMetrics& c) {
    report << name << ",jerk_rms," << b.jerk_rms << "," << c.jerk_rms << ","
           << (b.jerk_rms > 0.0 ? 100.0 * (c.jerk_rms - b.jerk_rms) / b.jerk_rms : 0.0) << "\n";
    report << name << ",jerk_max," << b.jerk_max << "," << c.jerk_max << ","
           << (b.jerk_max > 0.0 ? 100.0 * (c.jerk_max - b.jerk_max) / b.jerk_max : 0.0) << "\n";
    report << name << ",acc_rms," << b.acceleration_rms << "," << c.acceleration_rms << ","
           << (b.acceleration_rms > 0.0 ? 100.0 * (c.acceleration_rms - b.acceleration_rms) / b.acceleration_rms
                                        : 0.0)
           << "\n";
    report << name << ",max_vel_ratio," << b.max_velocity_ratio << "," << c.max_velocity_ratio << ",0\n";
    report << name << ",max_acc_ratio," << b.max_acceleration_ratio << "," << c.max_acceleration_ratio << ",0\n";
    report << name << ",vel_violation," << boolName(b.velocity_violation) << ","
           << boolName(c.velocity_violation) << ",0\n";
    report << name << ",acc_violation," << boolName(b.acceleration_violation) << ","
           << boolName(c.acceleration_violation) << ",0\n";
    report << name << ",pos_violation," << boolName(b.position_violation) << ","
           << boolName(c.position_violation) << ",0\n";
    report << name << ",samples," << b.samples << "," << c.samples << ",0\n";
  };
  report << "category,metric,baseline,candidate,change_percent\n";
  row("continuous", result.baseline_continuous, result.candidate_continuous);
  row("exported_jtc", result.baseline_metrics, result.candidate_metrics);

  tpo::BernsteinTimeMap baseline_map(options.degree);
  baseline_map.setParameters(Eigen::VectorXd::Zero(result.parameters.size()));
  tpo::BernsteinTimeMap candidate_map(options.degree);
  candidate_map.setParameters(result.parameters);

  // Is the exported 21-point trajectory a faithful representation of its own
  // continuous motion? A large deviation means the gain (or loss) lives in the
  // sample placement, not in the motion.
  double dev_baseline = 0.0, dev_candidate = 0.0;
  const bool dev_ok =
      tpo::maxPositionDeviation(result.baseline_export, path, baseline_map, path.duration(),
                                0.0005, dev_baseline, error) &&
      tpo::maxPositionDeviation(result.candidate_export, path, candidate_map, path.duration(),
                                0.0005, dev_candidate, error);

  // Sensitivity to the export grid: the same time maps exported at 201 uniform
  // times. If the coarse-grid gain does not survive, it came from where the
  // samples land rather than from a smoother motion.
  std::vector<double> dense_times;
  for (int i = 0; i <= 200; ++i) dense_times.push_back(path.duration() * i / 200.0);
  std::vector<tpo::TrajectoryPoint> dense_baseline, dense_candidate;
  tpo::TrajectoryMetrics dense_baseline_metrics, dense_candidate_metrics;
  const bool dense_ok =
      tpo::exportTrajectory(path, baseline_map, path.duration(), dense_times, dense_baseline, error) &&
      tpo::exportTrajectory(path, candidate_map, path.duration(), dense_times, dense_candidate, error) &&
      tpo::evaluateExported(dense_baseline, sample.limits, options.interpolation_dt,
                            dense_baseline_metrics, error) &&
      tpo::evaluateExported(dense_candidate, sample.limits, options.interpolation_dt,
                            dense_candidate_metrics, error);
  if (dense_ok)
  {
    row("exported_dense_201", dense_baseline_metrics, dense_candidate_metrics);
  }
  if (dev_ok)
  {
    report << "export_fidelity,max_joint_deviation_rad," << dev_baseline << "," << dev_candidate
           << ",0\n";
  }
  report.close();

  std::cout << "time profile optimisation (fixed geometry, fixed duration)\n"
            << "  source            : " << sample.source << "\n"
            << "  duration          : " << path.duration() << " s\n"
            << "  parameters        : " << result.parameters.size() << "\n"
            << "  evaluations       : " << result.evaluations << " / " << options.max_evaluations << "\n"
            << "  continuous jerk   : " << result.baseline_continuous.jerk_rms << " -> "
            << result.candidate_continuous.jerk_rms << " ("
            << 100.0 * (result.candidate_continuous.jerk_rms - result.baseline_continuous.jerk_rms) /
                   std::max(1e-12, result.baseline_continuous.jerk_rms)
            << " %)\n"
            << "  exported jerk RMS : " << result.baseline_metrics.jerk_rms << " -> "
            << result.candidate_metrics.jerk_rms << " ("
            << 100.0 * (result.candidate_metrics.jerk_rms - result.baseline_metrics.jerk_rms) /
                   std::max(1e-12, result.baseline_metrics.jerk_rms)
            << " %)\n"
            << "  exported jerk max : " << result.baseline_metrics.jerk_max << " -> "
            << result.candidate_metrics.jerk_max << "\n"
            << "  velocity/acceleration violations: candidate "
            << boolName(result.candidate_metrics.velocity_violation) << "/"
            << boolName(result.candidate_metrics.acceleration_violation) << "\n";
  if (dense_ok)
  {
    std::cout << "  dense(201) jerk   : " << dense_baseline_metrics.jerk_rms << " -> "
              << dense_candidate_metrics.jerk_rms << " ("
              << 100.0 * (dense_candidate_metrics.jerk_rms - dense_baseline_metrics.jerk_rms) /
                     std::max(1e-12, dense_baseline_metrics.jerk_rms)
              << " %)\n";
  }
  if (dev_ok)
  {
    std::cout << "  export fidelity   : max joint deviation from the continuous curve "
              << dev_baseline << " -> " << dev_candidate << " rad\n";
  }
  return 0;
}
