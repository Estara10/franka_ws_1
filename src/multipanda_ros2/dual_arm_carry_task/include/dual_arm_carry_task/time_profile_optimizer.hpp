#pragma once

// Constrained time-profile optimisation on a FIXED geometric path.
//
// The geometric path is never re-optimised here: q(s) is the baseline
// JointTrajectoryController trajectory recovered from its own quintic
// interpolation, reparameterised by s in [0,1]. Only the monotone map
// s(tau), tau = t/T in [0,1], is optimised, so the robot visits exactly the
// same configurations in the same order.
//
// Chain rule actually used (see also the .cpp):
//   qdot  = qs*sdot
//   qddot = qss*sdot^2 + qs*sddot
//   qjerk = qsss*sdot^3 + 3*qss*sdot*sddot + qs*sjerk
// where the dots are with respect to real time and qs = dq/ds.
//
// The objective is a jerk measure. It is an ALGORITHM SETTING, not a declared
// robot jerk limit: the model declares position/velocity/acceleration bounds
// only.
//
// No ROS dependency.

#include <Eigen/Core>

#include <string>
#include <vector>

namespace tpo
{
struct TrajectoryPoint
{
  double time{0.0};
  Eigen::VectorXd q;
  Eigen::VectorXd v;
  Eigen::VectorXd a;
};

struct JointLimits
{
  double min_position{-3.2};
  double max_position{3.2};
  double min_velocity{-2.0};
  double max_velocity{2.0};
  double min_acceleration{-5.0};
  double max_acceleration{5.0};
};

// Continuous q(s) on s in [0,1], piecewise quintic, C2 across the knots.
// The third derivative is piecewise continuous and jumps at the knots; this
// class does not claim any smoothness beyond that.
class ContinuousPath
{
public:
  bool build(const std::vector<std::string>& joint_names,
             const std::vector<TrajectoryPoint>& points,
             std::string& error);

  bool valid() const { return valid_; }
  Eigen::Index dimension() const { return dimension_; }
  double duration() const { return duration_; }
  std::size_t segmentCount() const { return points_.size() > 1 ? points_.size() - 1 : 0; }
  const std::vector<std::string>& jointNames() const { return joint_names_; }
  // Knot times of the baseline trajectory (t_0 = 0). The export uses these for
  // both the baseline and the candidate.
  std::vector<double> knotTimes() const;

  // q, dq/ds, d2q/ds2, d3q/ds3 at s in [0,1].
  bool evaluate(double s,
                Eigen::VectorXd& q,
                Eigen::VectorXd& qs,
                Eigen::VectorXd& qss,
                Eigen::VectorXd& qsss,
                std::string& error) const;

private:
  std::size_t segmentFor(double s) const;

  bool valid_{false};
  Eigen::Index dimension_{0};
  double duration_{0.0};
  std::vector<std::string> joint_names_;
  std::vector<TrajectoryPoint> points_;
};

// Monotone map s(tau) built from a positive Bernstein speed profile
//   w(tau) = sum_i c_i B_{i,d}(tau),  c_0 = 1, c_i = exp(u_i)
//   s(tau) = int_0^tau w / int_0^1 w
// w > 0, so s is strictly increasing; s(0)=0 and s(1)=1 by construction.
// u = 0 gives c_i = 1 for all i, i.e. w = 1 and s(tau) = tau (the linear clock).
class BernsteinTimeMap
{
public:
  explicit BernsteinTimeMap(int degree = 6);

  int degree() const { return degree_; }
  int parameterCount() const { return degree_; }  // u_1 .. u_d

  void setParameters(const Eigen::VectorXd& u);
  const Eigen::VectorXd& parameters() const { return u_; }

  double s(double tau) const;
  double dsdtau(double tau) const;
  double ddsdtau(double tau) const;
  double dddsdtau(double tau) const;
  // Inverse of the monotone map. Used to preserve the original path segment
  // boundaries when a non-linear clock is exported to the controller.
  double inverseS(double s_value, double tolerance = 1e-12) const;

  // Coefficient c_i for the current parameters (index 0 is fixed at 1).
  double coefficient(int i) const;

private:
  double bernstein(int i, int degree, double tau) const;
  void coefficients();

  int degree_;
  Eigen::VectorXd u_;
  std::vector<double> c_;
};

struct TrajectoryMetrics
{
  std::size_t samples{0};
  double duration{0.0};
  double max_velocity_ratio{0.0};
  double max_acceleration_ratio{0.0};
  bool velocity_violation{false};
  bool acceleration_violation{false};
  bool position_violation{false};
  double acceleration_rms{0.0};
  double jerk_rms{0.0};
  double jerk_max{0.0};
};

struct TimeProfileOptions
{
  int degree{6};
  // Sampling interval used for both the continuous objective and the
  // continuous-vs-export representation check.
  double interpolation_dt{0.001};
  int max_evaluations{100};
  // Penalty weight for limit violations; a candidate must be feasible.
  double violation_weight{1e6};
  // Export sample spacing. This is deliberately separate from the 1 ms
  // evaluation interval used to measure the continuous objective.
  double export_spacing{0.005};
  double refined_export_spacing{0.0025};
  double export_fidelity_tolerance{1e-5};
};

struct TimeProfileResult
{
  bool ok{false};
  bool accepted{false};
  bool fallback_to_baseline{true};
  bool constraints_ok{false};
  bool rms_improved{false};
  bool peak_ok{false};
  bool export_fidelity_ok{false};
  double total_duration{0.0};
  Eigen::VectorXd parameters;
  Eigen::VectorXd selected_parameters;
  int evaluations{0};
  double objective{0.0};
  double baseline_objective{0.0};
  TrajectoryMetrics baseline_metrics;
  TrajectoryMetrics candidate_metrics;
  TrajectoryMetrics selected_metrics;
  // Metrics of the continuous composition, before export (no JTC
  // re-interpolation). Reported separately from the exported result.
  TrajectoryMetrics baseline_continuous;
  TrajectoryMetrics candidate_continuous;
  std::vector<TrajectoryPoint> baseline_export;
  std::vector<TrajectoryPoint> candidate_export;
  std::vector<TrajectoryPoint> selected_export;
  double export_spacing_used{0.0};
  double baseline_export_deviation_rad{0.0};
  double max_export_deviation_rad{0.0};
  std::string status;
  std::string detail;
  std::string decision;
  std::string fallback_reason;
};

struct TimeProfileAcceptance
{
  bool constraints_ok{false};
  bool rms_improved{false};
  bool peak_ok{false};
  bool accepted{false};
  std::string fallback_reason;
};

TimeProfileAcceptance evaluateTimeProfileAcceptance(
    const TrajectoryMetrics& baseline_continuous,
    const TrajectoryMetrics& candidate_continuous,
    bool export_fidelity_ok);

// Sample q(s(tau)) at the GIVEN times and fill q/v/a by the chain rule. This
// low-level interface is used by buildFaithfulExport(), which owns the common
// boundary-preserving grid policy.
bool exportTrajectory(const ContinuousPath& path,
                      const BernsteinTimeMap& map,
                      double duration,
                      const std::vector<double>& times,
                      std::vector<TrajectoryPoint>& out,
                      std::string& error);

// Build a controller-facing export with one consistent method for both the
// baseline and a candidate. The grid contains uniform samples plus the
// original path segment boundaries mapped through the current clock.
bool buildFaithfulExport(const ContinuousPath& path,
                         const BernsteinTimeMap& map,
                         double duration,
                         double export_spacing,
                         std::vector<TrajectoryPoint>& out,
                         std::string& error);

// Metrics of the trajectory AS THE CONTROLLER WILL INTERPOLATE IT (quintic
// between consecutive points), sampled every options.interpolation_dt.
bool evaluateExported(const std::vector<TrajectoryPoint>& points,
                      const std::vector<JointLimits>& limits,
                      double interpolation_dt,
                      TrajectoryMetrics& out,
                      std::string& error);

// Evaluate the same piecewise quintic used by JointTrajectoryController at
// an absolute trajectory time. Used to derive MMC Cartesian pose/twist
// references from the final, already-checked joint trajectory.
bool interpolateQuintic(const std::vector<TrajectoryPoint>& points,
                         double time,
                         TrajectoryPoint& out,
                         std::string& error);

// Metrics of the continuous composition q(s(tau)) without any export. Used to
// separate "the optimisation worked" from "the export kept it".
bool evaluateContinuous(const ContinuousPath& path,
                        const BernsteinTimeMap& map,
                        double duration,
                        const std::vector<JointLimits>& limits,
                        double sample_dt,
                        TrajectoryMetrics& out,
                        std::string& error);

// Largest joint-space gap between the exported trajectory (as the controller
// interpolates it) and the continuous composition it was sampled from. A large
// value means the exported representation does not faithfully carry the
// optimised motion.
bool maxPositionDeviation(const std::vector<TrajectoryPoint>& points,
                          const ContinuousPath& path,
                          const BernsteinTimeMap& map,
                          double duration,
                          double sample_dt,
                          double& max_deviation,
                          std::string& error);

TimeProfileResult optimizeTimeProfile(const ContinuousPath& path,
                                      const std::vector<JointLimits>& limits,
                                      double duration,
                                      const TimeProfileOptions& options);

}  // namespace tpo
