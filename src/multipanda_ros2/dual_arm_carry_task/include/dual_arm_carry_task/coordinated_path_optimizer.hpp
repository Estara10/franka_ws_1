#pragma once

// S2 transport-segment projection smoothing.
//
// The object-space path X_k and the two fixed grasp transforms G_L/G_R are
// frozen. Only the internal joint waypoints q_k (R^14) move, and only inside
// the null space of the two task Jacobians, so every waypoint still satisfies
// FK_L(q_k) = X_k*G_L and FK_R(q_k) = X_k*G_R.
//
// This module is deliberately model-agnostic: forward kinematics, the task
// Jacobian and every feasibility check are supplied as callbacks. It has no
// ROS dependency, so the objective, the finite-difference Jacobian and the
// stopping rules are testable without a robot model.
//
// The objective is a GEOMETRIC path-smoothness measure, not a time-domain
// jerk, an energy or a contact force:
//
//   J = w2*A2/S2 + w1*A1/S1 + w0*A0/S0
//
//   A1 = sum_k ||q_{k+1}-q_k||^2 / ds_k                      (first difference)
//   A2 = sum_k h_k ||d2_k||^2                                (second derivative)
//        d2_k = ((q_{k+1}-q_k)/ds_k - (q_k-q_{k-1})/ds_{k-1}) / h_k
//        with h_k = (ds_{k-1}+ds_k)/2
//   A0 = sum_internal ||q_k - q_seed_k||^2                   (regularisation)
//
// ds_k are the path-parameter intervals. The second term uses divided
// differences, so it is a second derivative on any grid; the plain second
// difference q_{k+1}-2q_k+q_{k-1} would only be one on a uniform grid.
// s is normalised to [0,1]. Joints are in radians.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace cpo
{
using JointVector = Eigen::VectorXd;
using Transform = Eigen::Isometry3d;

struct DualPose
{
  Transform left{Transform::Identity()};
  Transform right{Transform::Identity()};
};

// Forward kinematics at one configuration. Returns false on non-finite input or
// an unusable model state.
using ProbeFn = std::function<bool(const JointVector& q, DualPose& out)>;

// Whole-path feasibility (bounds, continuity, collision, ...). Receives the
// candidate path including the fixed endpoints.
using ValidateFn = std::function<bool(const std::vector<JointVector>& path,
                                      std::string& detail)>;

struct ObjectiveTerms
{
  double a2{0.0};
  double a1{0.0};
  double a0{0.0};
};

struct ObjectiveValue
{
  ObjectiveTerms raw;
  double norm_a2{0.0};
  double norm_a1{0.0};
  double norm_a0{0.0};
  double total{0.0};
};

struct Options
{
  // Frozen before the first optimisation run; never tuned to manufacture gains.
  double w2{1.0};
  double w1{0.1};
  double w0{0.01};
  // S0 = joint_count * internal_count * s0_scale^2. The regularisation term is
  // zero at the seed, so it cannot be used as its own denominator.
  double s0_scale{0.1};  // rad

  int max_iterations{30};
  // Trial step lengths, applied to an objective-scaled base step (see below);
  // the first is the full step, the rest backtrack geometrically.
  std::array<double, 5> backtrack{{1.0, 0.1, 0.01, 0.001, 0.0001}};
  // The base step predicts a linear objective decrease of this fraction of |J|.
  // The normalised objective is stiff (S1/S2 divide by the seed values), so a
  // fixed joint-space step is orders of magnitude too coarse.
  double objective_step_fraction{0.1};
  // Projected-gradient RMS below which the path is treated as stationary.
  double gradient_stop{1e-6};

  double pos_tol{0.0005};                   // 0.5 mm, the existing IK residual
  double rot_tol{0.1 * M_PI / 180.0};       // 0.1 deg
  double fd_step{1e-6};                     // numerical Jacobian step (rad)
  double damping{1e-4};                     // relative damping of the solve
  double max_step_rad{0.02};                // per-waypoint update cap
  // A step must reduce J by at least this fraction of |J| to be accepted.
  double min_relative_decrease{1e-9};
};

struct Diagnostics
{
  ObjectiveValue initial;
  ObjectiveValue final;
  int variables{0};
  int internal_waypoints{0};
  int left_rank{0};
  int right_rank{0};
  int rank{0};
  int null_dim{0};
  double projected_gradient_rms{0.0};
  int iterations{0};
  int accepted_steps{0};
  // improved | stationary | feasibility_limited | projection_limited |
  // not_applicable | invalid_input
  std::string status;
  std::string detail;
  bool accepted{false};
};

struct Result
{
  std::vector<JointVector> path;
  Diagnostics diag;
};

// Pure objective evaluation, exposed so the discretisation can be tested.
// Returns false (and sets error) on invalid input.
bool evaluateObjective(const std::vector<JointVector>& path,
                       const std::vector<double>& path_params,
                       const std::vector<JointVector>& seed_path,
                       double w2, double s2,
                       double w1, double s1,
                       double w0, double s0,
                       ObjectiveValue& out,
                       std::string& error);

// Seed scales S1/S2/S0 for the given seed path. Exposed for the run record.
bool seedScales(const std::vector<JointVector>& seed_path,
                const std::vector<double>& path_params,
                const Options& options,
                double& s1, double& s2, double& s0,
                std::string& error);

// Analytic gradient of the objective with respect to every waypoint. The
// endpoint entries are returned as zero. Exposed so the divided-difference
// stencil can be checked against a central difference of evaluateObjective.
bool objectiveGradient(const std::vector<JointVector>& path,
                       const std::vector<double>& path_params,
                       const std::vector<JointVector>& seed_path,
                       double w2, double s2,
                       double w1, double s1,
                       double w0, double s0,
                       std::vector<JointVector>& grad,
                       std::string& error);

Result optimize(const std::vector<JointVector>& seed_path,
                const std::vector<Transform>& left_targets,
                const std::vector<Transform>& right_targets,
                const std::vector<double>& path_params,
                const ProbeFn& probe,
                const ValidateFn& validate,
                const Options& options = Options{});

}  // namespace cpo
