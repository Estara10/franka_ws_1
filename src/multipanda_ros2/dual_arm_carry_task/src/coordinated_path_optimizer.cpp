#include "dual_arm_carry_task/coordinated_path_optimizer.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cpo
{
namespace
{
constexpr int kErrorRows = 12;

bool validPath(const std::vector<JointVector>& path)
{
  if (path.size() < 2) return false;
  const Eigen::Index dim = path.front().size();
  if (dim <= 0) return false;
  for (const auto& q : path)
  {
    if (q.size() != dim || !q.allFinite()) return false;
  }
  return true;
}

bool validParams(const std::vector<double>& s, std::size_t expected)
{
  if (s.size() != expected) return false;
  if (!std::isfinite(s.front()) || !std::isfinite(s.back())) return false;
  for (std::size_t i = 1; i < s.size(); ++i)
  {
    if (!std::isfinite(s[i]) || !(s[i] > s[i - 1])) return false;
  }
  return true;
}

void objectiveTerms(const std::vector<JointVector>& path,
                    const std::vector<double>& s,
                    const std::vector<JointVector>& seed,
                    bool include_regularisation,
                    ObjectiveTerms& out)
{
  out = ObjectiveTerms{};
  const std::size_t n = path.size();
  for (std::size_t i = 0; i + 1 < n; ++i)
  {
    const double ds = s[i + 1] - s[i];
    out.a1 += (path[i + 1] - path[i]).squaredNorm() / ds;
  }
  // Interior second derivative on the path-parameter grid. The plain second
  // difference q[k+1]-2q[k]+q[k-1] is only a derivative on a uniform grid; the
  // divided differences below are one on any grid, and are integrated against
  // the local half-width h.
  for (std::size_t k = 1; k + 1 < n; ++k)
  {
    const double hm = s[k] - s[k - 1];
    const double hp = s[k + 1] - s[k];
    const double h = 0.5 * (hm + hp);
    const JointVector d2 = ((path[k + 1] - path[k]) / hp - (path[k] - path[k - 1]) / hm) / h;
    out.a2 += h * d2.squaredNorm();
  }
  if (include_regularisation)
  {
    for (std::size_t k = 1; k + 1 < n; ++k)
    {
      out.a0 += (path[k] - seed[k]).squaredNorm();
    }
  }
}
}  // namespace

bool seedScales(const std::vector<JointVector>& seed_path,
                const std::vector<double>& path_params,
                const Options& options,
                double& s1, double& s2, double& s0,
                std::string& error)
{
  error.clear();
  if (!validPath(seed_path))
  {
    error = "seed path is empty, ragged or non-finite";
    return false;
  }
  if (!validParams(path_params, seed_path.size()))
  {
    error = "path parameters must be strictly increasing and match the path size";
    return false;
  }
  ObjectiveTerms terms;
  objectiveTerms(seed_path, path_params, seed_path, false, terms);
  s1 = std::max(terms.a1, 1e-12);
  s2 = std::max(terms.a2, 1e-12);
  const double internal = static_cast<double>(seed_path.size() - 2);
  s0 = static_cast<double>(seed_path.front().size()) * internal *
       options.s0_scale * options.s0_scale;
  if (!(s0 > 0.0))
  {
    s0 = 1.0;
  }
  if (!std::isfinite(s1) || !std::isfinite(s2) || !std::isfinite(s0))
  {
    error = "seed objective scales are not finite";
    return false;
  }
  return true;
}

bool evaluateObjective(const std::vector<JointVector>& path,
                       const std::vector<double>& path_params,
                       const std::vector<JointVector>& seed_path,
                       double w2, double s2,
                       double w1, double s1,
                       double w0, double s0,
                       ObjectiveValue& out,
                       std::string& error)
{
  error.clear();
  out = ObjectiveValue{};
  if (!validPath(path) || !validPath(seed_path) || path.size() != seed_path.size() ||
      path.front().size() != seed_path.front().size())
  {
    error = "path is empty, ragged, non-finite or does not match the seed";
    return false;
  }
  if (!validParams(path_params, path.size()))
  {
    error = "path parameters must be strictly increasing and match the path size";
    return false;
  }
  if (!(s1 > 0.0) || !(s2 > 0.0) || !(s0 > 0.0) || !std::isfinite(s1) ||
      !std::isfinite(s2) || !std::isfinite(s0))
  {
    error = "objective scales must be finite and positive";
    return false;
  }
  objectiveTerms(path, path_params, seed_path, true, out.raw);
  out.norm_a2 = out.raw.a2 / s2;
  out.norm_a1 = out.raw.a1 / s1;
  out.norm_a0 = out.raw.a0 / s0;
  out.total = w2 * out.norm_a2 + w1 * out.norm_a1 + w0 * out.norm_a0;
  if (!std::isfinite(out.total))
  {
    error = "objective is not finite";
    return false;
  }
  return true;
}

namespace
{
// 6-DoF pose error of `current` against `target`, expressed in the target
// frame so that the two arms' errors share one parameterisation.
Eigen::Matrix<double, 6, 1> poseError(const Transform& current, const Transform& target)
{
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = current.translation() - target.translation();
  const Eigen::Matrix3d dR = target.linear().transpose() * current.linear();
  const Eigen::AngleAxisd aa(dR);
  e.tail<3>() = aa.axis() * aa.angle();
  return e;
}

Eigen::Matrix<double, kErrorRows, 1> dualError(const DualPose& pose,
                                                const Transform& left_target,
                                                const Transform& right_target)
{
  Eigen::Matrix<double, kErrorRows, 1> e;
  e.segment<6>(0) = poseError(pose.left, left_target);
  e.segment<6>(6) = poseError(pose.right, right_target);
  return e;
}

bool probeAt(const JointVector& q, const ProbeFn& probe, DualPose& out)
{
  out = DualPose{};
  if (!probe(q, out)) return false;
  return std::isfinite(out.left.translation().x()) &&
         std::isfinite(out.left.translation().y()) &&
         std::isfinite(out.left.translation().z()) &&
         std::isfinite(out.right.translation().x()) &&
         std::isfinite(out.right.translation().y()) &&
         std::isfinite(out.right.translation().z()) &&
         out.left.linear().allFinite() && out.right.linear().allFinite();
}

// Central-difference task Jacobian. dE/dq for the whole stacked error vector;
// the two arms are independent groups, so the off-diagonal blocks are zero.
bool numericJacobian(const JointVector& q,
                     const Transform& left_target,
                     const Transform& right_target,
                     const ProbeFn& probe,
                     double h,
                     Eigen::MatrixXd& jac)
{
  const Eigen::Index n = q.size();
  jac.setZero(kErrorRows, n);
  DualPose plus, minus;
  for (Eigen::Index j = 0; j < n; ++j)
  {
    JointVector qp = q;
    JointVector qm = q;
    qp[j] += h;
    qm[j] -= h;
    if (!probeAt(qp, probe, plus) || !probeAt(qm, probe, minus)) return false;
    const auto ep = dualError(plus, left_target, right_target);
    const auto em = dualError(minus, left_target, right_target);
    jac.col(j) = (ep - em) / (2.0 * h);
  }
  return jac.allFinite();
}

struct NullSpace
{
  Eigen::MatrixXd projector;
  Eigen::MatrixXd pseudo_inverse;
  int rank{0};
};

NullSpace buildNullSpace(const Eigen::MatrixXd& jac, double damping)
{
  NullSpace out;
  const Eigen::Index rows = jac.rows();
  const Eigen::Index cols = jac.cols();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& sv = svd.singularValues();
  const double largest = sv.size() > 0 ? sv(0) : 0.0;
  const double rank_tol = std::max(largest * 1e-6, 1e-12);
  int rank = 0;
  for (Eigen::Index i = 0; i < sv.size(); ++i)
  {
    if (sv(i) > rank_tol) ++rank;
  }
  const double lambda = std::max(damping * std::max(largest, 1e-12), 1e-12);
  const Eigen::MatrixXd& U = svd.matrixU();
  const Eigen::MatrixXd& V = svd.matrixV();

  out.projector = Eigen::MatrixXd::Identity(cols, cols);
  for (int i = 0; i < rank; ++i)
  {
    out.projector -= V.col(i) * V.col(i).transpose();
  }
  out.pseudo_inverse = Eigen::MatrixXd::Zero(cols, rows);
  for (int i = 0; i < rank; ++i)
  {
    const double sigma = sv(i);
    const double factor = sigma / (sigma * sigma + lambda * lambda);
    out.pseudo_inverse += factor * V.col(i) * U.col(i).transpose();
  }
  out.rank = rank;
  return out;
}

int rankOf(const Eigen::MatrixXd& block)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(block);
  const auto& sv = svd.singularValues();
  if (sv.size() == 0) return 0;
  const double tol = std::max(sv(0) * 1e-6, 1e-12);
  int rank = 0;
  for (Eigen::Index i = 0; i < sv.size(); ++i)
  {
    if (sv(i) > tol) ++rank;
  }
  return rank;
}

void gradient(const std::vector<JointVector>& path,
              const std::vector<JointVector>& seed,
              const std::vector<double>& s,
              double w2, double s2,
              double w1, double s1,
              double w0, double s0,
              std::vector<JointVector>& grad)
{
  const std::size_t n = path.size();
  grad.assign(n, JointVector::Zero(path.front().size()));

  // dA1/dq_j = 2(q_j-q_{j-1})/ds_{j-1} - 2(q_{j+1}-q_j)/ds_j
  for (std::size_t j = 0; j < n; ++j)
  {
    if (j > 0)
    {
      grad[j] += (2.0 * (path[j] - path[j - 1]) / (s[j] - s[j - 1])) * (w1 / s1);
    }
    if (j + 1 < n)
    {
      grad[j] -= (2.0 * (path[j + 1] - path[j]) / (s[j + 1] - s[j])) * (w1 / s1);
    }
  }
  // dA2/dq for the same divided-difference stencil. Each residual touches
  // k-1, k and k+1; the h of the objective cancels against the 1/h inside d2.
  for (std::size_t k = 1; k + 1 < n; ++k)
  {
    const double hm = s[k] - s[k - 1];
    const double hp = s[k + 1] - s[k];
    const double h = 0.5 * (hm + hp);
    const JointVector d2 = (path[k + 1] - path[k]) / hp / h -
                           (path[k] - path[k - 1]) / hm / h;
    const JointVector base = (2.0 * (w2 / s2)) * d2;
    grad[k - 1] += base / hm;
    grad[k] -= base * (1.0 / hm + 1.0 / hp);
    grad[k + 1] += base / hp;
  }
  for (std::size_t k = 1; k + 1 < n; ++k)
  {
    grad[k] += (2.0 * (path[k] - seed[k])) * (w0 / s0);
  }
}

double projectedGradientRms(const std::vector<JointVector>& grad,
                            const std::vector<NullSpace>& null_spaces,
                            int variables)
{
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t k = 1; k + 1 < grad.size(); ++k)
  {
    const JointVector projected = null_spaces[k].projector * grad[k];
    sum += projected.squaredNorm();
    count += static_cast<std::size_t>(variables);
  }
  if (count == 0) return 0.0;
  return std::sqrt(sum / static_cast<double>(count));
}
}  // namespace

bool objectiveGradient(const std::vector<JointVector>& path,
                       const std::vector<double>& path_params,
                       const std::vector<JointVector>& seed_path,
                       double w2, double s2,
                       double w1, double s1,
                       double w0, double s0,
                       std::vector<JointVector>& grad,
                       std::string& error)
{
  error.clear();
  grad.clear();
  if (!validPath(path) || !validPath(seed_path) || path.size() != seed_path.size() ||
      path.front().size() != seed_path.front().size())
  {
    error = "path is empty, ragged, non-finite or does not match the seed";
    return false;
  }
  if (!validParams(path_params, path.size()))
  {
    error = "path parameters must be strictly increasing and match the path size";
    return false;
  }
  if (!(s1 > 0.0) || !(s2 > 0.0) || !(s0 > 0.0) || !std::isfinite(s1) ||
      !std::isfinite(s2) || !std::isfinite(s0))
  {
    error = "objective scales must be finite and positive";
    return false;
  }
  gradient(path, seed_path, path_params, w2, s2, w1, s1, w0, s0, grad);
  // Both endpoints are fixed, so they carry no optimisation direction.
  grad.front().setZero();
  grad.back().setZero();
  return true;
}

Result optimize(const std::vector<JointVector>& seed_path,
                const std::vector<Transform>& left_targets,
                const std::vector<Transform>& right_targets,
                const std::vector<double>& path_params,
                const ProbeFn& probe,
                const ValidateFn& validate,
                const Options& options)
{
  Result result;
  result.path = seed_path;
  auto& diag = result.diag;

  if (!validPath(seed_path) || left_targets.size() != seed_path.size() ||
      right_targets.size() != seed_path.size() || !probe)
  {
    diag.status = "invalid_input";
    diag.detail = "seed path, target list or probe callback is invalid";
    return result;
  }
  if (!validParams(path_params, seed_path.size()))
  {
    diag.status = "invalid_input";
    diag.detail = "path parameters must be strictly increasing and match the path size";
    return result;
  }
  diag.variables = static_cast<int>(seed_path.front().size());
  diag.internal_waypoints = static_cast<int>(seed_path.size()) - 2;
  if (diag.internal_waypoints <= 0)
  {
    diag.status = "not_applicable";
    diag.detail = "no internal waypoints to optimise";
    return result;
  }

  double s1 = 0.0, s2 = 0.0, s0 = 0.0;
  std::string error;
  if (!seedScales(seed_path, path_params, options, s1, s2, s0, error))
  {
    diag.status = "invalid_input";
    diag.detail = error;
    return result;
  }
  if (!evaluateObjective(seed_path, path_params, seed_path, options.w2, s2, options.w1, s1,
                         options.w0, s0, diag.initial, error))
  {
    diag.status = "invalid_input";
    diag.detail = error;
    return result;
  }
  result.diag.final = diag.initial;

  std::vector<JointVector> current = seed_path;
  ObjectiveValue current_value = diag.initial;
  bool ranked = false;
  bool rejected_by_feasibility = false;
  bool rejected_by_projection = false;
  std::string last_rejection;

  for (int iteration = 0; iteration < options.max_iterations; ++iteration)
  {
    std::vector<JointVector> grad;
    gradient(current, seed_path, path_params, options.w2, s2, options.w1, s1, options.w0, s0,
             grad);

    std::vector<NullSpace> null_spaces(current.size());
    std::vector<JointVector> errors(current.size(), JointVector::Zero(kErrorRows));
    int min_rank = std::numeric_limits<int>::max();
    int min_left = std::numeric_limits<int>::max();
    int min_right = std::numeric_limits<int>::max();
    for (std::size_t k = 1; k + 1 < current.size(); ++k)
    {
      DualPose pose;
      if (!probeAt(current[k], probe, pose))
      {
        diag.status = "invalid_input";
        diag.detail = "forward kinematics failed at internal waypoint " + std::to_string(k);
        return result;
      }
      errors[k] = dualError(pose, left_targets[k], right_targets[k]);
      Eigen::MatrixXd jac;
      if (!numericJacobian(current[k], left_targets[k], right_targets[k], probe,
                           options.fd_step, jac))
      {
        diag.status = "invalid_input";
        diag.detail = "task Jacobian is not finite at internal waypoint " + std::to_string(k);
        return result;
      }
      null_spaces[k] = buildNullSpace(jac, options.damping);
      min_rank = std::min(min_rank, null_spaces[k].rank);
      min_left = std::min(min_left, rankOf(jac.topRows(6)));
      min_right = std::min(min_right, rankOf(jac.bottomRows(6)));
    }
    if (!ranked)
    {
      diag.rank = min_rank;
      diag.left_rank = min_left;
      diag.right_rank = min_right;
      diag.null_dim = diag.variables - min_rank;
      ranked = true;
    }

    const double pg_rms = projectedGradientRms(grad, null_spaces, diag.variables);
    if (iteration == 0) diag.projected_gradient_rms = pg_rms;
    if (pg_rms < options.gradient_stop)
    {
      diag.status = "stationary";
      diag.detail = "projected gradient RMS below the frozen stop threshold";
      break;
    }

    // Per-waypoint proposal: restore the closed chain, then move inside the
    // null space. The projected objective is stiff (its scales divide by the
    // seed values), so the trial step is sized on the OBJECTIVE, not on an
    // absolute joint cap: pick the step whose predicted linear decrease is
    // objective_step_fraction of the current objective, then backtrack
    // geometrically from there. max_step_rad stays as a hard safety cap.
    std::vector<JointVector> task_step(current.size());
    std::vector<JointVector> descent(current.size());
    double directional_derivative = 0.0;
    for (std::size_t k = 1; k + 1 < current.size(); ++k)
    {
      task_step[k] = -null_spaces[k].pseudo_inverse * errors[k];
      descent[k] = -(null_spaces[k].projector * grad[k]);
      // Predicted decrease along the descent direction (positive by construction
      // because the projector is symmetric positive semi-definite).
      directional_derivative -= grad[k].dot(descent[k]);
    }
    task_step.front().setZero();
    task_step.back().setZero();
    descent.front().setZero();
    descent.back().setZero();
    if (!(directional_derivative > 0.0) || !std::isfinite(directional_derivative))
    {
      diag.status = "stationary";
      diag.detail = "no finite descent component in the projected gradient";
      break;
    }
    // J is a sum of quadratic forms in q, so along a fixed direction it is an
    // exact quadratic in the step length. Measure it once and take the exact
    // minimising step; the backtracking list below then only handles
    // feasibility, not objective-shape guessing.
    double base_scale = 0.0;
    {
      const double eps = options.objective_step_fraction *
                         std::max(std::abs(current_value.total), 1e-12) / directional_derivative;
      if (!std::isfinite(eps) || eps <= 0.0)
      {
        diag.status = "stationary";
        diag.detail = "objective-scaled probe step is not finite";
        break;
      }
      std::vector<JointVector> probe_path = current;
      for (std::size_t k = 1; k + 1 < probe_path.size(); ++k)
      {
        probe_path[k] = current[k] + eps * descent[k];
      }
      ObjectiveValue probe_value;
      if (!evaluateObjective(probe_path, path_params, seed_path, options.w2, s2, options.w1, s1,
                             options.w0, s0, probe_value, error))
      {
        diag.status = "invalid_input";
        diag.detail = error;
        return result;
      }
      // J(eps) = J(0) - D*eps + curvature*eps^2
      const double curvature =
          (probe_value.total - current_value.total + directional_derivative * eps) / (eps * eps);
      base_scale = curvature > 0.0 && std::isfinite(curvature)
                       ? directional_derivative / (2.0 * curvature)
                       : eps;
      if (!std::isfinite(base_scale) || base_scale <= 0.0) base_scale = eps;
    }

    bool accepted = false;
    double accepted_value = 0.0;
    std::vector<JointVector> accepted_path;
    for (const double alpha : options.backtrack)
    {
      std::vector<JointVector> candidate = current;
      for (std::size_t k = 1; k + 1 < candidate.size(); ++k)
      {
        // Only the projected part is scaled; the closed-chain correction stays
        // whole so every trial waypoint is still a valid dual IK solution.
        JointVector step = task_step[k] + (alpha * base_scale) * descent[k];
        const double norm = step.norm();
        if (norm > options.max_step_rad)
        {
          step *= options.max_step_rad / norm;
        }
        candidate[k] = current[k] + step;
      }
      bool feasible = validPath(candidate);
      std::string detail;
      if (feasible)
      {
        // Every waypoint must still hold the frozen closed chain.
        for (std::size_t k = 1; feasible && k + 1 < candidate.size(); ++k)
        {
          DualPose pose;
          feasible = probeAt(candidate[k], probe, pose);
          if (feasible)
          {
            const auto e = dualError(pose, left_targets[k], right_targets[k]);
            feasible = e.head<3>().norm() <= options.pos_tol &&
                       e.tail<3>().norm() <= options.rot_tol;
          }
        }
        if (!feasible) rejected_by_projection = true;
      }
      if (feasible && validate)
      {
        feasible = validate(candidate, detail);
        if (!feasible)
        {
          rejected_by_feasibility = true;
          last_rejection = detail;
        }
      }
      if (!feasible) continue;

      ObjectiveValue value;
      if (!evaluateObjective(candidate, path_params, seed_path, options.w2, s2, options.w1, s1,
                             options.w0, s0, value, error))
      {
        continue;
      }
      if (value.total < current_value.total -
                               options.min_relative_decrease * std::abs(current_value.total))
      {
        accepted = true;
        accepted_value = value.total;
        accepted_path = std::move(candidate);
        break;
      }
    }

    if (!accepted)
    {
      diag.iterations = iteration + 1;
      if (rejected_by_projection && !rejected_by_feasibility)
      {
        diag.status = "projection_limited";
        diag.detail = "no trial step kept every waypoint inside the IK residual tolerance";
      }
      else
      {
        diag.status = "feasibility_limited";
        diag.detail = last_rejection.empty()
                          ? std::string("no trial step passed the path feasibility checks")
                          : std::string("path feasibility: ") + last_rejection;
      }
      break;
    }

    current = std::move(accepted_path);
    current_value.total = accepted_value;
    ++diag.accepted_steps;
    diag.iterations = iteration + 1;
    if (!evaluateObjective(current, path_params, seed_path, options.w2, s2, options.w1, s1,
                           options.w0, s0, current_value, error))
    {
      diag.status = "invalid_input";
      diag.detail = error;
      return result;
    }
  }

  if (diag.status.empty())
  {
    diag.status = diag.accepted_steps > 0 ? "improved" : "stationary";
    if (diag.status == "improved")
    {
      diag.detail = "iteration budget exhausted with accepted steps";
    }
  }

  diag.final = current_value;
  diag.accepted = diag.accepted_steps > 0 && diag.final.total < diag.initial.total;
  if (!diag.accepted)
  {
    result.path = seed_path;
  }
  else
  {
    result.path = current;
  }
  return result;
}

}  // namespace cpo
