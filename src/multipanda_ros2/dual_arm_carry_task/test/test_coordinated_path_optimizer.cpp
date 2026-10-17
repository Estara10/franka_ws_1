#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <dual_arm_carry_task/coordinated_path_optimizer.hpp>

// Pure-math coverage only. The transport-segment run against the real dual-arm
// model is a separate offline check; these tests do not replace it.

namespace
{
constexpr int kVariables = 14;

cpo::Transform poseFrom(const Eigen::Vector3d& p, const Eigen::Vector3d& rot)
{
  cpo::Transform t = cpo::Transform::Identity();
  t.translation() = p;
  const double angle = rot.norm();
  const Eigen::Vector3d axis = angle > 1e-12 ? Eigen::Vector3d(rot / angle)
                                             : Eigen::Vector3d::UnitZ();
  t.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  return t;
}

// A deliberately simple 14-DoF stand-in: six pose constraints per arm leave one
// redundant joint per arm, so the stacked task Jacobian has rank 12 and a
// two-dimensional null space.
bool toyProbe(const cpo::JointVector& q, cpo::DualPose& out)
{
  if (q.size() != kVariables || !q.allFinite()) return false;
  out.left = poseFrom(Eigen::Vector3d(q[0], q[1], q[2]), Eigen::Vector3d(q[3], q[4], q[5]));
  out.right = poseFrom(Eigen::Vector3d(q[7], q[8], q[9]), Eigen::Vector3d(q[10], q[11], q[12]));
  return true;
}

cpo::JointVector baseConfiguration()
{
  cpo::JointVector q = cpo::JointVector::Zero(kVariables);
  for (int j = 0; j < kVariables; ++j) q[j] = 0.01 * (j + 1);
  return q;
}

std::vector<cpo::JointVector> zigzagPath(int points, double amplitude)
{
  std::vector<cpo::JointVector> path;
  for (int k = 0; k < points; ++k)
  {
    cpo::JointVector q = baseConfiguration();
    q[6] += (k % 2 == 0 ? 1.0 : -1.0) * amplitude;
    q[13] += (k % 2 == 0 ? -1.0 : 1.0) * amplitude * 0.5;
    path.push_back(q);
  }
  return path;
}

std::vector<double> uniformParams(int points)
{
  std::vector<double> s;
  for (int k = 0; k < points; ++k) s.push_back(static_cast<double>(k) / (points - 1));
  return s;
}

std::vector<cpo::Transform> targetsFor(const std::vector<cpo::JointVector>& path, bool left)
{
  std::vector<cpo::Transform> targets;
  for (const auto& q : path)
  {
    cpo::DualPose pose;
    toyProbe(q, pose);
    targets.push_back(left ? pose.left : pose.right);
  }
  return targets;
}

double maxConstraintResidual(const std::vector<cpo::JointVector>& path,
                             const std::vector<cpo::Transform>& left,
                             const std::vector<cpo::Transform>& right)
{
  double worst = 0.0;
  for (std::size_t k = 0; k < path.size(); ++k)
  {
    cpo::DualPose pose;
    if (!toyProbe(path[k], pose)) return std::numeric_limits<double>::infinity();
    worst = std::max(worst, (pose.left.translation() - left[k].translation()).norm());
    worst = std::max(worst, (pose.right.translation() - right[k].translation()).norm());
  }
  return worst;
}
}  // namespace

TEST(CoordinatedPathOptimizer, ObjectiveUsesPathParameterIntervals)
{
  // Two waypoints, one interval of width 0.5: A1 = ||dq||^2/0.5 = 2*||dq||^2.
  std::vector<cpo::JointVector> path(3, cpo::JointVector::Zero(2));
  path[1] = (cpo::JointVector(2) << 0.1, 0.0).finished();
  path[2] = (cpo::JointVector(2) << 0.2, 0.0).finished();
  const std::vector<double> s{0.0, 0.25, 1.0};
  const auto seed = path;

  cpo::ObjectiveValue value;
  std::string error;
  ASSERT_TRUE(cpo::evaluateObjective(path, s, seed, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, value, error))
      << error;
  // A1 = (0.1^2)/0.25 + (0.1^2)/0.75
  EXPECT_NEAR(value.raw.a1, 0.01 / 0.25 + 0.01 / 0.75, 1e-12);
  // The samples are equally spaced in the joint value but NOT in s, so the
  // second derivative is not zero:
  //   d2 = ((0.2-0.1)/0.75 - (0.1-0)/0.25) / 0.5
  EXPECT_NEAR(value.raw.a2, 0.5 * std::pow((0.1 / 0.75 - 0.1 / 0.25) / 0.5, 2), 1e-12);
  EXPECT_NEAR(value.raw.a0, 0.0, 1e-15);

  // A non-uniform grid must not be evaluated as if it were uniform: the same
  // samples on an equal-spacing grid give a different A1.
  const std::vector<double> even{0.0, 0.5, 1.0};
  cpo::ObjectiveValue even_value;
  ASSERT_TRUE(
      cpo::evaluateObjective(path, even, seed, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, even_value, error))
      << error;
  EXPECT_NEAR(even_value.raw.a1, 0.01 / 0.5 + 0.01 / 0.5, 1e-12);
  EXPECT_GT(std::abs(even_value.raw.a1 - value.raw.a1), 1e-6);
}

TEST(CoordinatedPathOptimizer, NonUniformGridStraightLineHasNoSecondDerivative)
{
  // q(s) = a*s + b is straight in the path parameter. On a non-uniform grid the
  // plain second difference q[k+1]-2q[k]+q[k-1] is not zero, so this separates a
  // divided-difference stencil from a plain one.
  const std::vector<double> s{0.0, 0.05, 0.2, 0.21, 0.6, 1.0};
  std::vector<cpo::JointVector> path;
  for (const double sv : s)
  {
    cpo::JointVector q(2);
    q << 0.3 * sv + 0.1, -0.2 * sv + 0.5;
    path.push_back(q);
  }
  cpo::ObjectiveValue value;
  std::string error;
  ASSERT_TRUE(cpo::evaluateObjective(path, s, path, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, value, error))
      << error;
  EXPECT_NEAR(value.raw.a2, 0.0, 1e-18);
  // A1 is exact for a straight line: ||a||^2 * (s_back - s_front).
  EXPECT_NEAR(value.raw.a1, (0.3 * 0.3 + 0.2 * 0.2) * 1.0, 1e-12);

  // The plain second difference the previous implementation used is not zero
  // here, which is exactly why the two stencils are not interchangeable.
  double plain = 0.0;
  for (std::size_t k = 1; k + 1 < path.size(); ++k)
  {
    plain += (path[k + 1] - 2.0 * path[k] + path[k - 1]).squaredNorm();
  }
  EXPECT_GT(plain, 1e-6);
}

TEST(CoordinatedPathOptimizer, NonUniformGridQuadraticHasExactSecondDerivative)
{
  // q(s) = a*s^2 has d2/ds^2 = 2a everywhere, and the divided-difference stencil
  // reproduces it exactly on ANY grid. A2 therefore equals ||2a||^2 * sum(h_k).
  const std::vector<double> s{0.0, 0.02, 0.15, 0.16, 0.7, 1.0};
  const Eigen::Vector2d a(0.4, -0.25);
  std::vector<cpo::JointVector> path;
  double sum_h = 0.0;
  for (const double sv : s)
  {
    cpo::JointVector q = sv * sv * a;
    path.push_back(q);
  }
  for (std::size_t k = 1; k + 1 < s.size(); ++k)
  {
    sum_h += 0.5 * ((s[k] - s[k - 1]) + (s[k + 1] - s[k]));
  }
  cpo::ObjectiveValue value;
  std::string error;
  ASSERT_TRUE(cpo::evaluateObjective(path, s, path, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, value, error))
      << error;
  const double expected = (2.0 * a).squaredNorm() * sum_h;
  EXPECT_NEAR(value.raw.a2, expected, 1e-12 * std::max(1.0, expected));
}

TEST(CoordinatedPathOptimizer, AnalyticGradientMatchesCentralDifference)
{
  const std::vector<double> s{0.0, 0.05, 0.2, 0.21, 0.6, 1.0};
  std::vector<cpo::JointVector> path;
  std::vector<cpo::JointVector> seed;
  for (std::size_t k = 0; k < s.size(); ++k)
  {
    const double sv = s[k];
    cpo::JointVector q(3);
    q << 0.05 * sv * sv + 0.01 * sv,
         0.03 * std::sin(2.0 * sv),
         0.02 * std::cos(sv);
    path.push_back(q);
    cpo::JointVector qs = q;
    qs[1] += 0.01;  // a seed different from the current path exercises A0 too
    seed.push_back(qs);
  }
  const double w2 = 1.0, w1 = 0.1, w0 = 0.01;
  const double s2 = 2.5, s1 = 0.7, s0 = 1.3;

  std::vector<cpo::JointVector> grad;
  std::string error;
  ASSERT_TRUE(cpo::objectiveGradient(path, s, seed, w2, s2, w1, s1, w0, s0, grad, error)) << error;
  ASSERT_EQ(grad.size(), path.size());
  EXPECT_EQ(grad.front(), cpo::JointVector::Zero(3));
  EXPECT_EQ(grad.back(), cpo::JointVector::Zero(3));

  const double step = 1e-6;
  for (std::size_t k = 1; k + 1 < path.size(); ++k)
  {
    for (Eigen::Index j = 0; j < path[k].size(); ++j)
    {
      auto probe = [&](double delta) {
        std::vector<cpo::JointVector> shifted = path;
        shifted[k][j] += delta;
        cpo::ObjectiveValue value;
        std::string e;
        EXPECT_TRUE(cpo::evaluateObjective(shifted, s, seed, w2, s2, w1, s1, w0, s0, value, e))
            << e;
        return value.total;
      };
      const double numeric = (probe(step) - probe(-step)) / (2.0 * step);
      EXPECT_NEAR(grad[k][j], numeric, 1e-6 * std::max(1.0, std::abs(numeric)))
          << "waypoint " << k << " joint " << j;
    }
  }
}

TEST(CoordinatedPathOptimizer, UniformGridNormalisedSecondTermIsStencilInvariant)
{
  // On a uniform grid the divided-difference stencil scales the plain second
  // difference by 1/h^4. Both the seed scale and the current value scale the
  // same way, so the NORMALISED term is unchanged: the corrected formula does
  // not move a uniform-grid result.
  const int points = 7;
  const auto s = uniformParams(points);
  const double h = s[1] - s[0];
  const auto seed = zigzagPath(points, 0.02);
  auto path = seed;
  path[2][6] += 0.004;
  path[4][13] -= 0.003;

  cpo::ObjectiveValue value;
  std::string error;
  ASSERT_TRUE(cpo::evaluateObjective(path, s, seed, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, value, error))
      << error;
  double plain = 0.0;  // the previous implementation's A2
  for (std::size_t k = 1; k + 1 < path.size(); ++k)
  {
    plain += 0.5 * ((s[k] - s[k - 1]) + (s[k + 1] - s[k])) *
             (path[k + 1] - 2.0 * path[k] + path[k - 1]).squaredNorm();
  }
  ASSERT_GT(plain, 1e-6);
  EXPECT_NEAR(value.raw.a2 * std::pow(h, 4), plain, 1e-9 * plain);

  // Seed scales, computed the same way, keep the normalised term identical.
  cpo::ObjectiveValue seed_value;
  ASSERT_TRUE(
      cpo::evaluateObjective(seed, s, seed, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, seed_value, error))
      << error;
  double plain_seed = 0.0;
  for (std::size_t k = 1; k + 1 < seed.size(); ++k)
  {
    plain_seed += 0.5 * ((s[k] - s[k - 1]) + (s[k + 1] - s[k])) *
                  (seed[k + 1] - 2.0 * seed[k] + seed[k - 1]).squaredNorm();
  }
  ASSERT_GT(seed_value.raw.a2, 1e-12);
  ASSERT_GT(plain_seed, 1e-12);
  const double normalised_new = value.raw.a2 / seed_value.raw.a2;
  const double normalised_plain = plain / plain_seed;
  EXPECT_NEAR(normalised_new, normalised_plain, 1e-9 * std::max(1.0, normalised_plain));
}

TEST(CoordinatedPathOptimizer, SeedScalesMatchTheFrozenDefinitions)
{
  const auto seed = zigzagPath(9, 0.02);
  const auto s = uniformParams(9);
  cpo::Options options;
  double s1 = 0.0, s2 = 0.0, s0 = 0.0;
  std::string error;
  ASSERT_TRUE(cpo::seedScales(seed, s, options, s1, s2, s0, error)) << error;

  double expected_a1 = 0.0;
  for (std::size_t k = 0; k + 1 < seed.size(); ++k)
  {
    expected_a1 += (seed[k + 1] - seed[k]).squaredNorm() / (s[k + 1] - s[k]);
  }
  EXPECT_NEAR(s1, expected_a1, 1e-12);
  // S0 = variables * internal * 0.1 rad^2
  EXPECT_NEAR(s0, 14.0 * 7.0 * 0.1 * 0.1, 1e-12);
  EXPECT_GT(s2, 0.0);
}

TEST(CoordinatedPathOptimizer, ReducesSmoothnessObjectiveAndKeepsEndpoints)
{
  const auto seed = zigzagPath(9, 0.02);
  const auto s = uniformParams(9);
  const auto left = targetsFor(seed, true);
  const auto right = targetsFor(seed, false);

  cpo::Options options;
  cpo::ValidateFn validator = [](const std::vector<cpo::JointVector>&, std::string&) {
    return true;
  };
  const auto result = cpo::optimize(seed, left, right, s, toyProbe, validator, options);

  ASSERT_EQ(result.path.size(), seed.size());
  EXPECT_TRUE(result.diag.accepted) << result.diag.status << ": " << result.diag.detail;
  EXPECT_LT(result.diag.final.total, 0.8 * result.diag.initial.total);
  EXPECT_EQ(result.diag.left_rank, 6);
  EXPECT_EQ(result.diag.right_rank, 6);
  EXPECT_EQ(result.diag.rank, 12);
  EXPECT_EQ(result.diag.null_dim, 2);

  EXPECT_EQ(result.path.front(), seed.front());
  EXPECT_EQ(result.path.back(), seed.back());
  EXPECT_LT(maxConstraintResidual(result.path, left, right), 5e-4);
}

TEST(CoordinatedPathOptimizer, StraightUniformPathIsReportedStationary)
{
  // A path that is straight in the path parameter has zero second derivative.
  // The grid spacing is a binary-exact 1/8 and the joint values 1/4 apart, so
  // the divided-difference stencil sees a genuinely straight line rather than
  // the rounding residue of a decimal grid (which the frozen 1e-12 scale floor
  // would amplify). The property under test is the stopping rule, not rounding.
  std::vector<cpo::JointVector> path;
  std::vector<double> s;
  for (int k = 0; k < 9; ++k)
  {
    cpo::JointVector q = baseConfiguration();
    q[6] = 0.25 * k;
    path.push_back(q);
    s.push_back(0.125 * k);
  }
  const auto left = targetsFor(path, true);
  const auto right = targetsFor(path, false);

  cpo::ValidateFn validator = [](const std::vector<cpo::JointVector>&, std::string&) {
    return true;
  };
  const auto result = cpo::optimize(path, left, right, s, toyProbe, validator, cpo::Options{});

  EXPECT_EQ(result.diag.status, "stationary");
  EXPECT_LT(result.diag.projected_gradient_rms, 1e-6);
  EXPECT_FALSE(result.diag.accepted);
  EXPECT_EQ(result.diag.accepted_steps, 0);
  EXPECT_EQ(result.path, path);
}

TEST(CoordinatedPathOptimizer, NoInternalWaypointsIsNotApplicable)
{
  const std::vector<cpo::JointVector> path{baseConfiguration(), baseConfiguration()};
  const std::vector<double> s{0.0, 1.0};
  const std::vector<cpo::Transform> targets{poseFrom(Eigen::Vector3d::Zero(),
                                                     Eigen::Vector3d::Zero()),
                                            poseFrom(Eigen::Vector3d::Zero(),
                                                     Eigen::Vector3d::Zero())};
  cpo::ValidateFn validator = [](const std::vector<cpo::JointVector>&, std::string&) {
    return true;
  };
  const auto result = cpo::optimize(path, targets, targets, s, toyProbe, validator,
                                    cpo::Options{});

  EXPECT_EQ(result.diag.status, "not_applicable");
  EXPECT_FALSE(result.diag.accepted);
  EXPECT_EQ(result.path, path);
}

TEST(CoordinatedPathOptimizer, NonFiniteInputIsRejected)
{
  auto seed = zigzagPath(5, 0.02);
  const auto s = uniformParams(5);
  const auto left = targetsFor(seed, true);
  const auto right = targetsFor(seed, false);
  seed[2][3] = std::numeric_limits<double>::quiet_NaN();

  cpo::ValidateFn validator = [](const std::vector<cpo::JointVector>&, std::string&) {
    return true;
  };
  cpo::Options options;
  options.max_iterations = 2;
  const auto result = cpo::optimize(seed, left, right, s, toyProbe, validator, options);
  EXPECT_EQ(result.diag.status, "invalid_input");
  EXPECT_FALSE(result.diag.accepted);

  cpo::ObjectiveValue value;
  std::string error;
  EXPECT_FALSE(cpo::evaluateObjective(seed, s, seed, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, value, error));
  EXPECT_FALSE(error.empty());
}

TEST(CoordinatedPathOptimizer, NonIncreasingPathParametersAreRejected)
{
  const auto seed = zigzagPath(4, 0.02);
  const auto left = targetsFor(seed, true);
  const auto right = targetsFor(seed, false);
  const std::vector<double> s{0.0, 0.5, 0.5, 1.0};

  cpo::ValidateFn validator = [](const std::vector<cpo::JointVector>&, std::string&) {
    return true;
  };
  const auto result = cpo::optimize(seed, left, right, s, toyProbe, validator, cpo::Options{});
  EXPECT_EQ(result.diag.status, "invalid_input");
  EXPECT_EQ(result.path, seed);
}

TEST(CoordinatedPathOptimizer, RejectedCandidatesLeaveTheSeedInPlace)
{
  const auto seed = zigzagPath(7, 0.02);
  const auto s = uniformParams(7);
  const auto left = targetsFor(seed, true);
  const auto right = targetsFor(seed, false);

  int calls = 0;
  cpo::ValidateFn validator = [&calls](const std::vector<cpo::JointVector>&, std::string& d) {
    ++calls;
    d = "E_TEST_REJECTED";
    return false;
  };
  const auto result = cpo::optimize(seed, left, right, s, toyProbe, validator, cpo::Options{});

  EXPECT_EQ(result.diag.status, "feasibility_limited");
  EXPECT_FALSE(result.diag.accepted);
  EXPECT_EQ(result.diag.accepted_steps, 0);
  EXPECT_EQ(result.path, seed);
  EXPECT_GT(calls, 0);
  EXPECT_NEAR(result.diag.final.total, result.diag.initial.total, 1e-15);
}

TEST(CoordinatedPathOptimizer, ImproveIsCountedOnlyWhenTheObjectiveActuallyDrops)
{
  // A validator that always accepts, on a path whose free joint is already at
  // its optimum, must not be reported as an improvement.
  std::vector<cpo::JointVector> path(5, baseConfiguration());
  const auto s = uniformParams(5);
  const auto left = targetsFor(path, true);
  const auto right = targetsFor(path, false);

  cpo::ValidateFn validator = [](const std::vector<cpo::JointVector>&, std::string&) {
    return true;
  };
  const auto result = cpo::optimize(path, left, right, s, toyProbe, validator, cpo::Options{});
  EXPECT_FALSE(result.diag.accepted);
  EXPECT_FALSE(result.diag.accepted_steps > 0);
}
