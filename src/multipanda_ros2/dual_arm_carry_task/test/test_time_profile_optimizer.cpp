#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <dual_arm_carry_task/time_profile_optimizer.hpp>

// Pure-math coverage. The real dual-arm evaluation is a separate offline run
// against the saved transport sample; these tests do not replace it.

namespace
{
constexpr int kDim = 3;

std::vector<tpo::JointLimits> limits()
{
  std::vector<tpo::JointLimits> l(kDim);
  for (auto& x : l)
  {
    x = tpo::JointLimits{-3.0, 3.0, -2.0, 2.0, -10.0, 10.0};
  }
  return l;
}

std::vector<std::string> names()
{
  return {"j0", "j1", "j2"};
}

// A straight line in s at constant speed, sampled at five knots. The JTC
// quintic reproduces it exactly, so v is constant and a = jerk = 0.
std::vector<tpo::TrajectoryPoint> linearTrajectory()
{
  std::vector<tpo::TrajectoryPoint> pts;
  const int n = 5;
  const double T = 2.0;
  for (int k = 0; k < n; ++k)
  {
    tpo::TrajectoryPoint p;
    p.time = T * k / (n - 1);
    p.q.resize(kDim);
    p.v.resize(kDim);
    p.a = Eigen::VectorXd::Zero(kDim);
    for (int j = 0; j < kDim; ++j)
    {
      p.q[j] = 0.1 * j + 0.5 * p.time;
      p.v[j] = 0.5;
    }
    pts.push_back(p);
  }
  return pts;
}

// A non-trivial trajectory with genuine curvature in every derivative.
std::vector<tpo::TrajectoryPoint> curvedTrajectory()
{
  std::vector<tpo::TrajectoryPoint> pts;
  const int n = 7;
  const double T = 1.4;
  for (int k = 0; k < n; ++k)
  {
    const double t = T * k / (n - 1);
    tpo::TrajectoryPoint p;
    p.time = t;
    p.q.resize(kDim);
    p.v.resize(kDim);
    p.a.resize(kDim);
    for (int j = 0; j < kDim; ++j)
    {
      const double w = 1.0 + 0.4 * j;
      p.q[j] = 0.3 * std::sin(w * t) + 0.05 * j;
      p.v[j] = 0.3 * w * std::cos(w * t);
      p.a[j] = -0.3 * w * w * std::sin(w * t);
    }
    // Rest at both ends, like the real transport sample.
    if (k == 0 || k == n - 1)
    {
      p.v.setZero();
      p.a.setZero();
    }
    pts.push_back(p);
  }
  return pts;
}
}  // namespace

TEST(TimeProfileOptimizer, TimeMapIsMonotoneAndHitsBothEnds)
{
  tpo::BernsteinTimeMap map(6);
  const std::vector<Eigen::VectorXd> cases{
      Eigen::VectorXd::Zero(6),
      (Eigen::VectorXd(6) << 0.5, -0.4, 0.2, -0.1, 0.6, -0.3).finished(),
      (Eigen::VectorXd(6) << -1.0, 1.0, -1.0, 1.0, -1.0, 1.0).finished()};
  for (const auto& u : cases)
  {
    map.setParameters(u);
    EXPECT_NEAR(map.s(0.0), 0.0, 1e-15);
    EXPECT_NEAR(map.s(1.0), 1.0, 1e-12);
    double previous = -1.0;
    for (int i = 0; i <= 200; ++i)
    {
      const double tau = i / 200.0;
      const double s = map.s(tau);
      EXPECT_GE(map.dsdtau(tau), 0.0) << "tau " << tau;
      EXPECT_GE(s, previous - 1e-12) << "tau " << tau;
      previous = s;
    }
  }
}

TEST(TimeProfileOptimizer, QuinticInterpolationMatchesJtcBoundaryDerivatives)
{
  const auto points = linearTrajectory();
  tpo::TrajectoryPoint sample;
  std::string error;
  ASSERT_TRUE(tpo::interpolateQuintic(points, 0.73, sample, error)) << error;
  ASSERT_EQ(sample.q.size(), kDim);
  for (int j = 0; j < kDim; ++j)
  {
    EXPECT_NEAR(sample.q[j], 0.1 * j + 0.5 * 0.73, 1e-12);
    EXPECT_NEAR(sample.v[j], 0.5, 1e-12);
    EXPECT_NEAR(sample.a[j], 0.0, 1e-12);
  }
  EXPECT_FALSE(tpo::interpolateQuintic(points, -0.01, sample, error));
}

TEST(TimeProfileOptimizer, ChainRuleMatchesFiniteDifferences)
{
  tpo::ContinuousPath path;
  std::string error;
  ASSERT_TRUE(path.build(names(), curvedTrajectory(), error)) << error;
  const double T = path.duration();

  tpo::BernsteinTimeMap map(6);
  map.setParameters((Eigen::VectorXd(6) << 0.3, -0.2, 0.1, 0.4, -0.3, 0.2).finished());

  const std::vector<double> times{0.13 * T, 0.37 * T, 0.62 * T, 0.88 * T};
  std::vector<tpo::TrajectoryPoint> pts;
  ASSERT_TRUE(tpo::exportTrajectory(path, map, T, times, pts, error)) << error;
  ASSERT_EQ(pts.size(), times.size());

  const double eps = 1e-5;
  for (std::size_t k = 0; k < times.size(); ++k)
  {
    const double t = times[k];
    auto q_at = [&](double tt) {
      Eigen::VectorXd q, qs, qss, qsss;
      std::string e;
      EXPECT_TRUE(path.evaluate(map.s(tt / T), q, qs, qss, qsss, e)) << e;
      return q;
    };
    const Eigen::VectorXd v_fd = (q_at(t + eps) - q_at(t - eps)) / (2.0 * eps);
    for (int j = 0; j < kDim; ++j)
    {
      EXPECT_NEAR(pts[k].v[j], v_fd[j], 1e-5 * std::max(1.0, std::abs(pts[k].v[j])))
          << "velocity joint " << j << " at t=" << t;
    }
  }

  // The jerk formula is checked against a finite difference of the analytic
  // acceleration, so a missing chain-rule term cannot cancel out.
  tpo::TrajectoryMetrics metrics;
  ASSERT_TRUE(tpo::evaluateContinuous(path, map, T, limits(), 1e-3, metrics, error)) << error;
  EXPECT_GT(metrics.jerk_rms, 0.0);
  EXPECT_GT(metrics.acceleration_rms, 0.0);

  for (const double tau : {0.2, 0.5, 0.8})
  {
    auto a_at = [&](double tt, Eigen::VectorXd& a) {
      const double sp = map.dsdtau(tt / T);
      const double spp = map.ddsdtau(tt / T);
      Eigen::VectorXd q, qs, qss, qsss;
      std::string e;
      EXPECT_TRUE(path.evaluate(map.s(tt / T), q, qs, qss, qsss, e)) << e;
      a = qss * (sp / T) * (sp / T) + qs * (spp / (T * T));
    };
    Eigen::VectorXd a0, a1;
    a_at(tau * T - eps, a0);
    a_at(tau * T + eps, a1);
    const Eigen::VectorXd j_fd = (a1 - a0) / (2.0 * eps);

    const double sp = map.dsdtau(tau);
    const double spp = map.ddsdtau(tau);
    const double sppp = map.dddsdtau(tau);
    Eigen::VectorXd q, qs, qss, qsss;
    std::string e;
    ASSERT_TRUE(path.evaluate(map.s(tau), q, qs, qss, qsss, e)) << e;
    const Eigen::VectorXd sd = qs * (sp / T);
    const Eigen::VectorXd sjerk = qsss * std::pow(sp / T, 3) +
                                  3.0 * (qss * (sp / T) * (spp / (T * T))) +
                                  qs * (sppp / (T * T * T));
    (void)sd;
    for (int j = 0; j < kDim; ++j)
    {
      EXPECT_NEAR(sjerk[j], j_fd[j], 1e-3 * std::max(1.0, std::abs(sjerk[j])))
          << "jerk joint " << j << " at tau=" << tau;
    }
  }
}

TEST(TimeProfileOptimizer, LinearMapReproducesTheBaselineExactly)
{
  tpo::ContinuousPath path;
  std::string error;
  const auto baseline = curvedTrajectory();
  ASSERT_TRUE(path.build(names(), baseline, error)) << error;

  tpo::BernsteinTimeMap map(6);
  map.setParameters(Eigen::VectorXd::Zero(6));  // c_i = 1 -> w = 1 -> s = tau

  std::vector<tpo::TrajectoryPoint> pts;
  ASSERT_TRUE(tpo::exportTrajectory(path, map, path.duration(), path.knotTimes(), pts, error))
      << error;
  ASSERT_EQ(pts.size(), baseline.size());
  for (std::size_t k = 0; k < baseline.size(); ++k)
  {
    EXPECT_NEAR(pts[k].time, baseline[k].time, 1e-12);
    for (int j = 0; j < kDim; ++j)
    {
      EXPECT_NEAR(pts[k].q[j], baseline[k].q[j], 1e-12) << "q point " << k;
      EXPECT_NEAR(pts[k].v[j], baseline[k].v[j], 1e-12) << "v point " << k;
      EXPECT_NEAR(pts[k].a[j], baseline[k].a[j], 1e-12) << "a point " << k;
    }
  }
}

TEST(TimeProfileOptimizer, StraightLineHasZeroAccelerationAndJerk)
{
  // Because the JTC quintic reproduces a linear segment exactly, the exported
  // and re-interpolated trajectory of a constant-speed line has a = jerk = 0.
  tpo::ContinuousPath path;
  std::string error;
  ASSERT_TRUE(path.build(names(), linearTrajectory(), error)) << error;
  tpo::BernsteinTimeMap map(6);

  std::vector<tpo::TrajectoryPoint> pts;
  ASSERT_TRUE(tpo::exportTrajectory(path, map, path.duration(), path.knotTimes(), pts, error))
      << error;
  tpo::TrajectoryMetrics metrics;
  ASSERT_TRUE(tpo::evaluateExported(pts, limits(), 1e-3, metrics, error)) << error;
  EXPECT_NEAR(metrics.acceleration_rms, 0.0, 1e-12);
  EXPECT_NEAR(metrics.jerk_rms, 0.0, 1e-12);
  EXPECT_NEAR(metrics.jerk_max, 0.0, 1e-12);
  EXPECT_FALSE(metrics.velocity_violation);
  EXPECT_FALSE(metrics.acceleration_violation);
}

TEST(TimeProfileOptimizer, InvalidTrajectoriesAndExportsAreRejected)
{
  tpo::ContinuousPath path;
  std::string error;

  auto bad = curvedTrajectory();
  bad[2].time = bad[1].time;  // not strictly increasing
  EXPECT_FALSE(path.build(names(), bad, error));
  EXPECT_FALSE(error.empty());

  auto ragged = curvedTrajectory();
  ragged[1].v.resize(kDim - 1);
  EXPECT_FALSE(path.build(names(), ragged, error));

  auto nonfinite = curvedTrajectory();
  nonfinite[3].q[0] = std::nan("");
  EXPECT_FALSE(path.build(names(), nonfinite, error));

  EXPECT_FALSE(path.build({"only_one"}, curvedTrajectory(), error));

  ASSERT_TRUE(path.build(names(), curvedTrajectory(), error)) << error;
  tpo::BernsteinTimeMap map(6);
  std::vector<tpo::TrajectoryPoint> pts;

  std::vector<double> decreasing{0.0, 0.4, 0.2};
  EXPECT_FALSE(tpo::exportTrajectory(path, map, path.duration(), decreasing, pts, error));
  EXPECT_FALSE(error.empty());

  std::vector<double> short_times{0.1};
  EXPECT_FALSE(tpo::exportTrajectory(path, map, path.duration(), short_times, pts, error));

  EXPECT_FALSE(tpo::exportTrajectory(path, map, 0.0, path.knotTimes(), pts, error));

  // Non-finite map parameters fall back to the linear clock rather than
  // producing a non-monotone map.
  Eigen::VectorXd nan_params = Eigen::VectorXd::Constant(6, std::nan(""));
  map.setParameters(nan_params);
  EXPECT_NEAR(map.s(0.5), 0.5, 1e-12);
}

TEST(TimeProfileOptimizer, AcceptanceRequiresContinuousRmsPeakAndFidelity)
{
  tpo::TrajectoryMetrics baseline;
  baseline.jerk_rms = 10.0;
  baseline.jerk_max = 20.0;
  tpo::TrajectoryMetrics candidate = baseline;

  candidate.jerk_rms = 9.0;
  candidate.jerk_max = 21.0;
  auto acceptance = tpo::evaluateTimeProfileAcceptance(baseline, candidate, true);
  EXPECT_TRUE(acceptance.rms_improved);
  EXPECT_FALSE(acceptance.peak_ok);
  EXPECT_FALSE(acceptance.accepted);
  EXPECT_NE(acceptance.fallback_reason.find("continuous_peak_increased"), std::string::npos);

  candidate.jerk_max = 19.0;
  acceptance = tpo::evaluateTimeProfileAcceptance(baseline, candidate, false);
  EXPECT_TRUE(acceptance.peak_ok);
  EXPECT_FALSE(acceptance.accepted);
  EXPECT_NE(acceptance.fallback_reason.find("export_fidelity_exceeded"), std::string::npos);

  acceptance = tpo::evaluateTimeProfileAcceptance(baseline, candidate, true);
  EXPECT_TRUE(acceptance.accepted);
  EXPECT_TRUE(acceptance.fallback_reason.empty());

  candidate.jerk_rms = 11.0;
  acceptance = tpo::evaluateTimeProfileAcceptance(baseline, candidate, true);
  EXPECT_FALSE(acceptance.rms_improved);
  EXPECT_FALSE(acceptance.accepted);
  EXPECT_NE(acceptance.fallback_reason.find("continuous_rms_not_reduced"), std::string::npos);
}

TEST(TimeProfileOptimizer, FaithfulExportPreservesMappedSegmentBoundaries)
{
  tpo::ContinuousPath path;
  std::string error;
  ASSERT_TRUE(path.build(names(), curvedTrajectory(), error)) << error;

  tpo::BernsteinTimeMap map(6);
  map.setParameters((Eigen::VectorXd(6) << 0.3, -0.2, 0.1, 0.4, -0.3, 0.2).finished());
  std::vector<tpo::TrajectoryPoint> points;
  ASSERT_TRUE(tpo::buildFaithfulExport(path, map, path.duration(), 0.005, points, error))
      << error;
  ASSERT_GE(points.size(), path.knotTimes().size());
  EXPECT_NEAR(points.front().time, 0.0, 1e-15);
  EXPECT_NEAR(points.back().time, path.duration(), 1e-15);

  const auto knots = path.knotTimes();
  for (const double knot : knots)
  {
    const double expected_time = path.duration() * map.inverseS(knot / path.duration());
    const auto found = std::find_if(points.begin(), points.end(), [=](const auto& point) {
      return std::abs(point.time - expected_time) <= 1e-12;
    });
    EXPECT_NE(found, points.end()) << "missing mapped boundary " << expected_time;
  }
}

TEST(TimeProfileOptimizer, FaithfulExportHandlesNonGridEndpoint)
{
  tpo::ContinuousPath path;
  std::string error;
  ASSERT_TRUE(path.build(names(), curvedTrajectory(), error)) << error;
  tpo::BernsteinTimeMap map(6);
  std::vector<tpo::TrajectoryPoint> points;
  const double duration = 1.234567;
  ASSERT_TRUE(tpo::buildFaithfulExport(path, map, duration, 0.005, points, error))
      << error;
  ASSERT_GE(points.size(), 2u);
  EXPECT_DOUBLE_EQ(points.front().time, 0.0);
  EXPECT_DOUBLE_EQ(points.back().time, duration);
}

TEST(TimeProfileOptimizer, ContinuousJerkScalesUnderUniformTimeStretch)
{
  tpo::ContinuousPath path;
  std::string error;
  ASSERT_TRUE(path.build(names(), curvedTrajectory(), error)) << error;
  tpo::BernsteinTimeMap map(6);
  map.setParameters(Eigen::VectorXd::Zero(6));

  tpo::TrajectoryMetrics original;
  tpo::TrajectoryMetrics stretched;
  ASSERT_TRUE(tpo::evaluateContinuous(path, map, path.duration(), limits(), 0.001, original, error))
      << error;
  ASSERT_TRUE(tpo::evaluateContinuous(path, map, 1.2 * path.duration(), limits(), 0.001, stretched,
                                      error))
      << error;
  ASSERT_GT(original.jerk_rms, 0.0);
  ASSERT_GT(original.jerk_max, 0.0);
  const double scale = 1.0 / std::pow(1.2, 3.0);
  // Both curves use the required 1 ms evaluation interval; the two resulting
  // time grids sample a varying jerk signal at slightly different tau points.
  EXPECT_NEAR(stretched.jerk_rms / original.jerk_rms, scale, 2e-3);
  EXPECT_NEAR(stretched.jerk_max / original.jerk_max, scale, 2e-3);
}

TEST(TimeProfileOptimizer, OptimiserKeepsTheBaselineOrImprovesIt)
{
  tpo::ContinuousPath path;
  std::string error;
  ASSERT_TRUE(path.build(names(), curvedTrajectory(), error)) << error;

  tpo::TimeProfileOptions options;
  options.max_evaluations = 40;
  const auto result = tpo::optimizeTimeProfile(path, limits(), path.duration(), options);
  ASSERT_TRUE(result.ok) << result.detail;
  EXPECT_LE(result.evaluations, options.max_evaluations);
  EXPECT_NEAR(result.baseline_objective, result.baseline_continuous.jerk_rms, 1e-12);
  if (result.accepted)
  {
    EXPECT_LT(result.candidate_continuous.jerk_rms, result.baseline_continuous.jerk_rms);
    EXPECT_LE(result.candidate_continuous.jerk_max,
              result.baseline_continuous.jerk_max + 1e-12);
    EXPECT_NEAR(result.selected_parameters.norm(), result.parameters.norm(), 1e-12);
  }
  else
  {
    EXPECT_TRUE(result.fallback_to_baseline);
    EXPECT_TRUE(result.selected_parameters.isZero(1e-12));
    EXPECT_FALSE(result.fallback_reason.empty());
  }
  EXPECT_FALSE(result.baseline_export.empty());
  EXPECT_FALSE(result.selected_export.empty());
}

TEST(TimeProfileOptimizer, OptimiserFallsBackWhenExportCannotMeetFidelity)
{
  tpo::ContinuousPath path;
  std::string error;
  ASSERT_TRUE(path.build(names(), curvedTrajectory(), error)) << error;

  tpo::TimeProfileOptions options;
  options.max_evaluations = 40;
  options.export_fidelity_tolerance = 0.0;
  const auto result = tpo::optimizeTimeProfile(path, limits(), path.duration(), options);
  ASSERT_TRUE(result.ok) << result.detail;
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.fallback_to_baseline);
  EXPECT_FALSE(result.export_fidelity_ok);
  EXPECT_NE(result.fallback_reason.find("export_fidelity_exceeded"), std::string::npos);
  EXPECT_TRUE(result.selected_parameters.isZero(1e-12));
  EXPECT_DOUBLE_EQ(result.export_spacing_used, options.refined_export_spacing);
}
