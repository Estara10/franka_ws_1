#include "dual_arm_carry_task/time_profile_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tpo
{
namespace
{
double binomial(int n, int k)
{
  if (k < 0 || k > n) return 0.0;
  double r = 1.0;
  for (int i = 1; i <= k; ++i) r = r * (n - k + i) / static_cast<double>(i);
  return r;
}

// The JointTrajectoryController quintic between two points, exactly as it is
// solved in the task code: it reproduces q/v/a at both ends.
struct Quintic
{
  double c0{0.0}, c1{0.0}, c2{0.0}, c3{0.0}, c4{0.0}, c5{0.0};

  Quintic() = default;
  Quintic(double q0, double v0, double a0, double q1, double v1, double a1, double T)
  {
    c0 = q0;
    c1 = v0 * T;
    c2 = 0.5 * a0 * T * T;
    const double D = q1 - c0 - c1 - c2;
    const double V = v1 * T - c1 - 2.0 * c2;
    const double A = a1 * T * T - 2.0 * c2;
    c3 = 10.0 * D - 4.0 * V + 0.5 * A;
    c4 = -15.0 * D + 7.0 * V - A;
    c5 = 6.0 * D - 3.0 * V + 0.5 * A;
  }

  double q(double u) const { return c0 + u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * c5)))); }
  // First, second and third derivative with respect to REAL time, T the
  // segment length.
  double v(double u, double T) const
  {
    return (c1 + u * (2.0 * c2 + u * (3.0 * c3 + u * (4.0 * c4 + u * 5.0 * c5)))) / T;
  }
  double a(double u, double T2) const
  {
    return (2.0 * c2 + u * (6.0 * c3 + u * (12.0 * c4 + u * 20.0 * c5))) / T2;
  }
  double j(double u, double T3) const
  {
    return (6.0 * c3 + u * (24.0 * c4 + u * 60.0 * c5)) / T3;
  }
};

void accumulateMetrics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                       const Eigen::VectorXd& jerk, const std::vector<JointLimits>& limits,
                       double& acc_sq, double& jerk_sq, std::size_t& acc_n, std::size_t& jerk_n,
                       TrajectoryMetrics& out)
{
  for (Eigen::Index i = 0; i < q.size(); ++i)
  {
    const JointLimits& l = limits[static_cast<std::size_t>(i)];
    if (q[i] < l.min_position || q[i] > l.max_position) out.position_violation = true;
    if (v[i] < l.min_velocity || v[i] > l.max_velocity) out.velocity_violation = true;
    if (a[i] < l.min_acceleration || a[i] > l.max_acceleration) out.acceleration_violation = true;
    const double v_cap = std::max(std::abs(l.min_velocity), std::abs(l.max_velocity));
    const double a_cap = std::max(std::abs(l.min_acceleration), std::abs(l.max_acceleration));
    if (v_cap > 0.0) out.max_velocity_ratio = std::max(out.max_velocity_ratio, std::abs(v[i]) / v_cap);
    if (a_cap > 0.0)
      out.max_acceleration_ratio = std::max(out.max_acceleration_ratio, std::abs(a[i]) / a_cap);
    acc_sq += a[i] * a[i];
    ++acc_n;
    jerk_sq += jerk[i] * jerk[i];
    ++jerk_n;
    out.jerk_max = std::max(out.jerk_max, std::abs(jerk[i]));
  }
}
}  // namespace

bool interpolateQuintic(const std::vector<TrajectoryPoint>& points,
                         double time,
                         TrajectoryPoint& out,
                         std::string& error)
{
  if (points.size() < 2 || !std::isfinite(time))
  {
    error = "quintic interpolation needs two points and a finite time";
    return false;
  }
  if (time < points.front().time - 1e-12 || time > points.back().time + 1e-12)
  {
    error = "quintic interpolation time is outside trajectory";
    return false;
  }
  std::size_t segment = points.size() - 2;
  for (std::size_t i = 1; i < points.size(); ++i)
  {
    if (time <= points[i].time)
    {
      segment = i - 1;
      break;
    }
  }
  const auto& a = points[segment];
  const auto& b = points[segment + 1];
  const double duration = b.time - a.time;
  if (!(duration > 0.0) || a.q.size() == 0 || a.q.size() != b.q.size() ||
      a.v.size() != a.q.size() || b.v.size() != b.q.size() ||
      a.a.size() != a.q.size() || b.a.size() != b.q.size())
  {
    error = "quintic interpolation point dimensions or times are invalid";
    return false;
  }
  const double u = std::clamp((time - a.time) / duration, 0.0, 1.0);
  out.time = time;
  out.q.resize(a.q.size());
  out.v.resize(a.q.size());
  out.a.resize(a.q.size());
  for (Eigen::Index i = 0; i < a.q.size(); ++i)
  {
    const Quintic curve(a.q[i], a.v[i], a.a[i], b.q[i], b.v[i], b.a[i], duration);
    out.q[i] = curve.q(u);
    out.v[i] = curve.v(u, duration);
    out.a[i] = curve.a(u, duration * duration);
  }
  if (!out.q.allFinite() || !out.v.allFinite() || !out.a.allFinite())
  {
    error = "quintic interpolation produced non-finite values";
    return false;
  }
  return true;
}

// ---------------------------------------------------------------- time map

BernsteinTimeMap::BernsteinTimeMap(int degree)
  : degree_(std::max(2, degree)), u_(Eigen::VectorXd::Zero(std::max(2, degree)))
{
  coefficients();
}

void BernsteinTimeMap::coefficients()
{
  c_.assign(static_cast<std::size_t>(degree_ + 1), 1.0);
  for (int i = 1; i <= degree_; ++i) c_[static_cast<std::size_t>(i)] = std::exp(u_[i - 1]);
}

void BernsteinTimeMap::setParameters(const Eigen::VectorXd& u)
{
  u_ = (u.size() == degree_ && u.allFinite()) ? u : Eigen::VectorXd::Zero(degree_);
  coefficients();
}

double BernsteinTimeMap::coefficient(int i) const
{
  if (i < 0 || i > degree_) return 0.0;
  return c_[static_cast<std::size_t>(i)];
}

double BernsteinTimeMap::bernstein(int i, int d, double tau) const
{
  if (i < 0 || i > d) return 0.0;
  return binomial(d, i) * std::pow(tau, i) * std::pow(1.0 - tau, d - i);
}

double BernsteinTimeMap::s(double tau) const
{
  tau = std::clamp(tau, 0.0, 1.0);
  double w1 = 0.0;
  for (int i = 0; i <= degree_; ++i) w1 += c_[static_cast<std::size_t>(i)];
  w1 /= static_cast<double>(degree_ + 1);
  if (!(w1 > 1e-300)) return tau;
  double w = 0.0;
  for (int i = 0; i <= degree_; ++i)
  {
    double partial = 0.0;
    for (int j = i + 1; j <= degree_ + 1; ++j) partial += bernstein(j, degree_ + 1, tau);
    w += c_[static_cast<std::size_t>(i)] * partial / static_cast<double>(degree_ + 1);
  }
  return std::clamp(w / w1, 0.0, 1.0);
}

double BernsteinTimeMap::dsdtau(double tau) const
{
  tau = std::clamp(tau, 0.0, 1.0);
  double w = 0.0, w1 = 0.0;
  for (int i = 0; i <= degree_; ++i)
  {
    w += c_[static_cast<std::size_t>(i)] * bernstein(i, degree_, tau);
    w1 += c_[static_cast<std::size_t>(i)];
  }
  w1 /= static_cast<double>(degree_ + 1);
  return w1 > 1e-300 ? w / w1 : 1.0;
}

double BernsteinTimeMap::ddsdtau(double tau) const
{
  tau = std::clamp(tau, 0.0, 1.0);
  if (degree_ < 1) return 0.0;
  double w = 0.0, w1 = 0.0;
  for (int i = 0; i <= degree_ - 1; ++i)
  {
    w += (c_[static_cast<std::size_t>(i + 1)] - c_[static_cast<std::size_t>(i)]) *
         bernstein(i, degree_ - 1, tau);
  }
  w *= static_cast<double>(degree_);
  for (int i = 0; i <= degree_; ++i) w1 += c_[static_cast<std::size_t>(i)];
  w1 /= static_cast<double>(degree_ + 1);
  return w1 > 1e-300 ? w / w1 : 0.0;
}

double BernsteinTimeMap::dddsdtau(double tau) const
{
  tau = std::clamp(tau, 0.0, 1.0);
  if (degree_ < 2) return 0.0;
  double w = 0.0, w1 = 0.0;
  for (int i = 0; i <= degree_ - 2; ++i)
  {
    const double second = c_[static_cast<std::size_t>(i + 2)] -
                          2.0 * c_[static_cast<std::size_t>(i + 1)] +
                          c_[static_cast<std::size_t>(i)];
    w += second * bernstein(i, degree_ - 2, tau);
  }
  w *= static_cast<double>(degree_) * static_cast<double>(degree_ - 1);
  for (int i = 0; i <= degree_; ++i) w1 += c_[static_cast<std::size_t>(i)];
  w1 /= static_cast<double>(degree_ + 1);
  return w1 > 1e-300 ? w / w1 : 0.0;
}

double BernsteinTimeMap::inverseS(double s_value, double tolerance) const
{
  s_value = std::clamp(s_value, 0.0, 1.0);
  if (s_value <= 0.0) return 0.0;
  if (s_value >= 1.0) return 1.0;

  const double tol = std::max(1e-15, std::abs(tolerance));
  double lo = 0.0;
  double hi = 1.0;
  for (int iteration = 0; iteration < 80; ++iteration)
  {
    const double mid = 0.5 * (lo + hi);
    if (s(mid) < s_value)
      lo = mid;
    else
      hi = mid;
  }
  const double tau = 0.5 * (lo + hi);
  return std::abs(s(tau) - s_value) <= tol ? tau : std::clamp(tau, 0.0, 1.0);
}

// ------------------------------------------------------------ continuous path

bool ContinuousPath::build(const std::vector<std::string>& joint_names,
                           const std::vector<TrajectoryPoint>& points,
                           std::string& error)
{
  error.clear();
  valid_ = false;
  if (points.size() < 2)
  {
    error = "need at least two trajectory points";
    return false;
  }
  const auto dim = static_cast<std::size_t>(points.front().q.size());
  if (joint_names.size() != dim || dim == 0)
  {
    error = "joint names do not match the point dimension";
    return false;
  }
  for (std::size_t k = 0; k < points.size(); ++k)
  {
    const auto& p = points[k];
    if (p.q.size() != points.front().q.size() || p.v.size() != points.front().q.size() ||
        p.a.size() != points.front().q.size())
    {
      error = "ragged trajectory derivatives at point " + std::to_string(k);
      return false;
    }
    if (!p.q.allFinite() || !p.v.allFinite() || !p.a.allFinite() || !std::isfinite(p.time))
    {
      error = "non-finite trajectory value at point " + std::to_string(k);
      return false;
    }
    if (k > 0 && !(p.time > points[k - 1].time))
    {
      error = "time_from_start is not strictly increasing at point " + std::to_string(k);
      return false;
    }
  }
  joint_names_ = joint_names;
  points_ = points;
  dimension_ = points.front().q.size();
  duration_ = points.back().time - points.front().time;
  if (!(duration_ > 0.0))
  {
    error = "trajectory duration must be positive";
    return false;
  }
  valid_ = true;
  return true;
}

std::vector<double> ContinuousPath::knotTimes() const
{
  std::vector<double> t;
  t.reserve(points_.size());
  const double t0 = points_.empty() ? 0.0 : points_.front().time;
  for (const auto& p : points_) t.push_back(p.time - t0);
  return t;
}

std::size_t ContinuousPath::segmentFor(double s) const
{
  if (s <= 0.0) return 1;
  for (std::size_t k = 1; k + 1 < points_.size(); ++k)
  {
    if (s <= (points_[k].time - points_.front().time) / duration_) return k;
  }
  return points_.size() - 1;
}

bool ContinuousPath::evaluate(double s, Eigen::VectorXd& q, Eigen::VectorXd& qs,
                              Eigen::VectorXd& qss, Eigen::VectorXd& qsss,
                              std::string& error) const
{
  error.clear();
  if (!valid_)
  {
    error = "path is not built";
    return false;
  }
  if (!std::isfinite(s))
  {
    error = "path parameter is not finite";
    return false;
  }
  s = std::clamp(s, 0.0, 1.0);
  const std::size_t k = segmentFor(s);
  const auto& p0 = points_[k - 1];
  const auto& p1 = points_[k];
  const double T = p1.time - p0.time;
  const double t = s * duration_ + points_.front().time;
  double u = (t - p0.time) / T;
  u = std::clamp(u, 0.0, 1.0);

  q.resize(dimension_);
  qs.resize(dimension_);
  qss.resize(dimension_);
  qsss.resize(dimension_);
  for (Eigen::Index j = 0; j < dimension_; ++j)
  {
    const Quintic quint(p0.q[j], p0.v[j], p0.a[j], p1.q[j], p1.v[j], p1.a[j], T);
    q[j] = quint.q(u);
    qs[j] = quint.v(u, T) * duration_;
    qss[j] = quint.a(u, T * T) * duration_ * duration_;
    qsss[j] = quint.j(u, T * T * T) * duration_ * duration_ * duration_;
  }
  return true;
}

// ------------------------------------------------------------------ export

bool exportTrajectory(const ContinuousPath& path, const BernsteinTimeMap& map, double duration,
                      const std::vector<double>& times, std::vector<TrajectoryPoint>& out,
                      std::string& error)
{
  error.clear();
  out.clear();
  if (!path.valid())
  {
    error = "path is not built";
    return false;
  }
  if (!(duration > 0.0) || !std::isfinite(duration))
  {
    error = "duration must be positive and finite";
    return false;
  }
  if (times.size() < 2)
  {
    error = "need at least two export times";
    return false;
  }
  for (std::size_t k = 1; k < times.size(); ++k)
  {
    if (!(times[k] > times[k - 1]) || !std::isfinite(times[k]))
    {
      error = "export times must be finite and strictly increasing";
      return false;
    }
  }
  out.reserve(times.size());
  for (const double t : times)
  {
    const double tau = std::clamp(t / duration, 0.0, 1.0);
    const double sp = map.dsdtau(tau);
    const double spp = map.ddsdtau(tau);
    Eigen::VectorXd q, qs, qss, qsss;
    if (!path.evaluate(map.s(tau), q, qs, qss, qsss, error)) return false;
    TrajectoryPoint point;
    point.time = t;
    point.q = q;
    point.v = qs * (sp / duration);
    point.a = qss * (sp / duration) * (sp / duration) + qs * (spp / (duration * duration));
    if (!point.v.allFinite() || !point.a.allFinite())
    {
      error = "chain-rule derivative is not finite";
      return false;
    }
    out.push_back(std::move(point));
  }
  return true;
}

bool buildFaithfulExport(const ContinuousPath& path,
                         const BernsteinTimeMap& map,
                         double duration,
                         double export_spacing,
                         std::vector<TrajectoryPoint>& out,
                         std::string& error)
{
  error.clear();
  out.clear();
  if (!path.valid())
  {
    error = "path is not built";
    return false;
  }
  if (!(duration > 0.0) || !std::isfinite(duration))
  {
    error = "duration must be positive and finite";
    return false;
  }
  if (!(export_spacing > 0.0) || !std::isfinite(export_spacing))
  {
    error = "export spacing must be positive and finite";
    return false;
  }

  const int uniform_steps =
      std::max(1, static_cast<int>(std::ceil(duration / export_spacing)));
  std::vector<double> times;
  times.reserve(static_cast<std::size_t>(uniform_steps) + path.segmentCount() + 1);
  for (int i = 0; i <= uniform_steps; ++i)
  {
    times.push_back(duration * static_cast<double>(i) / static_cast<double>(uniform_steps));
  }
  times.front() = 0.0;
  times.back() = duration;

  const double path_duration = path.duration();
  for (const double knot_time : path.knotTimes())
  {
    const double s_knot = std::clamp(knot_time / path_duration, 0.0, 1.0);
    const double mapped_time = duration * map.inverseS(s_knot);
    times.push_back(std::clamp(mapped_time, 0.0, duration));
  }
  std::sort(times.begin(), times.end());
  const double duplicate_tolerance = std::max(1e-14, duration * 1e-12);
  times.erase(std::unique(times.begin(), times.end(),
                          [duplicate_tolerance](double a, double b) {
                            return std::abs(a - b) <= duplicate_tolerance;
                          }),
              times.end());
  times.front() = 0.0;
  times.back() = duration;
  if (times.size() < 2 || times.front() != 0.0 || times.back() != duration)
  {
    error = "faithful export grid did not retain both trajectory endpoints";
    return false;
  }
  return exportTrajectory(path, map, duration, times, out, error);
}

// ---------------------------------------------------------------- evaluation

bool evaluateExported(const std::vector<TrajectoryPoint>& points,
                      const std::vector<JointLimits>& limits, double interpolation_dt,
                      TrajectoryMetrics& out, std::string& error)
{
  error.clear();
  out = TrajectoryMetrics{};
  if (points.size() < 2)
  {
    error = "need at least two points";
    return false;
  }
  if (limits.size() != static_cast<std::size_t>(points.front().q.size()))
  {
    error = "limit count does not match the trajectory dimension";
    return false;
  }
  if (!(interpolation_dt > 0.0) || !std::isfinite(interpolation_dt))
  {
    error = "interpolation dt must be positive";
    return false;
  }
  out.duration = points.back().time - points.front().time;
  double acc_sq = 0.0, jerk_sq = 0.0;
  std::size_t acc_n = 0, jerk_n = 0;
  const Eigen::Index dim = points.front().q.size();
  for (std::size_t k = 1; k < points.size(); ++k)
  {
    const auto& p0 = points[k - 1];
    const auto& p1 = points[k];
    const double T = p1.time - p0.time;
    if (!(T > 0.0))
    {
      error = "non-positive segment duration";
      return false;
    }
    const int n = std::max(2, static_cast<int>(std::ceil(T / interpolation_dt)));
    for (int sample = 0; sample <= n; ++sample)
    {
      const double u = static_cast<double>(sample) / n;
      Eigen::VectorXd q(dim), v(dim), a(dim), jk(dim);
      for (Eigen::Index j = 0; j < dim; ++j)
      {
        const Quintic quint(p0.q[j], p0.v[j], p0.a[j], p1.q[j], p1.v[j], p1.a[j], T);
        q[j] = quint.q(u);
        v[j] = quint.v(u, T);
        a[j] = quint.a(u, T * T);
        jk[j] = quint.j(u, T * T * T);
      }
      if (!q.allFinite() || !v.allFinite() || !a.allFinite() || !jk.allFinite())
      {
        error = "non-finite interpolated value";
        return false;
      }
      accumulateMetrics(q, v, a, jk, limits, acc_sq, jerk_sq, acc_n, jerk_n, out);
      ++out.samples;
    }
  }
  out.acceleration_rms = acc_n > 0 ? std::sqrt(acc_sq / static_cast<double>(acc_n)) : 0.0;
  out.jerk_rms = jerk_n > 0 ? std::sqrt(jerk_sq / static_cast<double>(jerk_n)) : 0.0;
  return true;
}

bool evaluateContinuous(const ContinuousPath& path, const BernsteinTimeMap& map, double duration,
                        const std::vector<JointLimits>& limits, double sample_dt,
                        TrajectoryMetrics& out, std::string& error)
{
  error.clear();
  out = TrajectoryMetrics{};
  if (!path.valid())
  {
    error = "path is not built";
    return false;
  }
  if (!(duration > 0.0) || !std::isfinite(duration) || !(sample_dt > 0.0))
  {
    error = "invalid duration or sample dt";
    return false;
  }
  if (limits.size() != static_cast<std::size_t>(path.dimension()))
  {
    error = "limit count does not match the path dimension";
    return false;
  }
  const int n = std::max(2, static_cast<int>(std::ceil(duration / sample_dt)));
  double acc_sq = 0.0, jerk_sq = 0.0;
  std::size_t acc_n = 0, jerk_n = 0;
  for (int sample = 0; sample <= n; ++sample)
  {
    const double tau = static_cast<double>(sample) / n;
    const double sp = map.dsdtau(tau);
    const double spp = map.ddsdtau(tau);
    const double sppp = map.dddsdtau(tau);
    Eigen::VectorXd q, qs, qss, qsss;
    if (!path.evaluate(map.s(tau), q, qs, qss, qsss, error)) return false;
    const double sd = sp / duration;
    const double sdd = spp / (duration * duration);
    const double sddd = sppp / (duration * duration * duration);
    const Eigen::VectorXd v = qs * sd;
    const Eigen::VectorXd a = qss * sd * sd + qs * sdd;
    const Eigen::VectorXd jk = qsss * sd * sd * sd + 3.0 * (qss * sd * sdd) + qs * sddd;
    if (!v.allFinite() || !a.allFinite() || !jk.allFinite())
    {
      error = "non-finite continuous derivative";
      return false;
    }
    accumulateMetrics(q, v, a, jk, limits, acc_sq, jerk_sq, acc_n, jerk_n, out);
    ++out.samples;
  }
  out.duration = duration;
  out.acceleration_rms = acc_n > 0 ? std::sqrt(acc_sq / static_cast<double>(acc_n)) : 0.0;
  out.jerk_rms = jerk_n > 0 ? std::sqrt(jerk_sq / static_cast<double>(jerk_n)) : 0.0;
  return true;
}

bool maxPositionDeviation(const std::vector<TrajectoryPoint>& points, const ContinuousPath& path,
                          const BernsteinTimeMap& map, double duration, double sample_dt,
                          double& max_deviation, std::string& error)
{
  error.clear();
  max_deviation = 0.0;
  if (points.size() < 2)
  {
    error = "need at least two points";
    return false;
  }
  if (!(sample_dt > 0.0) || !std::isfinite(sample_dt) || !(duration > 0.0))
  {
    error = "invalid duration or sample dt";
    return false;
  }
  for (std::size_t k = 1; k < points.size(); ++k)
  {
    const auto& p0 = points[k - 1];
    const auto& p1 = points[k];
    const double T = p1.time - p0.time;
    if (!(T > 0.0))
    {
      error = "non-positive segment duration";
      return false;
    }
    const int n = std::max(2, static_cast<int>(std::ceil(T / sample_dt)));
    for (int sample = 0; sample <= n; ++sample)
    {
      const double u = static_cast<double>(sample) / n;
      const double t = p0.time + u * T;
      Eigen::VectorXd q, qs, qss, qsss;
      if (!path.evaluate(map.s(t / duration), q, qs, qss, qsss, error)) return false;
      for (Eigen::Index j = 0; j < q.size(); ++j)
      {
        const Quintic quint(p0.q[j], p0.v[j], p0.a[j], p1.q[j], p1.v[j], p1.a[j], T);
        max_deviation = std::max(max_deviation, std::abs(quint.q(u) - q[j]));
      }
    }
  }
  return true;
}

TimeProfileAcceptance evaluateTimeProfileAcceptance(
    const TrajectoryMetrics& baseline_continuous,
    const TrajectoryMetrics& candidate_continuous,
    bool export_fidelity_ok)
{
  TimeProfileAcceptance acceptance;
  acceptance.constraints_ok =
      !candidate_continuous.position_violation &&
      !candidate_continuous.velocity_violation &&
      !candidate_continuous.acceleration_violation &&
      candidate_continuous.max_velocity_ratio <= 1.0 + 1e-9 &&
      candidate_continuous.max_acceleration_ratio <= 1.0 + 1e-9;
  acceptance.rms_improved =
      candidate_continuous.jerk_rms < baseline_continuous.jerk_rms;
  const double peak_tolerance = 1e-12 * std::max(1.0, baseline_continuous.jerk_max);
  acceptance.peak_ok =
      candidate_continuous.jerk_max <= baseline_continuous.jerk_max + peak_tolerance;
  acceptance.accepted =
      acceptance.constraints_ok && acceptance.rms_improved && acceptance.peak_ok &&
      export_fidelity_ok;

  if (!acceptance.constraints_ok) acceptance.fallback_reason += "constraints_not_satisfied;";
  if (!acceptance.rms_improved) acceptance.fallback_reason += "continuous_rms_not_reduced;";
  if (!acceptance.peak_ok) acceptance.fallback_reason += "continuous_peak_increased;";
  if (!export_fidelity_ok) acceptance.fallback_reason += "export_fidelity_exceeded;";
  if (!acceptance.fallback_reason.empty()) acceptance.fallback_reason.pop_back();
  return acceptance;
}

// --------------------------------------------------------------- optimisation
TimeProfileResult optimizeTimeProfile(const ContinuousPath& path,
                                      const std::vector<JointLimits>& limits, double duration,
                                      const TimeProfileOptions& options)
{
  TimeProfileResult result;
  result.total_duration = duration;
  if (!path.valid())
  {
    result.status = "invalid_input";
    result.detail = "path is not built";
    return result;
  }
  if (limits.size() != static_cast<std::size_t>(path.dimension()))
  {
    result.status = "invalid_input";
    result.detail = "limit count does not match the path dimension";
    return result;
  }
  if (!(duration > 0.0) || !std::isfinite(duration))
  {
    result.status = "invalid_input";
    result.detail = "duration must be positive and finite";
    return result;
  }
  if (!(options.interpolation_dt > 0.0) || !std::isfinite(options.interpolation_dt) ||
      options.max_evaluations <= 0)
  {
    result.status = "invalid_input";
    result.detail = "invalid evaluation interval or evaluation budget";
    return result;
  }
  if (!(options.export_spacing > 0.0) || !std::isfinite(options.export_spacing) ||
      !(options.refined_export_spacing > 0.0) || !std::isfinite(options.refined_export_spacing) ||
      options.refined_export_spacing > options.export_spacing ||
      !(options.export_fidelity_tolerance >= 0.0) ||
      !std::isfinite(options.export_fidelity_tolerance))
  {
    result.status = "invalid_input";
    result.detail = "invalid export spacing or fidelity tolerance";
    return result;
  }

  const int n = BernsteinTimeMap(options.degree).parameterCount();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
  BernsteinTimeMap baseline_map(options.degree);
  baseline_map.setParameters(zero);
  std::string error;
  if (!evaluateContinuous(path, baseline_map, duration, limits, options.interpolation_dt,
                          result.baseline_continuous, error))
  {
    result.status = "invalid_input";
    result.detail = "the baseline time map could not be evaluated: " + error;
    return result;
  }
  result.baseline_objective = result.baseline_continuous.jerk_rms;
  const double baseline_peak = result.baseline_continuous.jerk_max;
  int evaluations = 1;

  BernsteinTimeMap search_map(options.degree);
  auto cost = [&](const Eigen::VectorXd& u, TrajectoryMetrics& metrics) -> double {
    if (evaluations >= options.max_evaluations) return std::numeric_limits<double>::infinity();
    ++evaluations;
    search_map.setParameters(u);
    std::string cost_error;
    if (!evaluateContinuous(path, search_map, duration, limits, options.interpolation_dt,
                            metrics, cost_error))
      return 1e12;
    const double excess_v = std::max(0.0, metrics.max_velocity_ratio - 1.0);
    const double excess_a = std::max(0.0, metrics.max_acceleration_ratio - 1.0);
    double peak_excess = 0.0;
    if (baseline_peak > 0.0)
      peak_excess = std::max(0.0, metrics.jerk_max / baseline_peak - 1.0);
    else if (metrics.jerk_max > 0.0)
      peak_excess = 1.0;
    double cost_value =
        metrics.jerk_rms +
        options.violation_weight *
            (excess_v * excess_v + excess_a * excess_a + peak_excess * peak_excess);
    if (metrics.position_violation) cost_value += options.violation_weight;
    return cost_value;
  };

  // Nelder-Mead over the log-speed parameters. The baseline vertex is kept, so
  // the result can never be worse than the linear clock. The baseline objective
  // is evaluated above; subsequent evaluations use the continuous composition,
  // never the exported interpolation metric.
  std::vector<Eigen::VectorXd> simplex(static_cast<std::size_t>(n) + 1);
  std::vector<double> f(static_cast<std::size_t>(n) + 1);
  simplex[0] = zero;
  f[0] = result.baseline_objective;
  for (int i = 0; i < n; ++i)
  {
    Eigen::VectorXd x = zero;
    x[i] = 0.3;
    simplex[static_cast<std::size_t>(i) + 1] = x;
    TrajectoryMetrics ignored;
    f[static_cast<std::size_t>(i) + 1] = cost(x, ignored);
  }

  while (evaluations < options.max_evaluations)
  {
    // order
    std::vector<int> order(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) order[static_cast<std::size_t>(i)] = i;
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return f[static_cast<std::size_t>(a)] < f[static_cast<std::size_t>(b)]; });
    std::vector<Eigen::VectorXd> sorted(static_cast<std::size_t>(n) + 1);
    std::vector<double> sorted_f(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i)
    {
      sorted[static_cast<std::size_t>(i)] = simplex[static_cast<std::size_t>(order[static_cast<std::size_t>(i)])];
      sorted_f[static_cast<std::size_t>(i)] = f[static_cast<std::size_t>(order[static_cast<std::size_t>(i)])];
    }
    simplex = sorted;
    f = sorted_f;

    Eigen::VectorXd centroid = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) centroid += simplex[static_cast<std::size_t>(i)];
    centroid /= static_cast<double>(n);

    const Eigen::VectorXd& worst = simplex[static_cast<std::size_t>(n)];
    Eigen::VectorXd reflected = centroid + (centroid - worst);
    TrajectoryMetrics m_ref;
    const double f_ref = cost(reflected, m_ref);
    if (f_ref < f[0])
    {
      Eigen::VectorXd expanded = centroid + 2.0 * (centroid - worst);
      TrajectoryMetrics m_exp;
      const double f_exp = cost(expanded, m_exp);
      if (f_exp < f_ref)
      {
        simplex[static_cast<std::size_t>(n)] = expanded;
        f[static_cast<std::size_t>(n)] = f_exp;
      }
      else
      {
        simplex[static_cast<std::size_t>(n)] = reflected;
        f[static_cast<std::size_t>(n)] = f_ref;
      }
      continue;
    }
    if (f_ref < f[static_cast<std::size_t>(n - 1)])
    {
      simplex[static_cast<std::size_t>(n)] = reflected;
      f[static_cast<std::size_t>(n)] = f_ref;
      continue;
    }
    Eigen::VectorXd contracted = centroid + 0.5 * (worst - centroid);
    TrajectoryMetrics m_con;
    const double f_con = cost(contracted, m_con);
    if (f_con < f[static_cast<std::size_t>(n)])
    {
      simplex[static_cast<std::size_t>(n)] = contracted;
      f[static_cast<std::size_t>(n)] = f_con;
      continue;
    }
    for (int i = 1; i <= n; ++i)
    {
      simplex[static_cast<std::size_t>(i)] =
          0.5 * (simplex[static_cast<std::size_t>(i)] + simplex[0]);
      TrajectoryMetrics m_shrink;
      f[static_cast<std::size_t>(i)] = cost(simplex[static_cast<std::size_t>(i)], m_shrink);
    }
    if (evaluations >= options.max_evaluations) break;
  }

  int best_index = 0;
  for (int i = 1; i <= n; ++i)
  {
    if (f[static_cast<std::size_t>(i)] < f[static_cast<std::size_t>(best_index)])
      best_index = i;
  }
  result.parameters = simplex[static_cast<std::size_t>(best_index)];
  result.objective = f[static_cast<std::size_t>(best_index)];
  result.selected_parameters = zero;

  BernsteinTimeMap candidate_map(options.degree);
  candidate_map.setParameters(result.parameters);
  if (!evaluateContinuous(path, candidate_map, duration, limits, options.interpolation_dt,
                          result.candidate_continuous, error))
  {
    result.status = "invalid_input";
    result.detail = "the candidate time map could not be evaluated: " + error;
    result.evaluations = evaluations;
    return result;
  }

  auto build_export = [&](const BernsteinTimeMap& export_map,
                          std::vector<TrajectoryPoint>& points,
                          TrajectoryMetrics& metrics,
                          double& deviation,
                          std::string& export_error) -> bool {
    if (!buildFaithfulExport(path, export_map, duration, options.export_spacing, points,
                             export_error))
      return false;
    if (!evaluateExported(points, limits, options.interpolation_dt, metrics, export_error))
      return false;
    return maxPositionDeviation(points, path, export_map, duration, options.interpolation_dt,
                                deviation, export_error);
  };

  double baseline_deviation = std::numeric_limits<double>::quiet_NaN();
  double candidate_deviation = std::numeric_limits<double>::quiet_NaN();
  std::vector<tpo::TrajectoryPoint> baseline_export;
  std::vector<tpo::TrajectoryPoint> candidate_export;
  TrajectoryMetrics baseline_export_metrics;
  TrajectoryMetrics candidate_export_metrics;
  bool baseline_export_ok =
      build_export(baseline_map, baseline_export, baseline_export_metrics, baseline_deviation, error);
  if (!baseline_export_ok)
  {
    result.status = "invalid_input";
    result.detail = "baseline faithful export failed: " + error;
    result.evaluations = evaluations;
    return result;
  }
  bool candidate_export_ok =
      build_export(candidate_map, candidate_export, candidate_export_metrics, candidate_deviation,
                   error);
  bool export_fidelity_ok =
      candidate_export_ok && baseline_deviation <= options.export_fidelity_tolerance &&
      candidate_deviation <= options.export_fidelity_tolerance;

  // The first export is deliberately coarse. If either side fails the
  // representation target, refine both once with the same grid and same
  // evaluation method.
  if (!export_fidelity_ok)
  {
    std::vector<tpo::TrajectoryPoint> refined_baseline;
    std::vector<tpo::TrajectoryPoint> refined_candidate;
    TrajectoryMetrics refined_baseline_metrics;
    TrajectoryMetrics refined_candidate_metrics;
    double refined_baseline_deviation = std::numeric_limits<double>::quiet_NaN();
    double refined_candidate_deviation = std::numeric_limits<double>::quiet_NaN();
    const bool refined_baseline_ok =
        buildFaithfulExport(path, baseline_map, duration, options.refined_export_spacing,
                            refined_baseline, error) &&
        evaluateExported(refined_baseline, limits, options.interpolation_dt,
                         refined_baseline_metrics, error) &&
        maxPositionDeviation(refined_baseline, path, baseline_map, duration,
                             options.interpolation_dt, refined_baseline_deviation, error);
    const bool refined_candidate_ok =
        buildFaithfulExport(path, candidate_map, duration, options.refined_export_spacing,
                            refined_candidate, error) &&
        evaluateExported(refined_candidate, limits, options.interpolation_dt,
                         refined_candidate_metrics, error) &&
        maxPositionDeviation(refined_candidate, path, candidate_map, duration,
                             options.interpolation_dt, refined_candidate_deviation, error);
    if (refined_baseline_ok)
    {
      baseline_export = std::move(refined_baseline);
      baseline_export_metrics = refined_baseline_metrics;
      baseline_deviation = refined_baseline_deviation;
    }
    if (refined_candidate_ok)
    {
      candidate_export = std::move(refined_candidate);
      candidate_export_metrics = refined_candidate_metrics;
      candidate_deviation = refined_candidate_deviation;
      candidate_export_ok = true;
    }
    export_fidelity_ok =
        refined_baseline_ok && refined_candidate_ok &&
        refined_baseline_deviation <= options.export_fidelity_tolerance &&
        refined_candidate_deviation <= options.export_fidelity_tolerance;
    result.export_spacing_used = options.refined_export_spacing;
  }
  else
  {
    result.export_spacing_used = options.export_spacing;
  }

  result.baseline_export = std::move(baseline_export);
  result.candidate_export = std::move(candidate_export);
  result.baseline_metrics = baseline_export_metrics;
  result.candidate_metrics = candidate_export_metrics;
  result.export_fidelity_ok = export_fidelity_ok;
  result.baseline_export_deviation_rad = baseline_deviation;
  result.max_export_deviation_rad = candidate_deviation;
  const auto acceptance = evaluateTimeProfileAcceptance(
      result.baseline_continuous, result.candidate_continuous,
      candidate_export_ok && result.export_fidelity_ok);
  result.constraints_ok = acceptance.constraints_ok;
  result.rms_improved = acceptance.rms_improved;
  result.peak_ok = acceptance.peak_ok;
  result.accepted = acceptance.accepted;
  result.fallback_reason = acceptance.fallback_reason;
  result.fallback_to_baseline = !result.accepted;
  if (result.accepted)
  {
    result.selected_parameters = result.parameters;
    result.selected_export = result.candidate_export;
    result.selected_metrics = result.candidate_metrics;
    result.decision = "accepted";
    result.status = "accepted";
    result.detail = "continuous RMS reduced and all final checks passed";
  }
  else
  {
    result.selected_parameters = zero;
    result.selected_export = result.baseline_export;
    result.selected_metrics = result.baseline_metrics;
    result.decision = "fallback_to_baseline";
    result.status = "fallback";
    result.detail = result.fallback_reason;
  }

  result.evaluations = evaluations;
  result.ok = true;
  return result;
}

}  // namespace tpo
