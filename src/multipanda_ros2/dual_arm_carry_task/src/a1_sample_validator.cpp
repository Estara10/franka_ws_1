// A1 P2a: pure, offline-testable sample validity checks. See the header.

#include "dual_arm_carry_task/a1_sample_validator.hpp"

#include <algorithm>
#include <thread>
#include <functional>
#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>

namespace a1v {
namespace {

bool finite(double v) { return std::isfinite(v); }

bool finite(const Vec3& v) { return finite(v.x) && finite(v.y) && finite(v.z); }

bool finite(const Quat& q) { return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w); }

double clampUnit(double v) { return std::min(1.0, std::max(-1.0, v)); }

ItemResult makeItem(CheckStatus s, std::string code, std::string detail)
{
  ItemResult r;
  r.status = s;
  r.reason_code = std::move(code);
  r.detail = std::move(detail);
  return r;
}

std::string fmt(double v, int precision = 6)
{
  std::ostringstream os;
  os.precision(precision);
  os << std::fixed << v;
  return os.str();
}

// True when the payload is usable for geometric reasoning at all. A light local
// check so this function is safe when called standalone; the authoritative
// message validation (run_id, frame, bounds) remains a1t::validateTruth.
bool payloadUsable(const a1t::TruthPayload& t, std::string& why)
{
  if (!finite(t.position)) {
    why = "position has a non-finite component";
    return false;
  }
  if (!finite(t.orientation)) {
    why = "orientation has a non-finite component";
    return false;
  }
  const double n2 = t.orientation.x * t.orientation.x + t.orientation.y * t.orientation.y +
                    t.orientation.z * t.orientation.z + t.orientation.w * t.orientation.w;
  if (!(n2 > 0.0)) {
    why = "orientation has zero norm";
    return false;
  }
  if (std::fabs(std::sqrt(n2) - 1.0) > 1e-6) {
    why = "orientation is not a unit quaternion";
    return false;
  }
  if (!finite(t.size)) {
    why = "size has a non-finite component";
    return false;
  }
  if (!(t.size.x > 0.0) || !(t.size.y > 0.0) || !(t.size.z > 0.0)) {
    why = "size has a non-positive component";
    return false;
  }
  return true;
}

}  // namespace

const char* statusName(CheckStatus s)
{
  switch (s) {
    case CheckStatus::PASSED: return "passed";
    case CheckStatus::FAILED: return "failed";
    case CheckStatus::NOT_RUN: return "NOT_RUN";
  }
  return "NOT_RUN";
}

// ---------------------------------------------------------------------------
// Shared target construction
// ---------------------------------------------------------------------------

Quat quatFromRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  Quat q;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  return q;
}

double quatYaw(const Quat& q)
{
  if (!finite(q)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!(n2 > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  if (std::fabs(siny) < 1e-12 && std::fabs(cosy) < 1e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(siny, cosy);
}

Vec3 rotateZ(const Vec3& v, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Vec3{v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

DualTarget buildDualTarget(const a1t::TruthPayload& truth,
                           const GraspGeometry& geom,
                           double height_m,
                           double* yaw_out)
{
  const double yaw = quatYaw(truth.orientation);
  if (yaw_out != nullptr) {
    *yaw_out = yaw;
  }

  DualTarget out;
  if (!std::isfinite(yaw)) {
    return out;  // caller must treat a non-finite result as unusable
  }

  // Offsets are expressed along the rod's local Y (its long axis) and then
  // rotated into the world by the rod yaw. Left is the +Y end, matching A0.
  const Vec3 left_off = rotateZ(Vec3{0.0, geom.grasp_offset_m, 0.0}, yaw);
  const Vec3 right_off = rotateZ(Vec3{0.0, -geom.grasp_offset_m, 0.0}, yaw);

  out.left.position = Vec3{truth.position.x + left_off.x, truth.position.y + left_off.y, height_m};
  out.right.position = Vec3{truth.position.x + right_off.x, truth.position.y + right_off.y, height_m};

  // Rz(yaw + base_yaw) * Rx(pi): the same construction A0 uses, with the rod
  // yaw added so the wrist turns with the rod.
  const Quat q = quatFromRpy(kPi, 0.0, yaw + geom.eef_base_yaw_rad);
  out.left.orientation = q;
  out.right.orientation = q;
  return out;
}

DualTarget buildPreGraspTarget(const a1t::TruthPayload& truth,
                               const GraspGeometry& geom,
                               double* yaw_out)
{
  return buildDualTarget(truth, geom, geom.pre_grasp_height_m, yaw_out);
}

DualTarget buildGraspTarget(const a1t::TruthPayload& truth,
                            const GraspGeometry& geom,
                            double* yaw_out)
{
  return buildDualTarget(truth, geom, geom.grasp_height_m, yaw_out);
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

Footprint computeFootprint(const Vec3& center, const Vec3& size, double yaw_rad)
{
  Footprint fp;
  const double hx = size.x * 0.5;
  const double hy = size.y * 0.5;

  const Vec3 corners[4] = {
      Vec3{-hx, -hy, 0.0},
      Vec3{-hx, hy, 0.0},
      Vec3{hx, -hy, 0.0},
      Vec3{hx, hy, 0.0},
  };

  bool first = true;
  for (const Vec3& c : corners) {
    const Vec3 r = rotateZ(c, yaw_rad);
    const double x = center.x + r.x;
    const double y = center.y + r.y;
    if (first) {
      fp.x_min = fp.x_max = x;
      fp.y_min = fp.y_max = y;
      first = false;
    } else {
      fp.x_min = std::min(fp.x_min, x);
      fp.x_max = std::max(fp.x_max, x);
      fp.y_min = std::min(fp.y_min, y);
      fp.y_max = std::max(fp.y_max, y);
    }
  }
  return fp;
}

GeometryResult checkGeometry(const a1t::TruthPayload& truth,
                             const WorkspaceBounds& bounds,
                             const SupportCriteria& support)
{
  GeometryResult g;
  g.yaw_rad = quatYaw(truth.orientation);

  std::string why;
  if (!payloadUsable(truth, why)) {
    g.item = makeItem(CheckStatus::FAILED, "E_GEO_BAD_INPUT", why);
    return g;
  }
  if (!std::isfinite(g.yaw_rad)) {
    g.item = makeItem(CheckStatus::FAILED, "E_GEO_BAD_INPUT", "rod yaw is not defined");
    return g;
  }
  if (!(bounds.x_max > bounds.x_min) || !(bounds.y_max > bounds.y_min)) {
    g.item = makeItem(CheckStatus::FAILED, "E_GEO_BAD_CONFIG", "workspace bounds are degenerate");
    return g;
  }

  g.footprint = computeFootprint(truth.position, truth.size, g.yaw_rad);
  g.center_z = truth.position.z;

  // Tilt of the rod's local Z (its short axis, vertical when lying flat)
  // against world +Z.
  const Quat& q = truth.orientation;
  const double zax_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  g.tilt_deg = std::acos(clampUnit(zax_z)) * 180.0 / kPi;

  g.footprint.clearance_x_min = g.footprint.x_min - bounds.x_min;
  g.footprint.clearance_x_max = bounds.x_max - g.footprint.x_max;
  g.footprint.clearance_y_min = g.footprint.y_min - bounds.y_min;
  g.footprint.clearance_y_max = bounds.y_max - g.footprint.y_max;
  g.footprint.min_clearance_m = std::min({g.footprint.clearance_x_min,
                                          g.footprint.clearance_x_max,
                                          g.footprint.clearance_y_min,
                                          g.footprint.clearance_y_max});

  // Support height: the rod centre must sit half its height above the surface.
  const double expected_z = support.support_z + truth.size.z * 0.5;
  if (std::fabs(g.center_z - expected_z) > support.resting_z_tol) {
    g.item = makeItem(CheckStatus::FAILED, "E_GEO_SUPPORT_HEIGHT",
                      "rod centre z " + fmt(g.center_z) + " differs from support_z + half height " +
                          fmt(expected_z) + " by more than " + fmt(support.resting_z_tol));
    return g;
  }

  if (g.tilt_deg > support.max_tilt_deg) {
    g.item = makeItem(CheckStatus::FAILED, "E_GEO_TILT",
                      "rod tilt " + fmt(g.tilt_deg, 4) + " deg exceeds " +
                          fmt(support.max_tilt_deg, 4) + " deg");
    return g;
  }

  // Containment: both the footprint and the workspace are convex, so requiring
  // every footprint corner to be inside the rectangle implies containment.
  if (g.footprint.min_clearance_m < 0.0) {
    g.item = makeItem(CheckStatus::FAILED, "E_GEO_OUT_OF_WORKSPACE",
                      "rod footprint leaves the configured workspace (min clearance " +
                          fmt(g.footprint.min_clearance_m * 1000.0, 3) + " mm); footprint x=[" +
                          fmt(g.footprint.x_min, 4) + ", " + fmt(g.footprint.x_max, 4) + "] y=[" +
                          fmt(g.footprint.y_min, 4) + ", " + fmt(g.footprint.y_max, 4) + "]");
    return g;
  }

  g.item = makeItem(CheckStatus::PASSED, "E_OK",
                    "footprint inside the configured workspace by at least " +
                        fmt(g.footprint.min_clearance_m * 1000.0, 3) + " mm");
  return g;
}

// ---------------------------------------------------------------------------
// Joint-space helpers
// ---------------------------------------------------------------------------

JointLimitCheck checkJointLimits(const std::vector<double>& positions,
                                 const std::vector<double>& lower,
                                 const std::vector<double>& upper,
                                 const std::vector<std::string>& names)
{
  JointLimitCheck r;
  const std::size_t n = positions.size();
  if (n == 0 || lower.size() != n || upper.size() != n) {
    r.offending_name = "<size mismatch>";
    return r;
  }

  r.worst_margin_rad = std::numeric_limits<double>::infinity();
  r.ok = true;
  for (std::size_t i = 0; i < n; ++i) {
    const double v = positions[i];
    const double lo = lower[i];
    const double hi = upper[i];
    if (!std::isfinite(v)) {
      r.ok = false;
      r.offending_index = i;
      r.offending_name = i < names.size() ? names[i] : "<unnamed>";
      r.value = v;
      r.lower = lo;
      r.upper = hi;
      return r;
    }
    // Unbounded joints contribute no margin; they are still checked for
    // finiteness above.
    if (std::isfinite(lo) && std::isfinite(hi)) {
      r.worst_margin_rad = std::min(r.worst_margin_rad, std::min(v - lo, hi - v));
    }
    if (v < lo || v > hi) {
      r.ok = false;
      r.offending_index = i;
      r.offending_name = i < names.size() ? names[i] : "<unnamed>";
      r.value = v;
      r.lower = lo;
      r.upper = hi;
      return r;
    }
  }
  if (n == 0 || !std::isfinite(r.worst_margin_rad)) {
    r.worst_margin_rad = 0.0;
  }
  return r;
}

double maxAbsJointDelta(const std::vector<double>& a, const std::vector<double>& b)
{
  const std::size_t n = std::min(a.size(), b.size());
  double worst = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    worst = std::max(worst, std::fabs(a[i] - b[i]));
  }
  return worst;
}

PoseError poseError(const ArmTarget& target, const ArmTarget& actual)
{
  PoseError e;
  const double dx = actual.position.x - target.position.x;
  const double dy = actual.position.y - target.position.y;
  const double dz = actual.position.z - target.position.z;
  e.pos_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  e.angle_rad = a1t::rotationDistanceRad(target.orientation, actual.orientation);
  return e;
}

// ---------------------------------------------------------------------------
// Collision-matrix classification
// ---------------------------------------------------------------------------

bool isCrossArmPair(const std::string& a, const std::string& b)
{
  auto arm_of = [](const std::string& s) -> int {
    if (s.rfind("mj_left", 0) == 0) {
      return 1;
    }
    if (s.rfind("mj_right", 0) == 0) {
      return 2;
    }
    return 0;
  };
  const int ia = arm_of(a);
  const int ib = arm_of(b);
  return ia != 0 && ib != 0 && ia != ib;
}

bool involvesObject(const std::string& a, const std::string& b, const std::string& object_id)
{
  return (a == object_id) != (b == object_id);
}

bool isRodPair(const std::string& a, const std::string& b, const std::string& rod_id)
{
  return involvesObject(a, b, rod_id);
}

bool isSupportPair(const std::string& a, const std::string& b, const std::string& support_id)
{
  return involvesObject(a, b, support_id);
}

bool isMountingLink(const std::string& name)
{
  // The base assembly. Each of these has collision geometry that straddles the
  // mounting plane in this model, and their z-extent does not depend on the arm
  // configuration:
  //   base_link      -- box at the origin, z in [-0.05, +0.05]
  //   mj_*_link0     -- spheres at local z=0.06, r=0.06(+0.03), z in [-0.03, 0.15]
  //   mj_*_link1     -- joint1 sits at z=0.333 and link1's lowest sphere is at
  //                     local z=-0.333, so its centre lands exactly on z=0 with
  //                     r=0.06(+0.03) -> z in [-0.09, +0.09]; joint1 rotates
  //                     about the vertical axis only, so this does not change.
  // Everything else, the hand and fingers included, stays checked against the
  // ground.
  return name == "base_link" || name == "mj_left_link0" || name == "mj_right_link0" ||
         name == "mj_left_link1" || name == "mj_right_link1";
}

bool isFingerLink(const std::string& name)
{
  return name.find("finger") != std::string::npos;
}

// ---------------------------------------------------------------------------
// Bounded dispatch
// ---------------------------------------------------------------------------

BoundedExecOutcome runBoundedExec(const std::function<int()>& work,
                                  const std::function<void()>& on_timeout,
                                  double timeout_sec,
                                  double grace_sec,
                                  double poll_interval_sec)
{
  BoundedExecOutcome outcome;
  constexpr int kPending = -999;
  constexpr int kException = -998;
  constexpr long long kNotFinished = -1;

  // Shared, never a stack reference: a detached worker would otherwise write
  // into a dead frame.
  auto slot = std::make_shared<std::atomic<int>>(kPending);
  auto threw = std::make_shared<std::atomic<bool>>(false);
  // When the worker actually finished, as nanoseconds since `begin`; -1 while it
  // is still running. Published last, so a non-negative value also implies the
  // result slot is already written.
  auto finished_ns = std::make_shared<std::atomic<long long>>(kNotFinished);
  std::function<int()> captured = work;

  // The budget starts here, BEFORE the worker exists, so the order is explicit:
  // begin -> deadline -> worker start. The worker receives `begin` by value and
  // stamps its own completion against it.
  const auto begin = std::chrono::steady_clock::now();
  const auto poll =
      std::chrono::duration<double>(poll_interval_sec > 0.0 ? poll_interval_sec : 0.001);
  const auto budget =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(timeout_sec));
  const auto deadline = begin + budget;
  outcome.timeout_sec = timeout_sec;
  const long long budget_ns =
      static_cast<long long>(std::chrono::duration<double>(budget).count() * 1e9);

  std::thread worker([slot, threw, finished_ns, captured, begin]() mutable {
    try {
      slot->store(captured());
    } catch (...) {
      threw->store(true);
      slot->store(kException);
    }
    const long long stamp = static_cast<long long>(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() * 1e9);
    finished_ns->store(stamp);  // published last: value is already visible
  });

  while (finished_ns->load() == kNotFinished && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(poll);
  }

  const long long finished_stamp = finished_ns->load();
  if (finished_stamp != kNotFinished) {
    worker.join();
    outcome.elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    outcome.value = slot->load();
    outcome.worker_finished = true;
    outcome.completed_at_sec = static_cast<double>(finished_stamp) / 1e9;
    if (finished_stamp > budget_ns) {
      // Finished, but too late. The value is recorded for diagnostics only and
      // the status stays TIMEOUT: a completed slot is not an in-time result.
      outcome.status = BoundedExecOutcome::Status::TIMEOUT;
      return outcome;
    }
    outcome.status = threw->load() ? BoundedExecOutcome::Status::EXCEPTION
                                   : BoundedExecOutcome::Status::COMPLETED;
    return outcome;
  }

  // Reaching here means the worker was STILL RUNNING when the budget expired
  // (a worker that had already finished is handled above, by its own timestamp).
  // Only a still-running worker needs a stop request; terminate this attempt as
  // a timeout regardless of what the worker does later.
  outcome.status = BoundedExecOutcome::Status::TIMEOUT;
  if (on_timeout) {
    on_timeout();
    outcome.stop_requested = true;
  }
  const auto grace_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(grace_sec));
  while (finished_ns->load() == kNotFinished && std::chrono::steady_clock::now() < grace_deadline) {
    std::this_thread::sleep_for(poll);
  }

  const long long late_stamp = finished_ns->load();
  outcome.elapsed_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  if (late_stamp != kNotFinished) {
    outcome.worker_finished = true;
    worker.join();
    // The late value and instant are recorded for diagnostics only; the status
    // stays TIMEOUT, so a late success can never be read as an in-time one.
    outcome.value = slot->load();
    outcome.completed_at_sec = static_cast<double>(late_stamp) / 1e9;
  } else {
    // Safe to detach: the worker holds only shared state.
    worker.detach();
  }
  return outcome;
}

// ---------------------------------------------------------------------------
// Trajectory structural validation
// ---------------------------------------------------------------------------

JointNameCheck validateJointNames(const std::vector<std::string>& names,
                                  const std::vector<std::string>& expected)
{
  JointNameCheck r;
  if (expected.empty()) {
    r.reason_code = "E_TRAJ_NO_EXPECTED_JOINTS";
    r.detail = "the expected joint set is empty, so nothing can be verified";
    return r;
  }

  std::set<std::string> expected_set(expected.begin(), expected.end());
  std::set<std::string> seen;
  for (const std::string& n : names) {
    if (expected_set.count(n) == 0) {
      ++r.unknown;
      if (r.first_unknown.empty()) {
        r.first_unknown = n;
      }
    }
    if (!seen.insert(n).second) {
      ++r.duplicates;
      if (r.first_duplicate.empty()) {
        r.first_duplicate = n;
      }
    }
  }
  for (const std::string& e : expected_set) {
    if (seen.count(e) == 0) {
      ++r.missing;
    }
  }

  if (r.unknown > 0) {
    r.reason_code = "E_TRAJ_UNKNOWN_JOINT";
    r.detail = "the plan names " + std::to_string(r.unknown) +
               " joint(s) outside the expected set, e.g. '" + r.first_unknown + "'";
    return r;
  }
  if (r.duplicates > 0) {
    r.reason_code = "E_TRAJ_DUPLICATE_JOINT";
    r.detail = "the plan names '" + r.first_duplicate + "' more than once";
    return r;
  }
  if (r.missing > 0) {
    r.reason_code = "E_TRAJ_MISSING_JOINT";
    r.detail = std::to_string(r.missing) +
               " expected joint(s) are absent from the plan, so part of the robot would "
               "not be commanded";
    return r;
  }
  r.ok = true;
  return r;
}

TimeCheck validateTimeFromStart(const std::vector<double>& times_sec)
{
  TimeCheck r;
  if (times_sec.empty()) {
    r.reason_code = "E_TIME_EMPTY";
    r.detail = "the trajectory has no points";
    return r;
  }
  r.first_sec = times_sec.front();
  r.last_sec = times_sec.back();

  for (std::size_t i = 0; i < times_sec.size(); ++i) {
    if (!std::isfinite(times_sec[i])) {
      r.bad_index = i;
      r.reason_code = "E_TIME_NOT_FINITE";
      r.detail = "time_from_start at point " + std::to_string(i) + " is not finite";
      return r;
    }
  }
  if (times_sec.front() < 0.0) {
    r.reason_code = "E_TIME_NEGATIVE";
    r.detail = "time_from_start starts negative (" + std::to_string(times_sec.front()) + ")";
    return r;
  }
  if (times_sec.front() > 1e-9) {
    r.reason_code = "E_TIME_NOT_STARTING_AT_ZERO";
    r.detail = "the first point is at t=" + std::to_string(times_sec.front()) +
               " s instead of 0";
    return r;
  }
  for (std::size_t i = 1; i < times_sec.size(); ++i) {
    if (!(times_sec[i] > times_sec[i - 1])) {
      r.bad_index = i;
      r.reason_code = "E_TIME_NOT_INCREASING";
      r.detail = "time_from_start is not strictly increasing at point " + std::to_string(i) +
                 " (" + std::to_string(times_sec[i - 1]) + " -> " + std::to_string(times_sec[i]) +
                 ")";
      return r;
    }
  }
  r.ok = true;
  return r;
}

DerivativeCheck checkDerivativeField(const std::vector<double>& values,
                                     const std::vector<double>& limits,
                                     const std::vector<bool>& bounded,
                                     const std::vector<std::string>& names)
{
  DerivativeCheck r;
  if (values.size() != limits.size() || limits.size() != bounded.size()) {
    r.reason_code = "E_DERIV_DIM_MISMATCH";
    r.detail = "field has " + std::to_string(values.size()) + " components but " +
               std::to_string(limits.size()) + " limits are available";
    return r;
  }
  if (values.empty()) {
    r.reason_code = "E_DERIV_DIM_MISMATCH";
    r.detail = "the derivative field is present but empty";
    return r;
  }

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) {
      r.reason_code = "E_DERIV_NOT_FINITE";
      r.detail = "component " + std::to_string(i) + " (" +
                 (i < names.size() ? names[i] : std::string("<unnamed>")) + ") is not finite";
      return r;
    }
    if (!bounded[i]) {
      ++r.unbounded;
      continue;
    }
    ++r.checked;
    const double lim = std::fabs(limits[i]);
    if (!(lim > 0.0)) {
      // A declared limit of zero would be unusable; treat it as undeclared.
      ++r.unbounded;
      --r.checked;
      continue;
    }
    const double ratio = std::fabs(values[i]) / lim;
    if (ratio > r.worst_ratio) {
      r.worst_ratio = ratio;
      r.worst_joint = i < names.size() ? names[i] : std::string("<unnamed>");
    }
    if (ratio > 1.0) {
      r.reason_code = "E_DERIV_LIMIT_EXCEEDED";
      r.detail = "component " + std::to_string(i) + " (" + r.worst_joint + ") = " +
                 std::to_string(values[i]) + " exceeds the declared limit " +
                 std::to_string(lim);
      return r;
    }
  }
  r.ok = true;
  return r;
}

// ---------------------------------------------------------------------------
// Path segment sampling
// ---------------------------------------------------------------------------

std::size_t subdivisionsFor(double max_joint_delta_rad, double max_step_rad)
{
  if (!(max_step_rad > 0.0) || !std::isfinite(max_joint_delta_rad) ||
      !(max_joint_delta_rad > 0.0)) {
    return 1;
  }
  const double n = std::ceil(max_joint_delta_rad / max_step_rad);
  if (!std::isfinite(n) || n < 1.0) {
    return 1;
  }
  return static_cast<std::size_t>(n);
}

std::vector<double> interpolateJoints(const std::vector<double>& a,
                                      const std::vector<double>& b,
                                      double t)
{
  if (a.size() != b.size()) {
    return {};
  }
  const double u = std::min(1.0, std::max(0.0, t));
  std::vector<double> out(a.size(), 0.0);
  for (std::size_t i = 0; i < a.size(); ++i) {
    out[i] = a[i] + (b[i] - a[i]) * u;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Local hand-model substitution
// ---------------------------------------------------------------------------

Vec3 Aabb::center() const
{
  return Vec3{0.5 * (min.x + max.x), 0.5 * (min.y + max.y), 0.5 * (min.z + max.z)};
}

Vec3 Aabb::size() const { return Vec3{max.x - min.x, max.y - min.y, max.z - min.z}; }

Aabb aabbOfPoints(const std::vector<Vec3>& points)
{
  Aabb b;
  if (points.empty()) {
    return b;
  }
  b.min = points[0];
  b.max = points[0];
  for (const Vec3& p : points) {
    b.min.x = std::min(b.min.x, p.x);
    b.min.y = std::min(b.min.y, p.y);
    b.min.z = std::min(b.min.z, p.z);
    b.max.x = std::max(b.max.x, p.x);
    b.max.y = std::max(b.max.y, p.y);
    b.max.z = std::max(b.max.z, p.z);
  }
  b.valid = true;
  return b;
}

bool aabbContainsAll(const Aabb& box, const std::vector<Vec3>& points, double eps)
{
  if (!box.valid) {
    return false;
  }
  for (const Vec3& p : points) {
    if (p.x < box.min.x - eps || p.x > box.max.x + eps || p.y < box.min.y - eps ||
        p.y > box.max.y + eps || p.z < box.min.z - eps || p.z > box.max.z + eps) {
      return false;
    }
  }
  return true;
}

bool readBinaryStlVertices(const std::string& path, std::vector<Vec3>& out, std::string& error)
{
  out.clear();
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "cannot open '" + path + "'";
    return false;
  }
  char header[80];
  if (!in.read(header, 80)) {
    error = "'" + path + "' is shorter than an STL header";
    return false;
  }
  std::uint32_t triangles = 0;
  if (!in.read(reinterpret_cast<char*>(&triangles), 4)) {
    error = "'" + path + "' has no triangle count";
    return false;
  }
  // Guard against an ASCII file being read as binary: a plausible triangle
  // count is bounded by the file size.
  const std::streamoff total = [&path]() -> std::streamoff {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    return f ? static_cast<std::streamoff>(f.tellg()) : 0;
  }();
  if (total != static_cast<std::streamoff>(84) + static_cast<std::streamoff>(triangles) * 50) {
    error = "'" + path + "' is not a binary STL of the declared size (expected " +
            std::to_string(84 + static_cast<long long>(triangles) * 50) + " bytes, got " +
            std::to_string(static_cast<long long>(total)) + ")";
    return false;
  }
  out.reserve(static_cast<std::size_t>(triangles) * 3);
  char record[50];
  for (std::uint32_t i = 0; i < triangles; ++i) {
    if (!in.read(record, 50)) {
      error = "'" + path + "' ends early at triangle " + std::to_string(i);
      out.clear();
      return false;
    }
    for (int k = 0; k < 3; ++k) {
      float v[3];
      std::memcpy(v, record + 12 + k * 12, sizeof(v));
      out.push_back(Vec3{v[0], v[1], v[2]});
    }
  }
  if (out.empty()) {
    error = "'" + path + "' contains no vertices";
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Aggregation
// ---------------------------------------------------------------------------

bool SampleValidation::allStaticPassed() const
{
  return geometry.status == CheckStatus::PASSED && ik.status == CheckStatus::PASSED &&
         collision.status == CheckStatus::PASSED;
}

std::string SampleValidation::summary() const
{
  std::ostringstream os;
  os << "geometry=" << statusName(geometry.status) << " ik=" << statusName(ik.status)
     << " collision=" << statusName(collision.status)
     << " path_validation=" << statusName(path_validation.status);
  return os.str();
}

}  // namespace a1v
