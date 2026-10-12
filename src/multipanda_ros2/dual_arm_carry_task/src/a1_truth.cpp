// A1 P1b: pure, offline-testable truth handling. See a1_truth.hpp.

#include "dual_arm_carry_task/a1_truth.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace a1t {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(double v) { return std::isfinite(v); }
bool finite(const Vec3& v) { return finite(v.x) && finite(v.y) && finite(v.z); }
bool finite(const Quat& q) { return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w); }

double norm2(const Quat& q)
{
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

}  // namespace

Verdict makeVerdict(bool ok, std::string code, std::string detail)
{
  Verdict v;
  v.ok = ok;
  v.reason_code = std::move(code);
  v.detail = std::move(detail);
  return v;
}

Verdict validateTruth(const TruthPayload& payload,
                      const std::string& expected_run_id,
                      double min_size_m,
                      double max_size_m)
{
  if (expected_run_id.empty()) {
    return makeVerdict(false, "E_TRUTH_EXPECTED_RUN_ID_EMPTY",
                       "this run has no expected run_id configured");
  }
  if (payload.run_id != expected_run_id) {
    return makeVerdict(false, "E_TRUTH_RUN_ID_MISMATCH",
                       "payload run_id '" + payload.run_id + "' does not match expected '" +
                           expected_run_id + "'");
  }
  if (payload.frame_id.empty()) {
    return makeVerdict(false, "E_TRUTH_FRAME_EMPTY", "payload carries no frame_id");
  }
  if (!finite(payload.position)) {
    return makeVerdict(false, "E_TRUTH_NOT_FINITE", "position has a non-finite component");
  }
  if (!finite(payload.orientation)) {
    return makeVerdict(false, "E_TRUTH_BAD_ORIENTATION",
                       "orientation has a non-finite component");
  }
  if (!(norm2(payload.orientation) > 0.0)) {
    return makeVerdict(false, "E_TRUTH_BAD_ORIENTATION", "orientation has zero norm");
  }
  const double n = std::sqrt(norm2(payload.orientation));
  if (std::fabs(n - 1.0) > 1e-6) {
    return makeVerdict(false, "E_TRUTH_BAD_ORIENTATION",
                       "orientation is not a unit quaternion (norm " + std::to_string(n) + ")");
  }
  if (!finite(payload.size)) {
    return makeVerdict(false, "E_TRUTH_BAD_SIZE", "size has a non-finite component");
  }
  // A full BOX size must be strictly positive on every axis. Negative sizes
  // must be rejected, not silently folded by an abs() into a valid geometry.
  if (!(payload.size.x > 0.0) || !(payload.size.y > 0.0) || !(payload.size.z > 0.0)) {
    return makeVerdict(false, "E_TRUTH_BAD_SIZE", "size has a non-positive component");
  }
  const double max_component = std::max({payload.size.x, payload.size.y, payload.size.z});
  const double min_component = std::min({payload.size.x, payload.size.y, payload.size.z});
  if (min_component < min_size_m || max_component > max_size_m) {
    return makeVerdict(false, "E_TRUTH_BAD_SIZE",
                       "rod size outside the accepted range [" + std::to_string(min_size_m) + ", " +
                           std::to_string(max_size_m) + "]");
  }
  return makeVerdict(true, "", "");
}

FrameMapping resolveFrameMapping(const std::string& truth_frame,
                                const std::string& planning_frame)
{
  if (truth_frame.empty() || planning_frame.empty()) {
    return FrameMapping::UNSUPPORTED;
  }
  if (truth_frame == planning_frame) {
    return FrameMapping::SAME_FRAME;
  }
  // Documented identity between the simulation world and the planning base.
  const bool world_and_base = (truth_frame == "world" && planning_frame == "base_link") ||
                             (truth_frame == "base_link" && planning_frame == "world");
  if (world_and_base) {
    return FrameMapping::WORLD_TO_BASE_LINK;
  }
  return FrameMapping::UNSUPPORTED;
}

double rotationDistanceRad(const Quat& a, const Quat& b)
{
  if (!finite(a) || !finite(b)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  dot = std::fabs(dot);  // q and -q are the same rotation
  dot = std::min(1.0, std::max(0.0, dot));
  if (dot >= 1.0) {
    return 0.0;
  }
  const double s = std::sqrt(std::max(0.0, 1.0 - dot * dot));
  return 2.0 * std::atan2(s, dot);
}

SceneCheck verifySceneReadback(const TruthPayload& expected,
                               const Vec3& actual_position,
                               const Quat& actual_orientation,
                               const Vec3& actual_size,
                               double pos_tol_m,
                               double angle_tol_rad,
                               double size_tol_m)
{
  SceneCheck r;

  if (!finite(actual_position) || !finite(actual_orientation) || !finite(actual_size)) {
    r.reason_code = "E_SCENE_READBACK_NOT_FINITE";
    r.detail = "planning scene read-back contains a non-finite value";
    return r;
  }

  const double dx = actual_position.x - expected.position.x;
  const double dy = actual_position.y - expected.position.y;
  const double dz = actual_position.z - expected.position.z;
  r.pos_err_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  r.angle_err_rad = rotationDistanceRad(expected.orientation, actual_orientation);
  if (!finite(r.angle_err_rad)) {
    r.reason_code = "E_SCENE_READBACK_NOT_FINITE";
    r.detail = "orientation comparison produced a non-finite angle";
    return r;
  }
  r.size_err_m = std::max({std::fabs(actual_size.x - expected.size.x),
                           std::fabs(actual_size.y - expected.size.y),
                           std::fabs(actual_size.z - expected.size.z)});

  if (r.pos_err_m > pos_tol_m) {
    r.reason_code = "E_SCENE_POS_MISMATCH";
    r.detail = "position error " + std::to_string(r.pos_err_m * 1000.0) +
               " mm exceeds tolerance " + std::to_string(pos_tol_m * 1000.0) + " mm";
    return r;
  }
  if (r.angle_err_rad > angle_tol_rad) {
    r.reason_code = "E_SCENE_ANGLE_MISMATCH";
    r.detail = "orientation error " + std::to_string(r.angle_err_rad * 180.0 / kPi) +
               " deg exceeds tolerance " + std::to_string(angle_tol_rad * 180.0 / kPi) + " deg";
    return r;
  }
  if (r.size_err_m > size_tol_m) {
    r.reason_code = "E_SCENE_SIZE_MISMATCH";
    r.detail = "size error " + std::to_string(r.size_err_m) + " m exceeds tolerance " +
               std::to_string(size_tol_m) + " m";
    return r;
  }

  r.ok = true;
  return r;
}

Verdict TruthLatch::offer(const TruthPayload& payload,
                          const std::string& expected_run_id,
                          double min_size_m,
                          double max_size_m)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (frozen_) {
    ++rejected_count_;
    last_rejection_code_ = "E_TRUTH_ALREADY_FROZEN";
    return makeVerdict(false, last_rejection_code_,
                       "this run's reference is already frozen and is not updated");
  }

  const Verdict verdict = validateTruth(payload, expected_run_id, min_size_m, max_size_m);
  if (!verdict.ok) {
    ++rejected_count_;
    last_rejection_code_ = verdict.reason_code;
    return verdict;
  }

  value_ = payload;
  frozen_ = true;
  return verdict;
}

bool TruthLatch::frozen() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return frozen_;
}

TruthPayload TruthLatch::value() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return value_;
}

std::string TruthLatch::last_rejection_code() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return last_rejection_code_;
}

std::uint64_t TruthLatch::rejected_count() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return rejected_count_;
}

}  // namespace a1t
