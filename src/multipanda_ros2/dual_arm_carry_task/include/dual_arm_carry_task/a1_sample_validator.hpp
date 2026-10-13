// A1 P2a: pure, offline-testable sample validity checks.
//
// Covers the parts of P2a that need no ROS: the rotated rod footprint against a
// configured workspace, the support-height/tilt check, and the shared
// pre-grasp/grasp target construction. The ROS-dependent halves (real IK
// against a combined RobotState, and collision against the live planning
// scene) live in the controller and use these types.
//
// Scope: P2a judges a STATIC combined target only. Passing here does NOT mean
// the sample is transportable -- joint pre-grasp PATH validity is P2b, and the
// final acceptance flag stays null until then.

#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "dual_arm_carry_task/a1_truth.hpp"

namespace a1v {

using a1t::Quat;
using a1t::Vec3;

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Per-item outcome
// ---------------------------------------------------------------------------

enum class CheckStatus { PASSED, FAILED, NOT_RUN };

const char* statusName(CheckStatus s);

struct ItemResult {
  CheckStatus status {CheckStatus::NOT_RUN};
  std::string reason_code;
  std::string detail;
};

// ---------------------------------------------------------------------------
// Shared target construction
// ---------------------------------------------------------------------------

// Grasp geometry, each field traced to the A0 implementation rather than
// assumed:
//   grasp_offset_m      -- dual_arm_controller.cpp `final_y_offset = 0.15`
//                          (grasp point 0.15 m along the rod's long axis from
//                          its centre; the rod half-length is 0.20 m, so the
//                          grasp sits 0.05 m inboard of each end)
//   eef_base_yaw_rad    -- the yaw term of A0's `setRPY(M_PI, 0, M_PI/4)`
//   pre_grasp_height_m  -- the `approach_height` launch parameter (0.28)
//   grasp_height_m      -- the `grasp_height` launch parameter (0.13)
//
// Both heights are the Z of the ARM GROUP'S end-effector link (mj_*_link8),
// NOT of the finger tip or the hand_tcp: the arm chain ends at link8 and
// `setPoseTarget` defaults to that link.
struct GraspGeometry {
  double grasp_offset_m {0.15};
  double eef_base_yaw_rad {0.7853981633974483};  // pi/4
  double pre_grasp_height_m {0.28};
  double grasp_height_m {0.13};
};

struct ArmTarget {
  Vec3 position {};
  Quat orientation {};
};

struct DualTarget {
  ArmTarget left;   // reaches the rod's +long-axis end
  ArmTarget right;  // reaches the rod's -long-axis end
};

// Rotation builder matching tf2::Quaternion::setRPY(roll, pitch, yaw), i.e.
// R = Rz(yaw) * Ry(pitch) * Rx(roll). Kept local so this module stays ROS-free.
Quat quatFromRpy(double roll, double pitch, double yaw);

// Yaw of a quaternion about +Z, in (-pi, pi]. Returns NaN for a non-finite or
// degenerate quaternion.
double quatYaw(const Quat& q);

// Rotates a local XY offset by `yaw` about +Z.
Vec3 rotateZ(const Vec3& v, double yaw);

// Builds both arm targets for one height.
//
// The rod's long axis is its local Y, so the left/right identity is the sign of
// the offset along that axis: left = +offset, right = -offset. This mirrors A0
// exactly (left arm at obj+(0,+0.15,0), right arm at obj+(0,-0.15,0), both at
// setRPY(pi, 0, pi/4)) and reduces to it bit-for-bit when yaw = 0.
//
// The end-effector yaw is (rod yaw + base yaw) so that BOTH the grasp offsets
// and the wrist orientation rotate with the rod.
DualTarget buildDualTarget(const a1t::TruthPayload& truth,
                           const GraspGeometry& geom,
                           double height_m,
                           double* yaw_out = nullptr);

DualTarget buildPreGraspTarget(const a1t::TruthPayload& truth,
                               const GraspGeometry& geom,
                               double* yaw_out = nullptr);

DualTarget buildGraspTarget(const a1t::TruthPayload& truth,
                            const GraspGeometry& geom,
                            double* yaw_out = nullptr);

// ---------------------------------------------------------------------------
// Geometry: rotated footprint, workspace and support
// ---------------------------------------------------------------------------

struct WorkspaceBounds {
  double x_min {0.0};
  double x_max {0.0};
  double y_min {0.0};
  double y_max {0.0};
  // Where these numbers come from. Recorded verbatim in the result so a
  // configured envelope is never mistaken for a physical edge.
  std::string source;
};

// Axis-aligned bounding box of the yaw-rotated rod footprint, plus the signed
// clearance to each workspace edge (positive = inside).
struct Footprint {
  double x_min {0.0};
  double x_max {0.0};
  double y_min {0.0};
  double y_max {0.0};
  double clearance_x_min {0.0};
  double clearance_x_max {0.0};
  double clearance_y_min {0.0};
  double clearance_y_max {0.0};
  double min_clearance_m {0.0};
};

Footprint computeFootprint(const Vec3& center, const Vec3& size, double yaw_rad);

struct SupportCriteria {
  double support_z {0.0};
  double resting_z_tol {0.003};
  double max_tilt_deg {2.0};
};

// Full static-geometry judgement. Fails loudly on a bad payload; the footprint
// is never silently shrunk to make a sample fit.
struct GeometryResult {
  ItemResult item;
  Footprint footprint;
  double center_z {0.0};
  double tilt_deg {0.0};
  double yaw_rad {0.0};
};

GeometryResult checkGeometry(const a1t::TruthPayload& truth,
                             const WorkspaceBounds& bounds,
                             const SupportCriteria& support);

// ---------------------------------------------------------------------------
// Joint-space helpers (driven by the ROS side, judged here)
// ---------------------------------------------------------------------------

struct JointLimitCheck {
  bool ok {false};
  std::size_t offending_index {0};
  std::string offending_name;
  double value {0.0};
  double lower {0.0};
  double upper {0.0};
  double worst_margin_rad {0.0};  // smallest distance to either limit
};

JointLimitCheck checkJointLimits(const std::vector<double>& positions,
                                 const std::vector<double>& lower,
                                 const std::vector<double>& upper,
                                 const std::vector<std::string>& names);

// Largest absolute per-joint change between two configurations, in rad.
double maxAbsJointDelta(const std::vector<double>& a, const std::vector<double>& b);

struct PoseError {
  double pos_m {0.0};
  double angle_rad {0.0};
};

// Residual of the solved state against the requested target. Angle uses the
// geodesic distance, so q and -q compare equal.
PoseError poseError(const ArmTarget& target, const ArmTarget& actual);

// ---------------------------------------------------------------------------
// Collision-matrix bookkeeping (pure classification; the ACM itself is ROS)
// ---------------------------------------------------------------------------

// True when the two link names belong to different arms. Such an ACM entry
// would hide a dual-arm mutual collision.
bool isCrossArmPair(const std::string& a, const std::string& b);

// True when exactly one of the two names is the given world object.
bool involvesObject(const std::string& a, const std::string& b, const std::string& object_id);

// True when exactly one of the two names is the rod object.
bool isRodPair(const std::string& a, const std::string& b, const std::string& rod_id);

// True when exactly one of the two names is the support-surface object.
bool isSupportPair(const std::string& a, const std::string& b, const std::string& support_id);

// Links that are rigidly mounted to the world. Their contact with the support
// surface is the fixed mounting, not a free-link collision, and is the only
// robot-support contact that may be allowed.
bool isMountingLink(const std::string& name);

// True for a gripper finger link, the only links allowed to touch the rod in a
// grasp pose.
bool isFingerLink(const std::string& name);

// ---------------------------------------------------------------------------
// Path segment sampling (A1 P2b)
// ---------------------------------------------------------------------------

// How many equal intervals a segment must be split into so that no single-joint
// step exceeds `max_step_rad`. A zero-length segment needs one interval. This is
// discrete sampling, not a continuous collision guarantee.
std::size_t subdivisionsFor(double max_joint_delta_rad, double max_step_rad);

// Linear interpolation between two configurations; t is clamped to [0, 1].
// Returns an empty vector if the two configurations differ in size.
std::vector<double> interpolateJoints(const std::vector<double>& a,
                                      const std::vector<double>& b,
                                      double t);

// ---------------------------------------------------------------------------
// Bounded dispatch (A1 P3a-1)
// ---------------------------------------------------------------------------

// Result of a bounded, dispatched call.
//
// `worker_finished` says whether the worker ended inside the grace period after
// the timeout. A worker that did not is detached; it stays safe because every
// value it touches is shared, but it must NOT influence the verdict afterwards.
struct BoundedExecOutcome {
  enum class Status { COMPLETED, TIMEOUT, EXCEPTION };
  Status status {Status::TIMEOUT};
  int value {0};
  bool stop_requested {false};
  bool worker_finished {false};
  double elapsed_sec {0.0};
  // When the worker actually finished, in seconds from the moment the bounded
  // call began; -1 when it never finished. Whether the result counts as in time
  // is decided from THIS instant, never from when the polling loop happened to
  // wake up.
  double completed_at_sec {-1.0};
  double timeout_sec {0.0};
};

// Runs `work` on a worker thread and waits at most `timeout_sec` for it.
//
// Ownership is the point: the worker's result slot, and everything else it
// touches, is shared rather than a stack reference, so a worker that outlives
// the wait cannot write to freed memory.
//
// Timeliness is decided by the worker's own completion instant, recorded before
// the deadline check:
//   * a worker that finished within the budget counts as COMPLETED even if the
//     polling loop only noticed it much later (a long poll interval must not
//     turn an on-time result into a timeout);
//   * a worker that finished after the budget stays TIMEOUT even though its
//     value is present by the time the loop observes it (a completed slot is
//     not by itself evidence of an in-time result).
// A stop is only requested while the worker is still running; a late result
// that already ended needs no stop request and still reports TIMEOUT.
BoundedExecOutcome runBoundedExec(const std::function<int()>& work,
                                  const std::function<void()>& on_timeout,
                                  double timeout_sec,
                                  double grace_sec,
                                  double poll_interval_sec = 0.01);

// ---------------------------------------------------------------------------
// Trajectory structural validation (A1 P3a-1)
// ---------------------------------------------------------------------------

struct JointNameCheck {
  bool ok {false};
  std::string reason_code;
  std::string detail;
  std::size_t missing {0};
  std::size_t duplicates {0};
  std::size_t unknown {0};
  std::string first_unknown;
  std::string first_duplicate;
};

// The plan must name every expected joint exactly once: no missing joint (which
// would silently drop one arm), no duplicate, and nothing outside the expected
// set (which would mean the plan is for a different robot).
JointNameCheck validateJointNames(const std::vector<std::string>& names,
                                  const std::vector<std::string>& expected);

struct TimeCheck {
  bool ok {false};
  std::string reason_code;
  std::string detail;
  double first_sec {0.0};
  double last_sec {0.0};
  std::size_t bad_index {0};
};

// time_from_start must be finite, non-negative and strictly increasing, and the
// first point must sit at t = 0.
TimeCheck validateTimeFromStart(const std::vector<double>& times_sec);

struct DerivativeCheck {
  bool ok {false};
  std::string reason_code;
  std::string detail;
  // Joints whose limit is not declared in the model. They are skipped and
  // counted -- never treated as a zero limit, and never reported as compliant.
  std::size_t unbounded {0};
  std::size_t checked {0};
  double worst_ratio {0.0};
  std::string worst_joint;
};

// Validates one trajectory point's velocity (or acceleration) field.
//
// A non-finite component always fails. A joint with no declared limit is
// skipped and counted in `unbounded`; the caller must report that rather than
// claim the field was fully checked. `limits` is the absolute limit per joint,
// taken from the model actually in use -- never invented.
DerivativeCheck checkDerivativeField(const std::vector<double>& values,
                                     const std::vector<double>& limits,
                                     const std::vector<bool>& bounded,
                                     const std::vector<std::string>& names);

// ---------------------------------------------------------------------------
// Local hand-model substitution (A1 P2a)
// ---------------------------------------------------------------------------

// Axis-aligned bounding box, used as the conservative stand-in for the hand's
// convex collision mesh. Not a shrink-wrapped approximation: it must contain
// every vertex of the mesh.
struct Aabb {
  Vec3 min {};
  Vec3 max {};
  bool valid {false};

  Vec3 center() const;
  Vec3 size() const;
};

Aabb aabbOfPoints(const std::vector<Vec3>& points);

// True when every point lies inside the box (inclusive, with a small epsilon).
// Used to assert the substituted box really is conservative.
bool aabbContainsAll(const Aabb& box, const std::vector<Vec3>& points, double eps = 1e-9);

// Reads the triangle vertices of a BINARY STL. Returns false with a reason
// rather than returning an empty model, so a load failure can never be mistaken
// for "the hand has no collision geometry".
bool readBinaryStlVertices(const std::string& path, std::vector<Vec3>& out, std::string& error);

// ---------------------------------------------------------------------------
// Aggregated result
// ---------------------------------------------------------------------------

struct SampleValidation {
  ItemResult geometry;
  ItemResult ik;
  ItemResult collision;
  // Recorded so a reader can tell a static pass from a transportable sample.
  ItemResult path_validation;  // always NOT_RUN in P2a

  bool allStaticPassed() const;
  std::string summary() const;
};

}  // namespace a1v
