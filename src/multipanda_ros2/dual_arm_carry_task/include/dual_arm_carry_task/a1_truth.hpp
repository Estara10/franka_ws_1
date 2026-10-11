// A1 P1b: pure, offline-testable truth handling.
//
// Deliberately free of ROS so the acceptance logic can be driven directly by
// unit tests: message validation, the latch-once freeze, the frame-mapping
// decision and the planning-scene read-back check. The ROS plumbing (the
// subscription, the planning-scene services) lives in the task node and
// controller and adapts to these types.
//
// Scope: P1b only relates the settled truth to the task's frozen reference and
// the planning scene. It does not move the robot.

#pragma once

#include <cstdint>
#include <mutex>
#include <string>

namespace a1t {

struct Vec3 {
  double x {0.0};
  double y {0.0};
  double z {0.0};
};

struct Quat {
  double x {0.0};
  double y {0.0};
  double z {0.0};
  double w {1.0};
};

// One received truth message, decoded.
struct TruthPayload {
  std::string run_id;
  std::string frame_id;

  Vec3 position {};
  Quat orientation {};
  // Full box size (not half extents), matching the collision object's BOX
  // dimensions: 0.04 x 0.40 x 0.04.
  Vec3 size {};
  double mass_kg {0.0};

  // The simulation's own service returns a zero stamp, so a header stamp is
  // recorded for information only and is never used as a freshness measure.
  bool has_header_stamp {false};
  bool header_stamp_is_zero {true};

  // Reception instants, stamped by the receiver itself.
  double receipt_steady_sec {0.0};
  double receipt_ros_sec {0.0};
  bool receipt_ros_valid {false};
};

struct Verdict {
  bool ok {false};
  std::string reason_code;
  std::string detail;
};

Verdict makeVerdict(bool ok, std::string code, std::string detail);

// Validates a received payload against this run's expectation.
//
// `expected_run_id` must be non-empty: an unset expectation is a configuration
// error, not a licence to accept anything. Sizes are checked against
// [min_size_m, max_size_m] so a wrong-object message is rejected.
Verdict validateTruth(const TruthPayload& payload,
                      const std::string& expected_run_id,
                      double min_size_m,
                      double max_size_m);

// How the truth frame relates to the MoveIt planning frame.
enum class FrameMapping {
  SAME_FRAME,            // truth frame == planning frame: no transform
  WORLD_TO_BASE_LINK,    // documented identity, see resolveFrameMapping docs
  UNSUPPORTED,
};

// Decides how to interpret the truth frame.
//
// `world` and `base_link` are numerically coincident in this workspace: the
// SRDF virtual joint is a fixed world->base_link joint, the URDF mounts both
// arms at base_link +/-0.26, and the MuJoCo scene puts both link0 bodies at
// world +/-0.26. That is source evidence for an IDENTITY mapping -- it is NOT a
// claim that a TF chain exists. Anything else is unsupported and must fail
// rather than be silently coerced.
FrameMapping resolveFrameMapping(const std::string& truth_frame,
                                const std::string& planning_frame);

// Geodesic rotation distance in [0, pi]; q and -q describe the same rotation.
double rotationDistanceRad(const Quat& a, const Quat& b);

struct SceneCheck {
  bool ok {false};
  std::string reason_code;
  std::string detail;
  double pos_err_m {0.0};
  double angle_err_rad {0.0};
  double size_err_m {0.0};
};

// Verifies what the planning scene reports back against the frozen truth.
// Position, orientation (q/-q equivalent) and size are judged independently;
// the first violation determines the reason code while all three errors are
// always reported.
SceneCheck verifySceneReadback(const TruthPayload& expected,
                               const Vec3& actual_position,
                               const Quat& actual_orientation,
                               const Vec3& actual_size,
                               double pos_tol_m,
                               double angle_tol_rad,
                               double size_tol_m);

// Latch-once holder for this round's frozen reference.
//
// The first payload that passes validation freezes the reference; every later
// payload is rejected and counted, so a late or repeated message can never
// change the pose the task was prepared with.
class TruthLatch {
 public:
  Verdict offer(const TruthPayload& payload,
                const std::string& expected_run_id,
                double min_size_m,
                double max_size_m);

  bool frozen() const;
  // Only meaningful when frozen() is true.
  TruthPayload value() const;
  std::string last_rejection_code() const;
  std::uint64_t rejected_count() const;

 private:
  mutable std::mutex mutex_;
  bool frozen_ {false};
  TruthPayload value_ {};
  std::string last_rejection_code_;
  std::uint64_t rejected_count_ {0};
};

}  // namespace a1t
