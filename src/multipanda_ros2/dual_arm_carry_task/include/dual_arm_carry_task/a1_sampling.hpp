// A1 P1a: pure, offline-testable core.
//
// This header deliberately contains NO ROS dependency. Everything the P1a
// acceptance criteria require to be verifiable offline lives here:
//   * deterministic candidate sampling (splitmix64 + explicit integer->double map)
//   * rod footprint geometry
//   * the settling (stability) criterion
//   * the single-candidate orchestration state machine, driven through an
//     injected transport + clock so service faults can be injected in tests.
//
// The ROS side (parameters, service clients, polling, manifest file writing)
// lives in src/a1_scene_runner_node.cpp and implements the interfaces below.
//
// Scope note: P1a only establishes "candidate -> applied to simulation ->
// stable read-back -> manifest". It does not validate samples, does not feed
// the task, and does not touch the planning scene. See
// docs/deliverables/A1_P0_接口核查与实现方案.md section 4 and 10.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace a1 {

// ---------------------------------------------------------------------------
// Checksums
// ---------------------------------------------------------------------------

// FNV-1a 64-bit. This is a change-detection checksum, NOT a cryptographic
// digest; manifests label it as such.
uint64_t fnv1a64(const std::string& data);

// ---------------------------------------------------------------------------
// Basic geometry types
// ---------------------------------------------------------------------------

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

// Rotation of `yaw_rad` about world +Z, as a unit quaternion.
Quat quatFromYaw(double yaw_rad);

// Geodesic rotation distance in [0, pi]. Uses |dot| so q and -q are the same
// rotation. Returns NaN if either quaternion is not finite.
//
// Precision: the result has a floor of roughly 3e-8 rad even for identical
// inputs, because a quaternion built from cos/sin does not satisfy |q| == 1 to
// the last bit. Do not compare against tolerances tighter than ~1e-6 rad.
double rotationDistanceRad(const Quat& a, const Quat& b);

// ZYX yaw / roll / pitch extracted from a unit quaternion.
double yawFromQuat(const Quat& q);
double rollFromQuat(const Quat& q);
double pitchFromQuat(const Quat& q);

bool isFinite(const Quat& q);
bool isFinite(const Vec3& v);
// Normalises in place. Returns false when the norm is zero or non-finite.
bool normalize(Quat& q);
// True when every component is finite AND the norm is within `tol` of 1.
// Finite components alone do not make a usable rotation.
bool isUnitQuat(const Quat& q, double tol = 1e-6);

// ---------------------------------------------------------------------------
// Sampling configuration
// ---------------------------------------------------------------------------

struct SamplingConfig {
  // Versioning
  std::string algorithm_version;
  std::string sampling_range_version;
  bool range_is_reachability_verified {false};

  // Seed. Kept signed so that a negative value from configuration can be
  // rejected explicitly instead of silently wrapping.
  int64_t seed_raw {0};

  // Sampling ranges. Lengths in metres, angles in degrees. Frame is `world`.
  double x_min {0.0};
  double x_max {0.0};
  double y_min {0.0};
  double y_max {0.0};
  double yaw_deg_min {0.0};
  double yaw_deg_max {0.0};
  double roll_deg {0.0};   // not sampled; rod is assumed lying flat
  double pitch_deg {0.0};  // not sampled

  // Rod geometry, from objects.xml <geom size="0.02 0.2 0.02">.
  double rod_half_length {0.0};
  double rod_half_width {0.0};
  double rod_half_height {0.0};

  // Support surface, from dual_scene.xml floor plane (no `pos` => z = 0).
  double support_surface_z {0.0};
  // Explicit clearance above the resting height when applying the pose.
  double apply_clearance_m {0.0};

  int32_t max_total_candidates {0};

  // Canonical, order-stable serialisation of every field that influences
  // sampling. Written into the manifest so the checksum can be recomputed
  // independently.
  std::string canonicalString() const;
  uint64_t checksum() const;
};

struct ValidationResult {
  bool ok {false};
  std::string reason_code;  // machine readable, empty when ok
  std::string detail;       // human readable, empty when ok
};

ValidationResult validateSamplingConfig(const SamplingConfig& cfg);

// A run identifier becomes a directory name, so it must not be able to escape
// the manifest root or address an existing path.
ValidationResult validateRunId(const std::string& run_id);

// Rod centre height used when applying a candidate:
//   support_surface_z + rod_half_height + apply_clearance_m
double rodCenterZ(const SamplingConfig& cfg);

// ---------------------------------------------------------------------------
// Deterministic sampling
// ---------------------------------------------------------------------------

struct Candidate {
  int32_t index {0};
  double x {0.0};
  double y {0.0};
  double z {0.0};
  double yaw_rad {0.0};
  Quat quat {};
};

// Deterministic sampler.
//
// Contract: same seed + same candidate index + same complete SamplingConfig
// (including algorithm_version) => same candidate. See the .cpp for the exact
// integer->double mapping; std::uniform_real_distribution is deliberately not
// used because its algorithm is unspecified by the standard.
class Sampler {
 public:
  explicit Sampler(SamplingConfig cfg);

  const SamplingConfig& config() const { return cfg_; }

  // `index` must be in [0, max_total_candidates).
  Candidate candidate(int32_t index) const;

  // Candidates [0, count), clamped to max_total_candidates.
  std::vector<Candidate> firstN(int32_t count) const;

 private:
  SamplingConfig cfg_;
};

// Builds a candidate from explicit values rather than from the sampler.
//
// This exists so that known test poses (e.g. yaw exactly 0) can be applied
// during verification; the uniform sampler cannot produce an exact value.
// `index` is only a label. Returns false when a value is not finite. The
// configured sampling range is deliberately NOT enforced here.
bool makeExplicitCandidate(const SamplingConfig& cfg,
                           int32_t index,
                           double x,
                           double y,
                           double yaw_rad,
                           Candidate& out);

// ---------------------------------------------------------------------------
// Footprint geometry (input for the P2 boundary check)
// ---------------------------------------------------------------------------

struct Footprint {
  // Four corners of the rod's bottom face, counter-clockwise in world XY,
  // at z = support_surface_z.
  std::array<Vec3, 4> corners {};
  double min_x {0.0};
  double max_x {0.0};
  double min_y {0.0};
  double max_y {0.0};
};

Footprint rodFootprint(const SamplingConfig& cfg, const Candidate& c);

// ---------------------------------------------------------------------------
// Settling / stability criterion
// ---------------------------------------------------------------------------

// One polled observation of the body state.
//
// The request and response instants are both kept: the value describes the
// body at the instant the server answered, so `ros_time_sec` / `steady_time_sec`
// are the RESPONSE times. Using the pre-request instant as the sample time
// would misdate every observation by the service latency.
struct PoseSample {
  double request_ros_sec {0.0};
  bool request_ros_valid {false};
  double request_steady_sec {0.0};

  double ros_time_sec {0.0};      // response instant, node clock (sim time)
  bool ros_time_valid {false};
  double steady_time_sec {0.0};   // response instant, monotonic wall clock

  Vec3 pos {};
  Quat quat {};

  double twist_lin_norm {0.0};
  double twist_ang_norm {0.0};
  bool twist_valid {false};
};

struct StabilityCriteria {
  double min_sim_window_sec {0.0};
  double max_window_pos_tol {0.0};
  double max_window_angle_tol_deg {0.0};
  double max_twist_linear_tol {0.0};
  double max_twist_angular_tol {0.0};
  double resting_z_tol {0.0};
  double resting_tilt_tol_deg {0.0};
  double min_clock_advance_ratio {0.0};
};

enum class StabilityOutcome {
  STABLE,
  MODEL_RELOADED,
  INSUFFICIENT_SAMPLES,
  CLOCK_INVALID,
  CLOCK_NOT_ADVANCING,
  CLOCK_JUMP,
  SIM_WINDOW_TOO_SHORT,
  CLOCK_RATIO_TOO_LOW,
  INVALID_TWIST,
  TWIST_EXCEEDED,
  DRIFT_EXCEEDED,
  NOT_RESTING,
  PAUSED,
};

struct StabilityResult {
  StabilityOutcome outcome {StabilityOutcome::INSUFFICIENT_SAMPLES};
  std::string reason_code;
  std::string detail;

  double sim_window_sec {0.0};
  double steady_window_sec {0.0};
  double clock_advance_ratio {0.0};
  double max_window_pos_delta {0.0};
  double max_window_angle_delta_deg {0.0};
  double max_twist_lin {0.0};
  double max_twist_ang {0.0};
};

std::string stabilityReasonCode(StabilityOutcome outcome);

// Evaluates a time-ordered window of samples.
//
// `rod_rest_z` is the theoretical resting centre height
// (support_surface_z + rod_half_height). `sim_paused` and `model_reloaded`
// come from the simulation info service and can only ever prevent success.
//
// The drift test uses the maximum difference over ALL sample pairs in the
// window (not adjacent pairs), so a slow monotonic drift is rejected.
StabilityResult evaluateStability(const std::vector<PoseSample>& window,
                                  const StabilityCriteria& crit,
                                  double rod_rest_z,
                                  bool sim_paused,
                                  bool model_reloaded);

// ---------------------------------------------------------------------------
// Transport / clock abstractions (implemented by the ROS node, faked in tests)
// ---------------------------------------------------------------------------

enum class WriteOutcome {
  SUCCESS,
  REJECTED,     // service answered, success == false
  TIMEOUT,      // no answer within the deadline; server side may still act
  UNAVAILABLE,  // service not reachable
};

std::string toString(WriteOutcome outcome);

struct BodyState {
  bool ok {false};
  std::string status_message;
  // Frame the service reported for the pose. Recorded rather than assumed, so a
  // published truth message states the frame it was actually given.
  std::string frame_id;
  // Body mass as reported by the service. Recorded so the 0.64 kg rod baseline
  // is verified at runtime rather than only derived from the model file.
  double mass {0.0};
  Vec3 pos {};
  Quat quat {};
  Vec3 twist_lin {};
  Vec3 twist_ang {};
  // True when the service returned an all-zero header stamp. This is the
  // observed behaviour of GetBodyStateCB (ros_api.cpp:282,292) and must be
  // recorded rather than treated as a real timestamp.
  bool pose_stamp_is_zero {true};
};

struct SimInfo {
  bool ok {false};
  std::string status_message;
  bool model_valid {false};
  std::string model_path;
  uint32_t load_count {0};
  bool paused {false};
};

class BodyStateTransport {
 public:
  virtual ~BodyStateTransport() = default;

  // Records the request/response steady-clock and ROS-time instants.
  struct WriteTiming {
    double request_ros_sec {0.0};
    bool request_ros_valid {false};
    double request_steady_sec {0.0};
    double response_ros_sec {0.0};
    bool response_ros_valid {false};
    double response_steady_sec {0.0};
  };

  // Waits (bounded, steady clock) until the required services are reachable.
  virtual bool waitForServices(double timeout_sec, std::string& message) = 0;

  // `budget_sec` is the time left in the caller's settling budget. Every
  // blocking call must be bounded by it; exceeding the budget is reported as
  // UNAVAILABLE / a failed read rather than being allowed to overrun. Note that
  // abandoning a request locally does not cancel it on the server.
  virtual SimInfo getSimInfo(double budget_sec) = 0;
  virtual BodyState getState(double budget_sec, PoseSample& timing_out) = 0;
  virtual WriteOutcome setPose(const Candidate& c, double budget_sec, WriteTiming& timing,
                               std::string& status_message) = 0;
};

class Clock {
 public:
  virtual ~Clock() = default;
  virtual double steadyNowSec() = 0;
  virtual double rosNowSec() = 0;
  virtual bool rosTimeValid() = 0;
  // Sleeps until the given steady instant (may return early).
  virtual void sleepUntil(double steady_deadline_sec) = 0;
};

// ---------------------------------------------------------------------------
// Single-candidate orchestration
// ---------------------------------------------------------------------------

struct ApplyPolicy {
  // Optional abort flag (e.g. set from a signal handler). Checked before the
  // write and once per poll, so an interrupted run still produces a manifest
  // instead of dying without a record. Only ever prevents success.
  const std::atomic<bool>* abort_flag {nullptr};
  double poll_rate_hz {50.0};
  double service_wait_timeout_sec {10.0};
  double stability_timeout_sec {5.0};
  // Upper bound on a single blocking service call. Further clamped by the
  // remaining settling budget.
  double service_call_timeout_sec {2.0};
  // How many polls between two sim-info queries used for reload detection.
  int32_t sim_info_check_every {10};
  // Upper bound on retained window samples.
  int32_t max_window_samples {200};
  // P1a writes exactly once. Declared so the policy is explicit and validated
  // rather than silently ignoring a configuration that asks for retries.
  int32_t max_write_attempts_per_run {1};
};

// Validates the numeric policy and criteria actually used by the run: finite,
// positive, in range, and integral where required. A configuration that asks
// for retries or disables health checking is rejected rather than ignored.
ValidationResult validateApplyParameters(const ApplyPolicy& policy,
                                         const StabilityCriteria& crit);

struct RunResult {
  // Every change of the settling reason, in poll order. Makes a run's
  // progression auditable instead of only showing the final outcome.
  struct SettleTraceEntry {
    int32_t poll {0};
    std::string reason_code;
  };
  std::vector<SettleTraceEntry> settle_trace;

  std::string scene_state {"NOT_RUN"};
  std::string outcome_code {"NOT_RUN"};
  std::string detail;

  bool write_attempted {false};
  WriteOutcome write_outcome {WriteOutcome::UNAVAILABLE};
  std::string write_status_message;
  BodyStateTransport::WriteTiming write_timing {};

  SimInfo sim_info_before {};
  SimInfo sim_info_after {};
  // Final health confirmation taken before success is reported. Success
  // requires this to agree with `sim_info_before`, so a health query that fails
  // or reports a changed model can never be reported as a clean run.
  SimInfo sim_info_final {};
  bool health_confirmed {false};
  std::string health_detail;
  BodyState baseline_state {};

  // The window used by the settling criterion (slides with simulated time).
  std::vector<PoseSample> samples {};
  // Every poll of this run, for raw-data output. Not used by the criterion.
  std::vector<PoseSample> history {};
  StabilityResult stability {};
  BodyState readback {};

  Candidate candidate {};
  bool candidate_generated {false};

  double steady_span_sec {0.0};
  double ros_span_sec {0.0};
  bool ros_span_valid {false};
};

// Applies one candidate and waits for the simulation to settle.
//
// Failure semantics: a write that times out or fails stops the run. The caller
// must NOT retry and must NOT move on to another candidate, because a ROS
// service timeout does not cancel server-side work and a failed response does
// not imply the write had no effect.
RunResult applyCandidateAndSettle(const SamplingConfig& cfg,
                                  const StabilityCriteria& crit,
                                  const ApplyPolicy& policy,
                                  int32_t candidate_index,
                                  BodyStateTransport& transport,
                                  Clock& clock);

// Same, but with an already-built candidate (used by the explicit-pose path).
RunResult applyCandidateAndSettle(const SamplingConfig& cfg,
                                  const StabilityCriteria& crit,
                                  const ApplyPolicy& policy,
                                  const Candidate& candidate,
                                  BodyStateTransport& transport,
                                  Clock& clock);

}  // namespace a1
