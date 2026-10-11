// A1 P1a: pure, offline-testable core. See a1_sampling.hpp for scope notes.

#include "dual_arm_carry_task/a1_sampling.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>

namespace a1 {
namespace {

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

inline bool isFiniteDouble(double v) { return std::isfinite(v); }

// Compact, round-trip exact rendering of a double. %.17g is exact for IEEE-754
// binary64 and is locale-independent for the C locale used here.
void appendNumber(std::ostringstream& os, double v)
{
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  os << buf;
}

void appendU64(std::ostringstream& os, uint64_t v)
{
  os << v;
}

// splitmix64 finaliser. Defined purely by integer arithmetic so the stream is
// identical across standard libraries and platforms.
inline uint64_t splitmix64(uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  uint64_t z = x;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

// One 64-bit value for a (candidate index, channel) pair. Documented formula:
//   v = splitmix64(seed ^ (index * 0x9E3779B97F4A7C15) ^ (channel * 0xBF58476D1CE4E5B9))
// Channels are 1..3 so that channel 0 (which would collapse to the bare seed)
// is never used.
inline uint64_t channelValue(uint64_t seed, int32_t index, uint32_t channel)
{
  const uint64_t mixed = seed ^ (static_cast<uint64_t>(index) * 0x9E3779B97F4A7C15ULL) ^
                         (static_cast<uint64_t>(channel) * 0xBF58476D1CE4E5B9ULL);
  return splitmix64(mixed);
}

// Explicit integer -> [0,1) mapping: take the top 53 bits as a double mantissa.
// This replaces std::uniform_real_distribution, whose algorithm the C++
// standard does not specify.
inline double unitFromU64(uint64_t v)
{
  return static_cast<double>(v >> 11) * (1.0 / 9007199254740992.0);  // 2^-53
}

inline double sampleUniform(uint64_t seed, int32_t index, uint32_t channel, double lo, double hi)
{
  const double u = unitFromU64(channelValue(seed, index, channel));
  return lo + u * (hi - lo);
}

struct RangeCheck {
  bool ok {true};
  std::string code;
  std::string detail;
};

void fail(RangeCheck& c, const std::string& code, const std::string& detail)
{
  if (c.ok) {
    c.ok = false;
    c.code = code;
    c.detail = detail;
  }
}

void requireFinite(RangeCheck& c, double v, const char* name)
{
  if (!isFiniteDouble(v)) {
    fail(c, "E_CONFIG_NOT_FINITE", std::string(name) + " is not finite");
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Checksums
// ---------------------------------------------------------------------------

uint64_t fnv1a64(const std::string& data)
{
  uint64_t h = 0xCBF29CE484222325ULL;
  for (unsigned char ch : data) {
    h ^= static_cast<uint64_t>(ch);
    h *= 0x100000001B3ULL;
  }
  return h;
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

Quat quatFromYaw(double yaw_rad)
{
  Quat q;
  q.w = std::cos(yaw_rad * 0.5);
  q.z = std::sin(yaw_rad * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  return q;
}

bool isFinite(const Quat& q)
{
  return isFiniteDouble(q.x) && isFiniteDouble(q.y) && isFiniteDouble(q.z) && isFiniteDouble(q.w);
}

bool isFinite(const Vec3& v)
{
  return isFiniteDouble(v.x) && isFiniteDouble(v.y) && isFiniteDouble(v.z);
}

bool normalize(Quat& q)
{
  if (!isFinite(q)) {
    return false;
  }
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(n > 0.0) || !isFiniteDouble(n)) {
    return false;
  }
  q.x /= n;
  q.y /= n;
  q.z /= n;
  q.w /= n;
  return true;
}

bool isUnitQuat(const Quat& q, double tol)
{
  if (!isFinite(q)) {
    return false;
  }
  const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!(n2 > 0.0) || !isFiniteDouble(n2)) {
    return false;
  }
  return std::fabs(std::sqrt(n2) - 1.0) <= tol;
}

double rotationDistanceRad(const Quat& a, const Quat& b)
{
  if (!isFinite(a) || !isFinite(b)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  dot = std::fabs(dot);  // q and -q are the same rotation
  dot = std::min(1.0, std::max(0.0, dot));
  if (dot >= 1.0) {
    return 0.0;
  }
  // atan2 form instead of acos: far better conditioned for nearly equal
  // rotations. Even so the result has a precision floor of roughly 3e-8 rad
  // for identical inputs, because a quaternion built from cos/sin does not
  // satisfy |q| == 1 to the last bit. That floor is ~5 orders of magnitude
  // below the tolerances used here, so it is not corrected artificially.
  const double s = std::sqrt(std::max(0.0, 1.0 - dot * dot));
  return 2.0 * std::atan2(s, dot);
}

double yawFromQuat(const Quat& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double rollFromQuat(const Quat& q)
{
  return std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                    1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

double pitchFromQuat(const Quat& q)
{
  const double s = 2.0 * (q.w * q.y - q.z * q.x);
  return std::asin(std::min(1.0, std::max(-1.0, s)));
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

std::string SamplingConfig::canonicalString() const
{
  std::ostringstream os;
  os << "algorithm_version=" << algorithm_version << '\n';
  os << "sampling_range_version=" << sampling_range_version << '\n';
  os << "range_is_reachability_verified=" << (range_is_reachability_verified ? 1 : 0) << '\n';
  os << "seed_raw=";
  appendU64(os, static_cast<uint64_t>(seed_raw));
  os << '\n';
  const double fields[] = {x_min,        x_max,           y_min,
                           y_max,        yaw_deg_min,     yaw_deg_max,
                           roll_deg,     pitch_deg,       rod_half_length,
                           rod_half_width, rod_half_height, support_surface_z,
                           apply_clearance_m};
  const char* names[] = {"x_min",           "x_max",           "y_min",
                         "y_max",           "yaw_deg_min",     "yaw_deg_max",
                         "roll_deg",        "pitch_deg",       "rod_half_length",
                         "rod_half_width",  "rod_half_height", "support_surface_z",
                         "apply_clearance_m"};
  for (std::size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
    os << names[i] << '=';
    appendNumber(os, fields[i]);
    os << '\n';
  }
  os << "max_total_candidates=" << max_total_candidates << '\n';
  return os.str();
}

uint64_t SamplingConfig::checksum() const
{
  return fnv1a64(canonicalString());
}

double rodCenterZ(const SamplingConfig& cfg)
{
  return cfg.support_surface_z + cfg.rod_half_height + cfg.apply_clearance_m;
}

ValidationResult validateSamplingConfig(const SamplingConfig& cfg)
{
  RangeCheck c;

  if (cfg.algorithm_version.empty()) {
    fail(c, "E_CONFIG_EMPTY_ALGORITHM_VERSION", "algorithm_version must not be empty");
  }
  if (cfg.seed_raw < 0) {
    fail(c, "E_CONFIG_BAD_SEED", "seed must be >= 0");
  }

  requireFinite(c, cfg.x_min, "x_min");
  requireFinite(c, cfg.x_max, "x_max");
  requireFinite(c, cfg.y_min, "y_min");
  requireFinite(c, cfg.y_max, "y_max");
  requireFinite(c, cfg.yaw_deg_min, "yaw_deg_min");
  requireFinite(c, cfg.yaw_deg_max, "yaw_deg_max");
  requireFinite(c, cfg.roll_deg, "roll_deg");
  requireFinite(c, cfg.pitch_deg, "pitch_deg");
  requireFinite(c, cfg.rod_half_length, "rod_half_length");
  requireFinite(c, cfg.rod_half_width, "rod_half_width");
  requireFinite(c, cfg.rod_half_height, "rod_half_height");
  requireFinite(c, cfg.support_surface_z, "support_surface_z");
  requireFinite(c, cfg.apply_clearance_m, "apply_clearance_m");

  if (c.ok) {
    if (!(cfg.x_min < cfg.x_max)) {
      fail(c, "E_CONFIG_BAD_RANGE", "x_min must be < x_max");
    } else if (!(cfg.y_min < cfg.y_max)) {
      fail(c, "E_CONFIG_BAD_RANGE", "y_min must be < y_max");
    } else if (!(cfg.yaw_deg_min < cfg.yaw_deg_max)) {
      fail(c, "E_CONFIG_BAD_RANGE", "yaw_deg_min must be < yaw_deg_max");
    } else if (cfg.yaw_deg_min < -180.0 || cfg.yaw_deg_max > 180.0) {
      fail(c, "E_CONFIG_BAD_RANGE", "yaw range must lie within [-180, 180] deg");
    } else if (!(cfg.rod_half_length > 0.0) || !(cfg.rod_half_width > 0.0) ||
               !(cfg.rod_half_height > 0.0)) {
      fail(c, "E_CONFIG_BAD_GEOMETRY", "rod half extents must be > 0");
    } else if (cfg.apply_clearance_m < 0.0) {
      fail(c, "E_CONFIG_BAD_CLEARANCE", "apply_clearance_m must be >= 0");
    } else if (cfg.max_total_candidates < 1) {
      fail(c, "E_CONFIG_BAD_CANDIDATE_COUNT", "max_total_candidates must be >= 1");
    } else if (std::fabs(cfg.roll_deg) > 1e-6 || std::fabs(cfg.pitch_deg) > 1e-6) {
      // The applied height and footprint geometry assume the rod lies flat with
      // yaw only. Supporting tilted rods is a separate task, so reject it here
      // rather than silently producing a pose the rest of P1a misinterprets.
      fail(c, "E_CONFIG_TILT_UNSUPPORTED",
           "P1a only supports roll_deg == pitch_deg == 0 (rod lying flat)");
    }
  }

  ValidationResult r;
  r.ok = c.ok;
  r.reason_code = c.code;
  r.detail = c.detail;
  return r;
}

ValidationResult validateRunId(const std::string& run_id)
{
  RangeCheck c;
  if (run_id.empty()) {
    fail(c, "E_RUN_ID_EMPTY", "run_id must not be empty");
  } else if (run_id.size() > 128) {
    fail(c, "E_RUN_ID_TOO_LONG", "run_id must be at most 128 characters");
  } else if (run_id == "." || run_id == "..") {
    fail(c, "E_RUN_ID_DANGEROUS", "run_id must not be '.' or '..'");
  } else if (run_id.find('/') != std::string::npos || run_id.find('\\') != std::string::npos) {
    fail(c, "E_RUN_ID_SEPARATOR", "run_id must not contain path separators");
  } else {
    for (unsigned char ch : run_id) {
      const bool allowed = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                           (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.';
      if (!allowed) {
        fail(c, "E_RUN_ID_CHARSET",
             "run_id may only contain letters, digits, '_', '-' and '.'");
        break;
      }
    }
  }

  ValidationResult r;
  r.ok = c.ok;
  r.reason_code = c.code;
  r.detail = c.detail;
  return r;
}

ValidationResult validateApplyParameters(const ApplyPolicy& policy,
                                         const StabilityCriteria& crit)
{
  RangeCheck c;
  auto finite = [&](double v, const char* name) { requireFinite(c, v, name); };

  finite(policy.poll_rate_hz, "poll_rate_hz");
  finite(policy.service_wait_timeout_sec, "service_wait_timeout_sec");
  finite(policy.stability_timeout_sec, "stability_timeout_sec");
  finite(policy.service_call_timeout_sec, "service_call_timeout_sec");
  finite(crit.min_sim_window_sec, "min_sim_window_sec");
  finite(crit.max_window_pos_tol, "max_window_pos_tol");
  finite(crit.max_window_angle_tol_deg, "max_window_angle_tol_deg");
  finite(crit.max_twist_linear_tol, "max_twist_linear_tol");
  finite(crit.max_twist_angular_tol, "max_twist_angular_tol");
  finite(crit.resting_z_tol, "resting_z_tol");
  finite(crit.resting_tilt_tol_deg, "resting_tilt_tol_deg");
  finite(crit.min_clock_advance_ratio, "min_clock_advance_ratio");

  if (c.ok) {
    if (!(policy.poll_rate_hz > 0.0) || policy.poll_rate_hz > 1000.0) {
      fail(c, "E_POLICY_BAD_RANGE", "poll_rate_hz must be in (0, 1000]");
    } else if (!(policy.service_wait_timeout_sec > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "service_wait_timeout_sec must be > 0");
    } else if (!(policy.stability_timeout_sec > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "stability_timeout_sec must be > 0");
    } else if (!(policy.service_call_timeout_sec > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "service_call_timeout_sec must be > 0");
    } else if (policy.service_call_timeout_sec > policy.stability_timeout_sec) {
      // A single call allowed to outlast the whole budget would defeat the
      // deadline checks.
      fail(c, "E_POLICY_BAD_RANGE",
           "service_call_timeout_sec must not exceed stability_timeout_sec");
    } else if (!(crit.min_sim_window_sec > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "min_sim_window_sec must be > 0");
    } else if (!(crit.max_window_pos_tol > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "max_window_pos_tol must be > 0");
    } else if (!(crit.max_window_angle_tol_deg > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "max_window_angle_tol_deg must be > 0");
    } else if (!(crit.max_twist_linear_tol > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "max_twist_linear_tol must be > 0");
    } else if (!(crit.max_twist_angular_tol > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "max_twist_angular_tol must be > 0");
    } else if (!(crit.resting_z_tol > 0.0)) {
      fail(c, "E_POLICY_BAD_RANGE", "resting_z_tol must be > 0");
    } else if (!(crit.resting_tilt_tol_deg > 0.0) || crit.resting_tilt_tol_deg > 90.0) {
      fail(c, "E_POLICY_BAD_RANGE", "resting_tilt_tol_deg must be in (0, 90]");
    } else if (!(crit.min_clock_advance_ratio > 0.0) || crit.min_clock_advance_ratio > 10.0) {
      fail(c, "E_POLICY_BAD_RANGE", "min_clock_advance_ratio must be in (0, 10]");
    } else if (policy.sim_info_check_every < 1) {
      fail(c, "E_POLICY_BAD_RANGE",
           "sim_info_check_every must be >= 1 (health checking may not be disabled)");
    } else if (policy.max_window_samples < 3) {
      fail(c, "E_POLICY_BAD_RANGE", "max_window_samples must be >= 3");
    } else if (policy.max_write_attempts_per_run != 1) {
      fail(c, "E_POLICY_BAD_WRITE_ATTEMPTS",
           "P1a requires exactly one write attempt per run");
    }
  }

  ValidationResult r;
  r.ok = c.ok;
  r.reason_code = c.code;
  r.detail = c.detail;
  return r;
}

// ---------------------------------------------------------------------------
// Sampler
// ---------------------------------------------------------------------------

Sampler::Sampler(SamplingConfig cfg) : cfg_(std::move(cfg)) {}

Candidate Sampler::candidate(int32_t index) const
{
  Candidate c;
  c.index = index;
  const uint64_t seed = static_cast<uint64_t>(cfg_.seed_raw);
  c.x = sampleUniform(seed, index, 1u, cfg_.x_min, cfg_.x_max);
  c.y = sampleUniform(seed, index, 2u, cfg_.y_min, cfg_.y_max);
  const double yaw_deg = sampleUniform(seed, index, 3u, cfg_.yaw_deg_min, cfg_.yaw_deg_max);
  c.yaw_rad = deg2rad(yaw_deg);
  c.z = rodCenterZ(cfg_);
  // roll/pitch are fixed configuration offsets (0 by default): the rod is
  // assumed to lie flat, so only yaw is sampled.
  const double cr = std::cos(deg2rad(cfg_.roll_deg) * 0.5);
  const double sr = std::sin(deg2rad(cfg_.roll_deg) * 0.5);
  const double cp = std::cos(deg2rad(cfg_.pitch_deg) * 0.5);
  const double sp = std::sin(deg2rad(cfg_.pitch_deg) * 0.5);
  const double cy = std::cos(c.yaw_rad * 0.5);
  const double sy = std::sin(c.yaw_rad * 0.5);
  // Rz(yaw) * Ry(pitch) * Rx(roll)
  c.quat.w = cr * cp * cy + sr * sp * sy;
  c.quat.x = sr * cp * cy - cr * sp * sy;
  c.quat.y = cr * sp * cy + sr * cp * sy;
  c.quat.z = cr * cp * sy - sr * sp * cy;
  normalize(c.quat);
  return c;
}

std::vector<Candidate> Sampler::firstN(int32_t count) const
{
  std::vector<Candidate> out;
  const int32_t n = std::min(count, cfg_.max_total_candidates);
  if (n <= 0) {
    return out;
  }
  out.reserve(static_cast<std::size_t>(n));
  for (int32_t i = 0; i < n; ++i) {
    out.push_back(candidate(i));
  }
  return out;
}

// ---------------------------------------------------------------------------
// Footprint
// ---------------------------------------------------------------------------

bool makeExplicitCandidate(const SamplingConfig& cfg,
                           int32_t index,
                           double x,
                           double y,
                           double yaw_rad,
                           Candidate& out)
{
  if (!isFiniteDouble(x) || !isFiniteDouble(y) || !isFiniteDouble(yaw_rad)) {
    return false;
  }
  Candidate c;
  c.index = index;
  c.x = x;
  c.y = y;
  c.yaw_rad = yaw_rad;
  c.z = rodCenterZ(cfg);
  c.quat = quatFromYaw(yaw_rad);
  if (!isFinite(c.quat)) {
    return false;
  }
  out = c;
  return true;
}

Footprint rodFootprint(const SamplingConfig& cfg, const Candidate& c)
{
  Footprint fp;
  const double cs = std::cos(c.yaw_rad);
  const double sn = std::sin(c.yaw_rad);
  const double w = cfg.rod_half_width;
  const double l = cfg.rod_half_length;
  // Local half-extent corners (sx * w, sy * l) mapped by Rz(yaw).
  const double sign_x[4] = {1.0, -1.0, -1.0, 1.0};
  const double sign_y[4] = {1.0, 1.0, -1.0, -1.0};
  fp.min_x = std::numeric_limits<double>::infinity();
  fp.max_x = -std::numeric_limits<double>::infinity();
  fp.min_y = std::numeric_limits<double>::infinity();
  fp.max_y = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < 4; ++i) {
    const double lx = sign_x[i] * w;
    const double ly = sign_y[i] * l;
    Vec3 p;
    p.x = c.x + cs * lx - sn * ly;
    p.y = c.y + sn * lx + cs * ly;
    p.z = cfg.support_surface_z;
    fp.corners[static_cast<std::size_t>(i)] = p;
    fp.min_x = std::min(fp.min_x, p.x);
    fp.max_x = std::max(fp.max_x, p.x);
    fp.min_y = std::min(fp.min_y, p.y);
    fp.max_y = std::max(fp.max_y, p.y);
  }
  return fp;
}

// ---------------------------------------------------------------------------
// Stability
// ---------------------------------------------------------------------------

std::string stabilityReasonCode(StabilityOutcome outcome)
{
  switch (outcome) {
    case StabilityOutcome::STABLE: return "E_OK";
    case StabilityOutcome::MODEL_RELOADED: return "E_MODEL_RELOADED";
    case StabilityOutcome::INSUFFICIENT_SAMPLES: return "E_STABILITY_INSUFFICIENT_SAMPLES";
    case StabilityOutcome::CLOCK_INVALID: return "E_CLOCK_INVALID";
    case StabilityOutcome::CLOCK_NOT_ADVANCING: return "E_CLOCK_NOT_ADVANCING";
    case StabilityOutcome::CLOCK_JUMP: return "E_CLOCK_JUMP";
    case StabilityOutcome::SIM_WINDOW_TOO_SHORT: return "E_SIM_WINDOW_TOO_SHORT";
    case StabilityOutcome::CLOCK_RATIO_TOO_LOW: return "E_CLOCK_RATIO_TOO_LOW";
    case StabilityOutcome::INVALID_TWIST: return "E_TWIST_INVALID";
    case StabilityOutcome::TWIST_EXCEEDED: return "E_TWIST_EXCEEDED";
    case StabilityOutcome::DRIFT_EXCEEDED: return "E_DRIFT_EXCEEDED";
    case StabilityOutcome::NOT_RESTING: return "E_NOT_RESTING";
    case StabilityOutcome::PAUSED: return "E_SIM_PAUSED";
  }
  return "E_UNKNOWN";
}

StabilityResult evaluateStability(const std::vector<PoseSample>& window,
                                  const StabilityCriteria& crit,
                                  double rod_rest_z,
                                  bool sim_paused,
                                  bool model_reloaded)
{
  StabilityResult r;

  // A reload or a pause can only ever prevent success.
  if (model_reloaded) {
    r.outcome = StabilityOutcome::MODEL_RELOADED;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "simulation model was reloaded during the settling window";
    return r;
  }
  if (sim_paused) {
    r.outcome = StabilityOutcome::PAUSED;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "simulation reports paused; repeated reads are not progress";
    return r;
  }
  if (window.size() < 3) {
    r.outcome = StabilityOutcome::INSUFFICIENT_SAMPLES;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "fewer than 3 samples in window";
    return r;
  }

  for (const auto& s : window) {
    if (!s.ros_time_valid) {
      r.outcome = StabilityOutcome::CLOCK_INVALID;
      r.reason_code = stabilityReasonCode(r.outcome);
      r.detail = "simulation clock is not valid";
      return r;
    }
  }
  for (std::size_t i = 1; i < window.size(); ++i) {
    if (window[i].ros_time_sec < window[i - 1].ros_time_sec) {
      r.outcome = StabilityOutcome::CLOCK_JUMP;
      r.reason_code = stabilityReasonCode(r.outcome);
      r.detail = "simulation clock moved backwards";
      return r;
    }
    if (window[i].ros_time_sec == window[i - 1].ros_time_sec) {
      r.outcome = StabilityOutcome::CLOCK_NOT_ADVANCING;
      r.reason_code = stabilityReasonCode(r.outcome);
      r.detail = "simulation clock did not advance between polls";
      return r;
    }
  }

  r.sim_window_sec = window.back().ros_time_sec - window.front().ros_time_sec;
  r.steady_window_sec = window.back().steady_time_sec - window.front().steady_time_sec;

  if (r.sim_window_sec < crit.min_sim_window_sec) {
    r.outcome = StabilityOutcome::SIM_WINDOW_TOO_SHORT;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "simulation window shorter than required";
    return r;
  }
  if (!(r.steady_window_sec > 0.0)) {
    r.outcome = StabilityOutcome::CLOCK_NOT_ADVANCING;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "steady-clock window is zero";
    return r;
  }
  r.clock_advance_ratio = r.sim_window_sec / r.steady_window_sec;
  if (r.clock_advance_ratio < crit.min_clock_advance_ratio) {
    r.outcome = StabilityOutcome::CLOCK_RATIO_TOO_LOW;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "simulation advanced far slower than wall clock";
    return r;
  }

  for (const auto& s : window) {
    if (!s.twist_valid) {
      r.outcome = StabilityOutcome::INVALID_TWIST;
      r.reason_code = stabilityReasonCode(r.outcome);
      r.detail = "service returned a non-finite twist";
      return r;
    }
    r.max_twist_lin = std::max(r.max_twist_lin, s.twist_lin_norm);
    r.max_twist_ang = std::max(r.max_twist_ang, s.twist_ang_norm);
  }
  if (r.max_twist_lin > crit.max_twist_linear_tol || r.max_twist_ang > crit.max_twist_angular_tol) {
    r.outcome = StabilityOutcome::TWIST_EXCEEDED;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "twist norm above tolerance";
    return r;
  }

  // Whole-window drift: maximum over ALL sample pairs, so a slow monotonic
  // drift is rejected even though adjacent differences stay small.
  for (std::size_t i = 0; i < window.size(); ++i) {
    for (std::size_t j = i + 1; j < window.size(); ++j) {
      const Vec3 d {window[j].pos.x - window[i].pos.x,
                    window[j].pos.y - window[i].pos.y,
                    window[j].pos.z - window[i].pos.z};
      r.max_window_pos_delta =
          std::max(r.max_window_pos_delta, std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z));
      const double angle = rotationDistanceRad(window[i].quat, window[j].quat);
      if (std::isfinite(angle)) {
        r.max_window_angle_delta_deg =
            std::max(r.max_window_angle_delta_deg, rad2deg(angle));
      } else {
        r.outcome = StabilityOutcome::DRIFT_EXCEEDED;
        r.reason_code = stabilityReasonCode(r.outcome);
        r.detail = "non-finite orientation in window";
        return r;
      }
    }
  }
  if (r.max_window_pos_delta > crit.max_window_pos_tol ||
      r.max_window_angle_delta_deg > crit.max_window_angle_tol_deg) {
    r.outcome = StabilityOutcome::DRIFT_EXCEEDED;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "position or orientation drift over the whole window";
    return r;
  }

  // Resting geometry. This is a static-geometry criterion only: it does NOT
  // prove contact with the support surface.
  const auto& last = window.back();
  const double z_err = std::fabs(last.pos.z - rod_rest_z);
  const double tilt = std::max(std::fabs(rad2deg(rollFromQuat(last.quat))),
                              std::fabs(rad2deg(pitchFromQuat(last.quat))));
  if (z_err > crit.resting_z_tol) {
    r.outcome = StabilityOutcome::NOT_RESTING;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "rod centre height is not at the theoretical resting height";
    return r;
  }
  if (tilt > crit.resting_tilt_tol_deg) {
    r.outcome = StabilityOutcome::NOT_RESTING;
    r.reason_code = stabilityReasonCode(r.outcome);
    r.detail = "rod is not lying flat (roll/pitch above tolerance)";
    return r;
  }

  r.outcome = StabilityOutcome::STABLE;
  r.reason_code = stabilityReasonCode(r.outcome);
  r.detail = "stable";
  return r;
}

// ---------------------------------------------------------------------------
// Write outcome helpers
// ---------------------------------------------------------------------------

std::string toString(WriteOutcome outcome)
{
  switch (outcome) {
    case WriteOutcome::SUCCESS: return "SUCCESS";
    case WriteOutcome::REJECTED: return "REJECTED";
    case WriteOutcome::TIMEOUT: return "TIMEOUT";
    case WriteOutcome::UNAVAILABLE: return "UNAVAILABLE";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// Single-candidate orchestration
// ---------------------------------------------------------------------------

RunResult applyCandidateAndSettle(const SamplingConfig& cfg,
                                  const StabilityCriteria& crit,
                                  const ApplyPolicy& policy,
                                  int32_t candidate_index,
                                  BodyStateTransport& transport,
                                  Clock& clock)
{
  RunResult r;

  const ValidationResult valid = validateSamplingConfig(cfg);
  if (!valid.ok) {
    r.scene_state = "NOT_RUN";
    r.outcome_code = valid.reason_code;
    r.detail = valid.detail;
    return r;
  }
  if (candidate_index < 0 || candidate_index >= cfg.max_total_candidates) {
    r.scene_state = "NOT_RUN";
    r.outcome_code = "E_BAD_CANDIDATE_INDEX";
    r.detail = "candidate index outside [0, max_total_candidates)";
    return r;
  }

  const Sampler sampler(cfg);
  return applyCandidateAndSettle(cfg, crit, policy, sampler.candidate(candidate_index),
                                 transport, clock);
}

RunResult applyCandidateAndSettle(const SamplingConfig& cfg,
                                  const StabilityCriteria& crit,
                                  const ApplyPolicy& policy,
                                  const Candidate& candidate,
                                  BodyStateTransport& transport,
                                  Clock& clock)
{
  RunResult r;

  const ValidationResult valid = validateSamplingConfig(cfg);
  if (!valid.ok) {
    r.scene_state = "NOT_RUN";
    r.outcome_code = valid.reason_code;
    r.detail = valid.detail;
    return r;
  }
  const ValidationResult policy_ok = validateApplyParameters(policy, crit);
  if (!policy_ok.ok) {
    r.scene_state = "NOT_RUN";
    r.outcome_code = policy_ok.reason_code;
    r.detail = policy_ok.detail;
    return r;
  }
  if (!isFiniteDouble(candidate.x) || !isFiniteDouble(candidate.y) ||
      !isFiniteDouble(candidate.z) || !isUnitQuat(candidate.quat)) {
    r.scene_state = "NOT_RUN";
    r.outcome_code = "E_BAD_CANDIDATE";
    r.detail = "candidate has non-finite values or a non-unit quaternion";
    return r;
  }

  r.candidate = candidate;
  r.candidate_generated = true;

  const double deadline = clock.steadyNowSec() + policy.stability_timeout_sec;
  const double period = 1.0 / std::max(1.0e-3, policy.poll_rate_hz);
  const double rod_rest_z = cfg.support_surface_z + cfg.rod_half_height;
  const int32_t max_samples = std::max<int32_t>(3, policy.max_window_samples);

  auto aborted = [&policy]() {
    return policy.abort_flag != nullptr && policy.abort_flag->load();
  };
  auto remaining = [&clock, deadline]() {
    return std::max(0.0, deadline - clock.steadyNowSec());
  };
  // A blocking call may never be allowed to outlast the remaining budget.
  auto call_budget = [&policy, &remaining]() {
    return std::min(policy.service_call_timeout_sec, remaining());
  };

  // Cancellation and the deadline are checked BEFORE anything that can act, so
  // a run cancelled up front performs zero writes.
  if (aborted()) {
    r.scene_state = "INTERRUPTED";
    r.outcome_code = "E_INTERRUPTED";
    r.detail = "cancelled before any service interaction; no write was attempted";
    return r;
  }
  if (!(remaining() > 0.0)) {
    r.scene_state = "NOT_RUN";
    r.outcome_code = "E_BUDGET_EXHAUSTED";
    r.detail = "no settling budget available before the write";
    return r;
  }

  // --- Preconditions: model ready, not paused, baseline state ---------------
  r.sim_info_before = transport.getSimInfo(call_budget());
  if (!r.sim_info_before.ok) {
    r.scene_state = "SCENE_APPLY_FAILED";
    r.outcome_code = "E_SIM_INFO_UNAVAILABLE";
    r.detail = r.sim_info_before.status_message;
    return r;
  }
  if (!r.sim_info_before.model_valid) {
    r.scene_state = "SCENE_APPLY_FAILED";
    r.outcome_code = "E_MODEL_INVALID";
    r.detail = "simulation reports model_valid == false";
    return r;
  }
  if (r.sim_info_before.paused) {
    r.scene_state = "SCENE_APPLY_FAILED";
    r.outcome_code = "E_SIM_PAUSED_BEFORE_WRITE";
    r.detail = "simulation is paused; refusing to write";
    return r;
  }

  PoseSample baseline_timing;
  r.baseline_state = transport.getState(call_budget(), baseline_timing);
  if (!r.baseline_state.ok) {
    r.scene_state = "SCENE_APPLY_FAILED";
    r.outcome_code = "E_BASELINE_READ_FAILED";
    r.detail = r.baseline_state.status_message;
    return r;
  }
  // Re-check after the blocking read: it may have consumed the whole budget or
  // a cancellation may have arrived meanwhile.
  if (aborted()) {
    r.scene_state = "INTERRUPTED";
    r.outcome_code = "E_INTERRUPTED";
    r.detail = "cancelled after the baseline read; no write was attempted";
    return r;
  }
  if (!(remaining() > 0.0)) {
    r.scene_state = "NOT_RUN";
    r.outcome_code = "E_BUDGET_EXHAUSTED";
    r.detail = "budget exhausted by the baseline read; no write was attempted";
    return r;
  }

  // --- Exactly one write ---------------------------------------------------
  r.write_attempted = true;
  const double write_start_steady = clock.steadyNowSec();
  r.write_outcome =
      transport.setPose(r.candidate, call_budget(), r.write_timing, r.write_status_message);
  const double write_end_steady = clock.steadyNowSec();
  if (r.write_timing.response_steady_sec <= 0.0) {
    r.write_timing.response_steady_sec = write_end_steady;
  }

  if (r.write_outcome != WriteOutcome::SUCCESS) {
    r.scene_state = "SCENE_APPLY_FAILED";
    switch (r.write_outcome) {
      case WriteOutcome::UNAVAILABLE:
        r.outcome_code = "E_WRITE_UNAVAILABLE";
        break;
      case WriteOutcome::REJECTED:
        r.outcome_code = "E_WRITE_REJECTED";
        break;
      case WriteOutcome::TIMEOUT:
        r.outcome_code = "E_WRITE_TIMEOUT";
        break;
      case WriteOutcome::SUCCESS:
        break;
    }
    // Deliberately no retry and no fallback to another candidate: a service
    // timeout does not cancel server-side work, and a failed response does not
    // imply the write had no effect. The run stops with an uncertain result.
    r.detail = "write did not succeed (" + std::string(toString(r.write_outcome)) +
               "); stopping this run without retrying, server side may still act";
    r.steady_span_sec = write_end_steady - write_start_steady;
    return r;
  }

  // --- Bounded polling until stable or steady-clock timeout ----------------
  // A failure to observe health is itself a failure: the previous
  // implementation kept the last known state and could still report success.
  bool sim_paused = r.sim_info_before.paused;
  bool model_reloaded = false;
  bool health_failed = false;
  std::string health_failure_code;
  std::string health_failure_detail;
  StabilityResult last;
  last.outcome = StabilityOutcome::INSUFFICIENT_SAMPLES;
  last.reason_code = stabilityReasonCode(last.outcome);
  last.detail = "no samples yet";

  int32_t polls = 0;
  bool finished = false;
  while (!finished) {
    // Cancellation and the deadline take priority over any success verdict.
    if (aborted()) {
      r.stability = last;
      r.scene_state = "INTERRUPTED";
      r.outcome_code = "E_INTERRUPTED";
      r.detail = "cancelled during settling; the applied pose stands";
      break;
    }
    if (!(clock.steadyNowSec() < deadline)) {
      r.stability = last;
      r.scene_state = "SCENE_SETTLE_FAILED";
      r.outcome_code = "E_SCENE_SETTLE_TIMEOUT";
      r.detail = "budget exhausted before a sample was taken; last reason: " + last.reason_code;
      break;
    }

    PoseSample s;
    s.request_steady_sec = clock.steadyNowSec();
    s.request_ros_valid = clock.rosTimeValid();
    s.request_ros_sec = s.request_ros_valid ? clock.rosNowSec() : 0.0;

    const BodyState st = transport.getState(call_budget(), s);
    if (st.ok) {
      s.pos = st.pos;
      s.quat = st.quat;
      s.twist_lin_norm = std::sqrt(st.twist_lin.x * st.twist_lin.x +
                                   st.twist_lin.y * st.twist_lin.y +
                                   st.twist_lin.z * st.twist_lin.z);
      s.twist_ang_norm = std::sqrt(st.twist_ang.x * st.twist_ang.x +
                                   st.twist_ang.y * st.twist_ang.y +
                                   st.twist_ang.z * st.twist_ang.z);
      s.twist_valid = isFinite(st.pos) && isFinite(st.quat) && isFinite(st.twist_lin) &&
                      isFinite(st.twist_ang);
    } else {
      s.twist_valid = false;
      s.pos = Vec3 {};
      s.quat = Quat {};
      // The response timing is unknown for a failed read; keep the request time
      // so the record still shows when the attempt happened.
      if (s.steady_time_sec <= 0.0) {
        s.steady_time_sec = s.request_steady_sec;
        s.ros_time_sec = s.request_ros_sec;
        s.ros_time_valid = false;
      }
    }
    r.samples.push_back(s);
    if (r.history.size() < 20000) {
      r.history.push_back(s);
    }

    // Slide the window by SIMULATED time, keeping roughly one to two times the
    // required window length. Trimming by raw sample count instead would keep a
    // fixed amount of history regardless of the configured window length: with
    // 200 samples at 50 Hz the landing transient stayed in the window for ~4 s
    // even though only 0.5 s of quiet history is required.
    const double keep_span = std::max(crit.min_sim_window_sec * 2.0,
                                      crit.min_sim_window_sec + 0.01);
    while (r.samples.size() > 3) {
      const double span = r.samples.back().ros_time_sec - r.samples.front().ros_time_sec;
      if (!(span > keep_span)) {
        break;
      }
      r.samples.erase(r.samples.begin());
    }
    // Hard safety cap, independent of the above.
    while (static_cast<int32_t>(r.samples.size()) > max_samples) {
      r.samples.erase(r.samples.begin());
    }
    ++polls;

    // Cancellation / deadline again: the read above may have blocked past the
    // budget, and a verdict must not be based on data taken outside it.
    if (aborted()) {
      r.stability = last;
      r.scene_state = "INTERRUPTED";
      r.outcome_code = "E_INTERRUPTED";
      r.detail = "cancelled after a read; the applied pose stands";
      break;
    }
    if (clock.steadyNowSec() > deadline) {
      r.stability = last;
      r.scene_state = "SCENE_SETTLE_FAILED";
      r.outcome_code = "E_SCENE_SETTLE_TIMEOUT";
      r.detail = "read completed past the settling budget; success is not reported";
      break;
    }

    if ((polls % policy.sim_info_check_every) == 0) {
      const SimInfo si = transport.getSimInfo(call_budget());
      if (!si.ok) {
        health_failed = true;
        health_failure_code = "E_HEALTH_QUERY_FAILED";
        health_failure_detail = "simulation health query failed during settling";
      } else {
        r.sim_info_after = si;
        sim_paused = si.paused;
        if (!si.model_valid) {
          health_failed = true;
          health_failure_code = "E_HEALTH_MODEL_INVALID";
          health_failure_detail = "simulation reports model_valid == false during settling";
        } else if (si.model_path != r.sim_info_before.model_path) {
          health_failed = true;
          health_failure_code = "E_HEALTH_MODEL_PATH_CHANGED";
          health_failure_detail = "simulation model path changed: " + si.model_path;
        } else if (si.load_count != r.sim_info_before.load_count) {
          model_reloaded = true;
        }
      }
    }
    if (health_failed) {
      r.stability = last;
      r.scene_state = "SCENE_SETTLE_FAILED";
      r.outcome_code = health_failure_code;
      r.detail = health_failure_detail;
      break;
    }

    last = evaluateStability(r.samples, crit, rod_rest_z, sim_paused, model_reloaded);

    // Record the reason only when it changes, so the trace stays short.
    if (r.settle_trace.empty() || r.settle_trace.back().reason_code != last.reason_code) {
      if (r.settle_trace.size() < 64) {
        r.settle_trace.push_back({polls, last.reason_code});
      }
    }

    if (last.outcome == StabilityOutcome::STABLE) {
      // Success requires one final, valid health confirmation that agrees with
      // the contract taken before the write. A failed query, a reload, a model
      // change or a pause here means no success is reported.
      r.sim_info_final = transport.getSimInfo(call_budget());

      // That query is itself a blocking call, so cancellation and the deadline
      // are re-checked before a success verdict is allowed. Both keep the
      // record of what was already written and never trigger a retry.
      if (aborted()) {
        r.stability = last;
        r.scene_state = "INTERRUPTED";
        r.outcome_code = "E_INTERRUPTED";
        r.detail = "cancelled while confirming final health; the applied pose stands";
        finished = true;
        break;
      }
      if (clock.steadyNowSec() > deadline) {
        r.stability = last;
        r.scene_state = "SCENE_SETTLE_FAILED";
        r.outcome_code = "E_SCENE_SETTLE_TIMEOUT";
        r.detail =
            "final health query completed past the settling budget; success is not reported";
        finished = true;
        break;
      }

      if (!r.sim_info_final.ok) {
        r.stability = last;
        r.scene_state = "SCENE_SETTLE_FAILED";
        r.outcome_code = "E_HEALTH_UNCONFIRMED";
        r.health_detail = "final health query failed: " + r.sim_info_final.status_message;
        r.detail = r.health_detail;
        finished = true;
        break;
      }
      if (!r.sim_info_final.model_valid) {
        r.stability = last;
        r.scene_state = "SCENE_SETTLE_FAILED";
        r.outcome_code = "E_HEALTH_MODEL_INVALID";
        r.health_detail = "final health query reports model_valid == false";
        r.detail = r.health_detail;
        finished = true;
        break;
      }
      if (r.sim_info_final.paused) {
        r.stability = last;
        r.scene_state = "SCENE_SETTLE_FAILED";
        r.outcome_code = "E_SIM_PAUSED";
        r.health_detail = "final health query reports paused";
        r.detail = r.health_detail;
        finished = true;
        break;
      }
      if (r.sim_info_final.model_path != r.sim_info_before.model_path) {
        r.stability = last;
        r.scene_state = "SCENE_SETTLE_FAILED";
        r.outcome_code = "E_HEALTH_MODEL_PATH_CHANGED";
        r.health_detail = "model path changed from '" + r.sim_info_before.model_path + "' to '" +
                          r.sim_info_final.model_path + "'";
        r.detail = r.health_detail;
        finished = true;
        break;
      }
      if (r.sim_info_final.load_count != r.sim_info_before.load_count) {
        r.stability = last;
        r.scene_state = "SCENE_SETTLE_FAILED";
        r.outcome_code = "E_MODEL_RELOADED";
        r.health_detail = "load_count changed from " +
                          std::to_string(r.sim_info_before.load_count) + " to " +
                          std::to_string(r.sim_info_final.load_count);
        r.detail = r.health_detail;
        finished = true;
        break;
      }

      r.health_confirmed = true;
      r.stability = last;
      r.readback = st;
      r.scene_state = "SCENE_READY_UNVALIDATED";
      r.outcome_code = "SCENE_READY_UNVALIDATED";
      r.detail = "pose applied and settled; sample validity is NOT established here";
      finished = true;
      break;
    }

    // Hard stops: these can never become success by waiting longer.
    if (last.outcome == StabilityOutcome::MODEL_RELOADED ||
        last.outcome == StabilityOutcome::CLOCK_JUMP ||
        last.outcome == StabilityOutcome::PAUSED) {
      r.stability = last;
      r.scene_state = "SCENE_SETTLE_FAILED";
      r.outcome_code = last.reason_code;
      r.detail = last.detail;
      finished = true;
      break;
    }

    // Absolute schedule avoids drift accumulation.
    const double next_tick =
        (deadline - policy.stability_timeout_sec) + period * static_cast<double>(polls);
    clock.sleepUntil(std::min(next_tick, deadline));
  }

  if (r.samples.size() >= 2) {
    r.steady_span_sec = r.samples.back().steady_time_sec - r.samples.front().steady_time_sec;
    if (r.samples.front().ros_time_valid && r.samples.back().ros_time_valid) {
      r.ros_span_sec = r.samples.back().ros_time_sec - r.samples.front().ros_time_sec;
      r.ros_span_valid = true;
    }
  }
  if (r.stability.reason_code.empty()) {
    r.stability = last;
  }
  return r;
}

}  // namespace a1
