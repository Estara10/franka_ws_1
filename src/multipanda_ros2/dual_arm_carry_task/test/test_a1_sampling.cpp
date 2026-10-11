// A1 P1a offline tests: deterministic sampling, config validation, geometry.
// These drive the pure library directly; no ROS, no simulation.

#include "dual_arm_carry_task/a1_sampling.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

namespace {

a1::SamplingConfig baseConfig()
{
  a1::SamplingConfig cfg;
  cfg.algorithm_version = "a1-uniform-splitmix64-v1";
  cfg.sampling_range_version = "test-v1";
  cfg.range_is_reachability_verified = false;
  cfg.seed_raw = 20261011;
  cfg.x_min = 0.45;
  cfg.x_max = 0.55;
  cfg.y_min = -0.05;
  cfg.y_max = 0.05;
  cfg.yaw_deg_min = -15.0;
  cfg.yaw_deg_max = 15.0;
  cfg.roll_deg = 0.0;
  cfg.pitch_deg = 0.0;
  cfg.rod_half_length = 0.20;
  cfg.rod_half_width = 0.02;
  cfg.rod_half_height = 0.02;
  cfg.support_surface_z = 0.0;
  cfg.apply_clearance_m = 0.002;
  cfg.max_total_candidates = 200;
  return cfg;
}

constexpr double kEps = 1e-9;

}  // namespace

// ---------------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------------

TEST(A1Sampling, SameSeedAndIndexProduceIdenticalCandidates)
{
  const a1::Sampler a(baseConfig());
  const a1::Sampler b(baseConfig());
  for (int32_t i = 0; i < 32; ++i) {
    const a1::Candidate ca = a.candidate(i);
    const a1::Candidate cb = b.candidate(i);
    EXPECT_EQ(ca.index, i);
    EXPECT_DOUBLE_EQ(ca.x, cb.x) << "index " << i;
    EXPECT_DOUBLE_EQ(ca.y, cb.y) << "index " << i;
    EXPECT_DOUBLE_EQ(ca.z, cb.z) << "index " << i;
    EXPECT_DOUBLE_EQ(ca.yaw_rad, cb.yaw_rad) << "index " << i;
    EXPECT_DOUBLE_EQ(ca.quat.w, cb.quat.w) << "index " << i;
    EXPECT_DOUBLE_EQ(ca.quat.z, cb.quat.z) << "index " << i;
  }
}

TEST(A1Sampling, SameSamplerIsStableAcrossRepeatedCalls)
{
  const a1::Sampler s(baseConfig());
  for (int32_t i = 0; i < 8; ++i) {
    const a1::Candidate first = s.candidate(i);
    const a1::Candidate again = s.candidate(i);
    EXPECT_DOUBLE_EQ(first.x, again.x);
    EXPECT_DOUBLE_EQ(first.y, again.y);
    EXPECT_DOUBLE_EQ(first.yaw_rad, again.yaw_rad);
  }
}

TEST(A1Sampling, FirstNReplaysCandidateSequence)
{
  const a1::Sampler s(baseConfig());
  const std::vector<a1::Candidate> seq = s.firstN(12);
  ASSERT_EQ(seq.size(), 12u);
  for (int32_t i = 0; i < 12; ++i) {
    const a1::Candidate c = s.candidate(i);
    EXPECT_DOUBLE_EQ(seq[static_cast<std::size_t>(i)].x, c.x);
    EXPECT_DOUBLE_EQ(seq[static_cast<std::size_t>(i)].y, c.y);
    EXPECT_DOUBLE_EQ(seq[static_cast<std::size_t>(i)].yaw_rad, c.yaw_rad);
  }
}

TEST(A1Sampling, DifferentSeedsProduceDifferentCandidates)
{
  a1::SamplingConfig other = baseConfig();
  other.seed_raw = baseConfig().seed_raw + 1;
  const a1::Sampler a(baseConfig());
  const a1::Sampler b(other);
  int differences = 0;
  for (int32_t i = 0; i < 32; ++i) {
    if (std::fabs(a.candidate(i).x - b.candidate(i).x) > kEps ||
        std::fabs(a.candidate(i).y - b.candidate(i).y) > kEps ||
        std::fabs(a.candidate(i).yaw_rad - b.candidate(i).yaw_rad) > kEps) {
      ++differences;
    }
  }
  EXPECT_GT(differences, 0);
}

TEST(A1Sampling, CandidatesStayInsideTheConfiguredRangesAndCoverThem)
{
  const a1::SamplingConfig cfg = baseConfig();
  const a1::Sampler s(cfg);
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double min_yaw = std::numeric_limits<double>::infinity();
  double max_yaw = -std::numeric_limits<double>::infinity();
  const int32_t n = 400;
  for (int32_t i = 0; i < n; ++i) {
    const a1::Candidate c = s.candidate(i);
    ASSERT_GE(c.x, cfg.x_min);
    ASSERT_LE(c.x, cfg.x_max);
    ASSERT_GE(c.y, cfg.y_min);
    ASSERT_LE(c.y, cfg.y_max);
    const double yaw_deg = c.yaw_rad * 180.0 / M_PI;
    ASSERT_GE(yaw_deg, cfg.yaw_deg_min - 1e-9);
    ASSERT_LE(yaw_deg, cfg.yaw_deg_max + 1e-9);
    min_x = std::min(min_x, c.x);
    max_x = std::max(max_x, c.x);
    min_y = std::min(min_y, c.y);
    max_y = std::max(max_y, c.y);
    min_yaw = std::min(min_yaw, yaw_deg);
    max_yaw = std::max(max_yaw, yaw_deg);
  }
  // The sample must actually span the range rather than clustering.
  EXPECT_LT(min_x, cfg.x_min + 0.05 * (cfg.x_max - cfg.x_min));
  EXPECT_GT(max_x, cfg.x_max - 0.05 * (cfg.x_max - cfg.x_min));
  EXPECT_LT(min_y, cfg.y_min + 0.05 * (cfg.y_max - cfg.y_min));
  EXPECT_GT(max_y, cfg.y_max - 0.05 * (cfg.y_max - cfg.y_min));
  EXPECT_LT(min_yaw, cfg.yaw_deg_min + 1.0);
  EXPECT_GT(max_yaw, cfg.yaw_deg_max - 1.0);
}

TEST(A1Sampling, ZIsDerivedFromSupportSurfaceHalfHeightAndClearance)
{
  const a1::SamplingConfig cfg = baseConfig();
  EXPECT_NEAR(a1::rodCenterZ(cfg), 0.022, 1e-12);
  const a1::Sampler s(cfg);
  EXPECT_NEAR(s.candidate(0).z, 0.022, 1e-12);

  a1::SamplingConfig raised = cfg;
  raised.support_surface_z = 0.10;
  raised.apply_clearance_m = 0.005;
  EXPECT_NEAR(a1::rodCenterZ(raised), 0.125, 1e-12);
}

TEST(A1Sampling, CandidateQuaternionMatchesYawRotation)
{
  const a1::Sampler s(baseConfig());
  for (int32_t i = 0; i < 16; ++i) {
    const a1::Candidate c = s.candidate(i);
    EXPECT_NEAR(std::fabs(c.quat.w), std::cos(c.yaw_rad / 2.0), 1e-12);
    EXPECT_NEAR(std::fabs(c.quat.z), std::fabs(std::sin(c.yaw_rad / 2.0)), 1e-12);
    EXPECT_NEAR(c.quat.x, 0.0, 1e-12);
    EXPECT_NEAR(c.quat.y, 0.0, 1e-12);
    const double n = std::sqrt(c.quat.x * c.quat.x + c.quat.y * c.quat.y +
                               c.quat.z * c.quat.z + c.quat.w * c.quat.w);
    EXPECT_NEAR(n, 1.0, 1e-12);
    EXPECT_NEAR(a1::yawFromQuat(c.quat), c.yaw_rad, 1e-12);
  }
}

// ---------------------------------------------------------------------------
// Configuration rejection
// ---------------------------------------------------------------------------

TEST(A1Config, ValidBaseConfigIsAccepted)
{
  const a1::ValidationResult r = a1::validateSamplingConfig(baseConfig());
  EXPECT_TRUE(r.ok) << r.reason_code << ": " << r.detail;
  EXPECT_TRUE(r.reason_code.empty());
}

TEST(A1Config, RejectsNonFiniteValues)
{
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.x_max = std::numeric_limits<double>::quiet_NaN();
    const a1::ValidationResult r = a1::validateSamplingConfig(cfg);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_CONFIG_NOT_FINITE");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.y_min = std::numeric_limits<double>::infinity();
    const a1::ValidationResult r = a1::validateSamplingConfig(cfg);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_CONFIG_NOT_FINITE");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.apply_clearance_m = -std::numeric_limits<double>::infinity();
    const a1::ValidationResult r = a1::validateSamplingConfig(cfg);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_CONFIG_NOT_FINITE");
  }
}

TEST(A1Config, RejectsInvertedOrEmptyRanges)
{
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.x_min = cfg.x_max;
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_BAD_RANGE");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.y_min = 1.0;
    cfg.y_max = -1.0;
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_BAD_RANGE");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.yaw_deg_min = 10.0;
    cfg.yaw_deg_max = 10.0;
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_BAD_RANGE");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.yaw_deg_min = -200.0;
    cfg.yaw_deg_max = 10.0;
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_BAD_RANGE");
  }
}

TEST(A1Config, RejectsNegativeSeed)
{
  a1::SamplingConfig cfg = baseConfig();
  cfg.seed_raw = -1;
  const a1::ValidationResult r = a1::validateSamplingConfig(cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_CONFIG_BAD_SEED");
}

TEST(A1Config, RejectsBadCandidateCount)
{
  a1::SamplingConfig cfg = baseConfig();
  cfg.max_total_candidates = 0;
  const a1::ValidationResult r = a1::validateSamplingConfig(cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_CONFIG_BAD_CANDIDATE_COUNT");
}

TEST(A1Config, RejectsBadGeometryAndClearanceAndVersion)
{
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.rod_half_length = 0.0;
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_BAD_GEOMETRY");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.rod_half_width = -0.02;
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_BAD_GEOMETRY");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.apply_clearance_m = -0.001;
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_BAD_CLEARANCE");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.algorithm_version.clear();
    EXPECT_EQ(a1::validateSamplingConfig(cfg).reason_code, "E_CONFIG_EMPTY_ALGORITHM_VERSION");
  }
}

// ---------------------------------------------------------------------------
// Config checksum
// ---------------------------------------------------------------------------

TEST(A1Config, ChecksumIsStableAndSensitiveToEverySamplingField)
{
  const a1::SamplingConfig cfg = baseConfig();
  EXPECT_EQ(cfg.checksum(), baseConfig().checksum());
  EXPECT_EQ(cfg.canonicalString(), baseConfig().canonicalString());

  a1::SamplingConfig changed = cfg;
  changed.seed_raw += 1;
  EXPECT_NE(cfg.checksum(), changed.checksum());

  changed = cfg;
  changed.x_max += 1e-9;
  EXPECT_NE(cfg.checksum(), changed.checksum());

  changed = cfg;
  changed.yaw_deg_max += 0.5;
  EXPECT_NE(cfg.checksum(), changed.checksum());

  changed = cfg;
  changed.algorithm_version = "other";
  EXPECT_NE(cfg.checksum(), changed.checksum());

  changed = cfg;
  changed.apply_clearance_m += 1e-6;
  EXPECT_NE(cfg.checksum(), changed.checksum());

  changed = cfg;
  changed.max_total_candidates += 1;
  EXPECT_NE(cfg.checksum(), changed.checksum());
}

// ---------------------------------------------------------------------------
// Explicit (known) test pose
// ---------------------------------------------------------------------------

TEST(A1ExplicitPose, BuildsAnExactKnownPose)
{
  const a1::SamplingConfig cfg = baseConfig();
  a1::Candidate c;
  ASSERT_TRUE(a1::makeExplicitCandidate(cfg, 0, 0.50, 0.0, 0.0, c));
  EXPECT_DOUBLE_EQ(c.x, 0.50);
  EXPECT_DOUBLE_EQ(c.y, 0.0);
  EXPECT_DOUBLE_EQ(c.yaw_rad, 0.0);
  EXPECT_NEAR(c.z, a1::rodCenterZ(cfg), 1e-15);
  EXPECT_NEAR(c.quat.w, 1.0, 1e-15);
  EXPECT_NEAR(c.quat.z, 0.0, 1e-15);
  EXPECT_NEAR(a1::yawFromQuat(c.quat), 0.0, 1e-15);
}

TEST(A1ExplicitPose, AcceptsAnglesOutsideTheSampledRange)
{
  // Verification needs poses the sampler cannot reach, e.g. a large yaw.
  const a1::SamplingConfig cfg = baseConfig();
  a1::Candidate c;
  ASSERT_TRUE(a1::makeExplicitCandidate(cfg, 5, 0.30, 0.20, M_PI / 2.0, c));
  EXPECT_DOUBLE_EQ(c.x, 0.30);
  EXPECT_NEAR(a1::yawFromQuat(c.quat), M_PI / 2.0, 1e-12);
}

TEST(A1ExplicitPose, RejectsNonFiniteValues)
{
  const a1::SamplingConfig cfg = baseConfig();
  a1::Candidate c;
  EXPECT_FALSE(a1::makeExplicitCandidate(cfg, 0, std::numeric_limits<double>::quiet_NaN(), 0.0,
                                        0.0, c));
  EXPECT_FALSE(a1::makeExplicitCandidate(cfg, 0, 0.5, std::numeric_limits<double>::infinity(),
                                        0.0, c));
  EXPECT_FALSE(a1::makeExplicitCandidate(cfg, 0, 0.5, 0.0,
                                        std::numeric_limits<double>::quiet_NaN(), c));
}

// ---------------------------------------------------------------------------
// Rotation distance: q and -q are the same rotation
// ---------------------------------------------------------------------------

TEST(A1Geometry, RotationDistanceTreatsQAndMinusQAsEqual)
{
  const a1::Quat q = a1::quatFromYaw(0.3);
  const a1::Quat neg {-q.x, -q.y, -q.z, -q.w};
  // Tolerance reflects the documented ~3e-8 rad precision floor, which is
  // still 5 orders of magnitude below the settling tolerances.
  EXPECT_NEAR(a1::rotationDistanceRad(q, neg), 0.0, 1e-6);
  EXPECT_NEAR(a1::rotationDistanceRad(q, q), 0.0, 1e-6);
}

TEST(A1Geometry, RotationDistanceMatchesKnownAngles)
{
  const a1::Quat a = a1::quatFromYaw(0.0);
  const a1::Quat b = a1::quatFromYaw(0.5);
  EXPECT_NEAR(a1::rotationDistanceRad(a, b), 0.5, 1e-12);

  const a1::Quat c = a1::quatFromYaw(M_PI);
  EXPECT_NEAR(a1::rotationDistanceRad(a, c), M_PI, 1e-12);

  const a1::Quat d = a1::quatFromYaw(-0.25);
  EXPECT_NEAR(a1::rotationDistanceRad(b, d), 0.75, 1e-12);
}

TEST(A1Geometry, NormalizeRejectsDegenerateQuaternion)
{
  a1::Quat zero {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(a1::normalize(zero));

  a1::Quat q {0.0, 0.0, 0.0, 2.0};
  EXPECT_TRUE(a1::normalize(q));
  EXPECT_NEAR(q.w, 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Footprint
// ---------------------------------------------------------------------------

TEST(A1Footprint, YawZeroAlignsLongAxisWithWorldY)
{
  const a1::SamplingConfig cfg = baseConfig();
  a1::Candidate c;
  c.x = 0.5;
  c.y = 0.0;
  c.yaw_rad = 0.0;
  const a1::Footprint fp = a1::rodFootprint(cfg, c);

  EXPECT_NEAR(fp.min_x, 0.5 - 0.02, 1e-12);
  EXPECT_NEAR(fp.max_x, 0.5 + 0.02, 1e-12);
  EXPECT_NEAR(fp.min_y, -0.20, 1e-12);
  EXPECT_NEAR(fp.max_y, 0.20, 1e-12);
  for (const auto& corner : fp.corners) {
    EXPECT_NEAR(corner.z, cfg.support_surface_z, 1e-12);
  }
}

TEST(A1Footprint, Yaw90RotatesLongAxisOntoWorldX)
{
  const a1::SamplingConfig cfg = baseConfig();
  a1::Candidate c;
  c.x = 0.5;
  c.y = 0.0;
  c.yaw_rad = M_PI / 2.0;
  const a1::Footprint fp = a1::rodFootprint(cfg, c);

  // Long axis (0.20) now spans X, short axis (0.02) spans Y.
  EXPECT_NEAR(fp.min_x, 0.5 - 0.20, 1e-9);
  EXPECT_NEAR(fp.max_x, 0.5 + 0.20, 1e-9);
  EXPECT_NEAR(fp.min_y, -0.02, 1e-9);
  EXPECT_NEAR(fp.max_y, 0.02, 1e-9);
}

TEST(A1Footprint, CornersAreTheRotatedHalfExtents)
{
  const a1::SamplingConfig cfg = baseConfig();
  a1::Candidate c;
  c.x = 0.50;
  c.y = 0.05;
  c.yaw_rad = M_PI / 6.0;  // 30 degrees
  const a1::Footprint fp = a1::rodFootprint(cfg, c);

  const double cs = std::cos(c.yaw_rad);
  const double sn = std::sin(c.yaw_rad);
  const double w = cfg.rod_half_width;
  const double l = cfg.rod_half_length;
  const double sign_x[4] = {1.0, -1.0, -1.0, 1.0};
  const double sign_y[4] = {1.0, 1.0, -1.0, -1.0};
  for (int i = 0; i < 4; ++i) {
    const double lx = sign_x[i] * w;
    const double ly = sign_y[i] * l;
    EXPECT_NEAR(fp.corners[static_cast<std::size_t>(i)].x, c.x + cs * lx - sn * ly, 1e-12);
    EXPECT_NEAR(fp.corners[static_cast<std::size_t>(i)].y, c.y + sn * lx + cs * ly, 1e-12);
  }

  // The bounding box must contain every corner.
  for (const auto& corner : fp.corners) {
    EXPECT_GE(corner.x, fp.min_x - 1e-12);
    EXPECT_LE(corner.x, fp.max_x + 1e-12);
    EXPECT_GE(corner.y, fp.min_y - 1e-12);
    EXPECT_LE(corner.y, fp.max_y + 1e-12);
  }
}

// ---------------------------------------------------------------------------
// Tilt rejection and unit-quaternion checks
// ---------------------------------------------------------------------------

TEST(A1Config, RejectsNonZeroRollOrPitchBecauseTiltIsUnsupported)
{
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.roll_deg = 5.0;
    const a1::ValidationResult r = a1::validateSamplingConfig(cfg);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_CONFIG_TILT_UNSUPPORTED");
  }
  {
    a1::SamplingConfig cfg = baseConfig();
    cfg.pitch_deg = -3.0;
    const a1::ValidationResult r = a1::validateSamplingConfig(cfg);
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_CONFIG_TILT_UNSUPPORTED");
  }
  // Flat is still accepted.
  EXPECT_TRUE(a1::validateSamplingConfig(baseConfig()).ok);
}

TEST(A1Geometry, UnitQuaternionCheckRejectsZeroAndUnnormalisedValues)
{
  EXPECT_TRUE(a1::isUnitQuat(a1::quatFromYaw(0.0)));
  EXPECT_TRUE(a1::isUnitQuat(a1::quatFromYaw(1.23)));
  EXPECT_FALSE(a1::isUnitQuat(a1::Quat {0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(a1::isUnitQuat(a1::Quat {0.0, 0.0, 0.0, 2.0}));
  EXPECT_FALSE(a1::isUnitQuat(a1::Quat {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0}));
  EXPECT_FALSE(
      a1::isUnitQuat(a1::Quat {0.0, 0.0, 0.0, std::numeric_limits<double>::infinity()}));
}

TEST(A1ExplicitPose, ProducesAUnitQuaternion)
{
  const a1::SamplingConfig cfg = baseConfig();
  for (double deg : {0.0, 10.0, -10.0, 179.0}) {
    a1::Candidate c;
    ASSERT_TRUE(a1::makeExplicitCandidate(cfg, 0, 0.5, 0.0, deg * M_PI / 180.0, c));
    EXPECT_TRUE(a1::isUnitQuat(c.quat)) << deg;
    EXPECT_NEAR(a1::yawFromQuat(c.quat), deg * M_PI / 180.0, 1e-12) << deg;
  }
}
