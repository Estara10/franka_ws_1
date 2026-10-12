// A1 P2a offline tests: shared target construction, rotated footprint,
// workspace/support judgement, joint-limit and residual helpers, and the
// ACM-entry classification. No ROS, no simulation.

#include "dual_arm_carry_task/a1_sample_validator.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

constexpr double kDeg = a1v::kPi / 180.0;

a1t::TruthPayload rodAt(double x, double y, double z, double yaw_deg)
{
  a1t::TruthPayload p;
  p.run_id = "a1_p2a_test";
  p.frame_id = "world";
  p.position = {x, y, z};
  p.orientation = a1v::quatFromRpy(0.0, 0.0, yaw_deg * kDeg);
  p.size = {0.04, 0.40, 0.04};
  p.mass_kg = 0.64;
  return p;
}

a1v::WorkspaceBounds defaultBounds()
{
  a1v::WorkspaceBounds b;
  b.x_min = 0.30;
  b.x_max = 0.70;
  b.y_min = -0.30;
  b.y_max = 0.30;
  b.source = "test";
  return b;
}

a1v::SupportCriteria defaultSupport()
{
  a1v::SupportCriteria s;
  s.support_z = 0.0;
  s.resting_z_tol = 0.003;
  s.max_tilt_deg = 2.0;
  return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// Target construction: the A0-consistency anchor
// ---------------------------------------------------------------------------

// The whole point of the shared constructor is that at yaw = 0 it reproduces
// the A0 targets exactly: left at obj+(0,+0.15,0), right at obj+(0,-0.15,0),
// both at setRPY(pi, 0, pi/4). If this drifts, P2b/P3 would inherit a
// different grasp than the one A0 was tuned with.
TEST(A1SampleTargets, AtZeroYawMatchesTheA0TargetsExactly)
{
  const a1t::TruthPayload rod = rodAt(0.50, 0.0, 0.02, 0.0);
  const a1v::GraspGeometry g;  // defaults are A0's values

  const a1v::DualTarget pre = a1v::buildPreGraspTarget(rod, g);
  EXPECT_NEAR(pre.left.position.x, 0.50, 1e-12);
  EXPECT_NEAR(pre.left.position.y, 0.15, 1e-12);
  EXPECT_NEAR(pre.left.position.z, 0.28, 1e-12);
  EXPECT_NEAR(pre.right.position.x, 0.50, 1e-12);
  EXPECT_NEAR(pre.right.position.y, -0.15, 1e-12);
  EXPECT_NEAR(pre.right.position.z, 0.28, 1e-12);

  const a1v::DualTarget grasp = a1v::buildGraspTarget(rod, g);
  EXPECT_NEAR(grasp.left.position.z, 0.13, 1e-12);
  EXPECT_NEAR(grasp.right.position.z, 0.13, 1e-12);

  // setRPY(pi, 0, pi/4) expands to this quaternion.
  const double c = std::cos(a1v::kPi / 8.0);
  const double s = std::sin(a1v::kPi / 8.0);
  EXPECT_NEAR(pre.left.orientation.x, c, 1e-12);
  EXPECT_NEAR(pre.left.orientation.y, s, 1e-12);
  EXPECT_NEAR(pre.left.orientation.z, 0.0, 1e-12);
  EXPECT_NEAR(pre.left.orientation.w, 0.0, 1e-12);
  EXPECT_NEAR(pre.right.orientation.x, c, 1e-12);
  EXPECT_NEAR(pre.right.orientation.y, s, 1e-12);
}

// Left is the +long-axis end and right the -long-axis end, at every yaw.
TEST(A1SampleTargets, LeftAndRightStayOnOppositeEndsOfTheLongAxis)
{
  const a1v::GraspGeometry g;
  for (double yaw_deg : {0.0, 10.0, -10.0, 15.0, -15.0, 90.0}) {
    const a1t::TruthPayload rod = rodAt(0.50, 0.0, 0.02, yaw_deg);
    double yaw = 0.0;
    const a1v::DualTarget t = a1v::buildPreGraspTarget(rod, g, &yaw);
    ASSERT_NEAR(yaw, yaw_deg * kDeg, 1e-12);

    // Vector from right to left target must be along the rod's long axis (+Y).
    const a1v::Vec3 axis = a1v::rotateZ(a1v::Vec3{0.0, 1.0, 0.0}, yaw);
    const a1v::Vec3 sep{t.left.position.x - t.right.position.x,
                        t.left.position.y - t.right.position.y, 0.0};
    const double sep_norm = std::hypot(sep.x, sep.y);
    ASSERT_NEAR(sep_norm, 2.0 * g.grasp_offset_m, 1e-12);
    EXPECT_NEAR(sep.x / sep_norm, axis.x, 1e-12);
    EXPECT_NEAR(sep.y / sep_norm, axis.y, 1e-12);
  }
}

// Both the offsets and the wrist orientation must turn with the rod.
TEST(A1SampleTargets, YawRotatesBothTheOffsetsAndTheWrist)
{
  const a1v::GraspGeometry g;
  const double yaw_deg = 10.0;
  const a1t::TruthPayload rod = rodAt(0.48, 0.03, 0.02, yaw_deg);
  const a1v::DualTarget t = a1v::buildPreGraspTarget(rod, g);

  const double yaw = yaw_deg * kDeg;
  EXPECT_NEAR(t.left.position.x, 0.48 - 0.15 * std::sin(yaw), 1e-12);
  EXPECT_NEAR(t.left.position.y, 0.03 + 0.15 * std::cos(yaw), 1e-12);
  EXPECT_NEAR(t.right.position.x, 0.48 + 0.15 * std::sin(yaw), 1e-12);
  EXPECT_NEAR(t.right.position.y, 0.03 - 0.15 * std::cos(yaw), 1e-12);

  // End-effector yaw = rod yaw + base yaw.
  EXPECT_NEAR(a1v::quatYaw(t.left.orientation), yaw + a1v::kPi / 4.0, 1e-12);
  EXPECT_NEAR(a1v::quatYaw(t.right.orientation), yaw + a1v::kPi / 4.0, 1e-12);
}

TEST(A1SampleTargets, GraspAndPreGraspShareXYAndOrientationAndDifferOnlyInZ)
{
  const a1v::GraspGeometry g;
  const a1t::TruthPayload rod = rodAt(0.52, -0.04, 0.02, -12.0);
  const a1v::DualTarget pre = a1v::buildPreGraspTarget(rod, g);
  const a1v::DualTarget grasp = a1v::buildGraspTarget(rod, g);

  EXPECT_NEAR(pre.left.position.x, grasp.left.position.x, 1e-12);
  EXPECT_NEAR(pre.left.position.y, grasp.left.position.y, 1e-12);
  EXPECT_NEAR(pre.right.position.x, grasp.right.position.x, 1e-12);
  EXPECT_NEAR(pre.right.position.y, grasp.right.position.y, 1e-12);
  EXPECT_NEAR(pre.left.orientation.x, grasp.left.orientation.x, 1e-12);
  EXPECT_NEAR(pre.left.orientation.y, grasp.left.orientation.y, 1e-12);
  EXPECT_GT(pre.left.position.z, grasp.left.position.z);
}

TEST(A1SampleTargets, ReverseYawSwapsTheOffsetsAboutTheRodCentre)
{
  const a1v::GraspGeometry g;
  const a1v::DualTarget a = a1v::buildPreGraspTarget(rodAt(0.5, 0.0, 0.02, 8.0), g);
  const a1v::DualTarget b = a1v::buildPreGraspTarget(rodAt(0.5, 0.0, 0.02, -8.0), g);
  // Mirroring the rod yaw mirrors the two grasp points about y = 0.
  EXPECT_NEAR(a.left.position.y, b.left.position.y, 1e-12);
  EXPECT_NEAR(a.left.position.x, -b.left.position.x + 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Footprint
// ---------------------------------------------------------------------------

TEST(A1SampleFootprint, ZeroYawGivesTheAxisAlignedRodBox)
{
  const a1v::Footprint fp = a1v::computeFootprint({0.50, 0.0, 0.0}, {0.04, 0.40, 0.04}, 0.0);
  EXPECT_NEAR(fp.x_min, 0.48, 1e-12);
  EXPECT_NEAR(fp.x_max, 0.52, 1e-12);
  EXPECT_NEAR(fp.y_min, -0.20, 1e-12);
  EXPECT_NEAR(fp.y_max, 0.20, 1e-12);
}

TEST(A1SampleFootprint, NinetyDegreesPutsTheLongAxisAlongX)
{
  const a1v::Footprint fp =
      a1v::computeFootprint({0.50, 0.0, 0.0}, {0.04, 0.40, 0.04}, a1v::kPi / 2.0);
  EXPECT_NEAR(fp.x_min, 0.30, 1e-12);
  EXPECT_NEAR(fp.x_max, 0.70, 1e-12);
  EXPECT_NEAR(fp.y_min, -0.02, 1e-12);
  EXPECT_NEAR(fp.y_max, 0.02, 1e-12);
}

TEST(A1SampleFootprint, SmallYawGrowsTheABBSymmetrically)
{
  const double yaw = 15.0 * kDeg;
  const a1v::Footprint fp = a1v::computeFootprint({0.50, 0.0, 0.0}, {0.04, 0.40, 0.04}, yaw);
  const double ex = 0.02 * std::cos(yaw) + 0.20 * std::sin(yaw);
  const double ey = 0.02 * std::sin(yaw) + 0.20 * std::cos(yaw);
  EXPECT_NEAR(fp.x_max - 0.50, ex, 1e-12);
  EXPECT_NEAR(0.50 - fp.x_min, ex, 1e-12);
  EXPECT_NEAR(fp.y_max, ey, 1e-12);
  EXPECT_NEAR(fp.y_min, -ey, 1e-12);
}

// ---------------------------------------------------------------------------
// Geometry judgement
// ---------------------------------------------------------------------------

TEST(A1SampleGeometry, AcceptsARestingRodInsideTheWorkspace)
{
  const a1v::GeometryResult r =
      a1v::checkGeometry(rodAt(0.50, 0.0, 0.02, 10.0), defaultBounds(), defaultSupport());
  EXPECT_EQ(r.item.status, a1v::CheckStatus::PASSED) << r.item.reason_code << ": " << r.item.detail;
  EXPECT_GT(r.footprint.min_clearance_m, 0.0);
  EXPECT_NEAR(r.tilt_deg, 0.0, 1e-9);
}

TEST(A1SampleGeometry, RejectsAFootprintOutsideTheWorkspace)
{
  const a1v::GeometryResult r =
      a1v::checkGeometry(rodAt(0.50, 0.28, 0.02, 0.0), defaultBounds(), defaultSupport());
  EXPECT_EQ(r.item.status, a1v::CheckStatus::FAILED);
  EXPECT_EQ(r.item.reason_code, "E_GEO_OUT_OF_WORKSPACE");
  // The reported clearance is negative, not silently clamped to zero.
  EXPECT_LT(r.footprint.min_clearance_m, 0.0);
}

// A long rod whose centre is well inside can still leave the workspace: this is
// exactly what the footprint check exists to catch.
TEST(A1SampleGeometry, RejectsWhenOnlyTheRodEndLeavesTheWorkspace)
{
  const a1v::GeometryResult r =
      a1v::checkGeometry(rodAt(0.50, 0.12, 0.02, 0.0), defaultBounds(), defaultSupport());
  EXPECT_EQ(r.item.status, a1v::CheckStatus::FAILED);
  EXPECT_EQ(r.item.reason_code, "E_GEO_OUT_OF_WORKSPACE");
}

TEST(A1SampleGeometry, RejectsAWrongRestingHeight)
{
  const a1v::GeometryResult r =
      a1v::checkGeometry(rodAt(0.50, 0.0, 0.10, 0.0), defaultBounds(), defaultSupport());
  EXPECT_EQ(r.item.status, a1v::CheckStatus::FAILED);
  EXPECT_EQ(r.item.reason_code, "E_GEO_SUPPORT_HEIGHT");
}

TEST(A1SampleGeometry, RejectsATiltedRod)
{
  a1t::TruthPayload rod = rodAt(0.50, 0.0, 0.02, 0.0);
  rod.orientation = a1v::quatFromRpy(10.0 * kDeg, 0.0, 0.0);  // rolled 10 deg
  const a1v::GeometryResult r = a1v::checkGeometry(rod, defaultBounds(), defaultSupport());
  EXPECT_EQ(r.item.status, a1v::CheckStatus::FAILED);
  EXPECT_EQ(r.item.reason_code, "E_GEO_TILT");
  EXPECT_NEAR(r.tilt_deg, 10.0, 1e-6);
}

TEST(A1SampleGeometry, RejectsBadInputAndBadConfigRatherThanGuessing)
{
  {
    a1t::TruthPayload rod = rodAt(0.50, 0.0, 0.02, 0.0);
    rod.size = {0.04, -0.40, 0.04};  // negative: never silently abs()'d
    const a1v::GeometryResult r = a1v::checkGeometry(rod, defaultBounds(), defaultSupport());
    EXPECT_EQ(r.item.reason_code, "E_GEO_BAD_INPUT");
  }
  {
    a1v::WorkspaceBounds degenerate = defaultBounds();
    degenerate.x_max = degenerate.x_min;
    const a1v::GeometryResult r =
        a1v::checkGeometry(rodAt(0.50, 0.0, 0.02, 0.0), degenerate, defaultSupport());
    EXPECT_EQ(r.item.reason_code, "E_GEO_BAD_CONFIG");
  }
}

// ---------------------------------------------------------------------------
// Joint limits and residual
// ---------------------------------------------------------------------------

TEST(A1SampleJointLimits, AcceptsAnInteriorConfigurationAndReportsTheWorstMargin)
{
  const std::vector<double> q{0.0, -0.5, 0.0};
  const std::vector<double> lo{-1.0, -1.0, -1.0};
  const std::vector<double> hi{1.0, 1.0, 1.0};
  const a1v::JointLimitCheck r =
      a1v::checkJointLimits(q, lo, hi, {"j1", "j2", "j3"});
  EXPECT_TRUE(r.ok);
  EXPECT_NEAR(r.worst_margin_rad, 0.5, 1e-12);
}

TEST(A1SampleJointLimits, RejectsAViolationAndNamesTheJoint)
{
  const std::vector<double> q{0.0, 1.2, 0.0};
  const std::vector<double> lo{-1.0, -1.0, -1.0};
  const std::vector<double> hi{1.0, 1.0, 1.0};
  const a1v::JointLimitCheck r =
      a1v::checkJointLimits(q, lo, hi, {"j1", "j2", "j3"});
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.offending_name, "j2");
  EXPECT_EQ(r.offending_index, 1u);
}

TEST(A1SampleJointLimits, RejectsNonFiniteAndSizeMismatch)
{
  const std::vector<double> lo{-1.0, -1.0};
  const std::vector<double> hi{1.0, 1.0};
  EXPECT_FALSE(a1v::checkJointLimits({0.0, std::nan("")}, lo, hi, {"a", "b"}).ok);
  EXPECT_FALSE(a1v::checkJointLimits({0.0}, lo, hi, {"a", "b"}).ok);
  EXPECT_FALSE(a1v::checkJointLimits({}, lo, hi, {}).ok);
}

TEST(A1SampleJointLimits, MaxDeltaFindsTheWorstJoint)
{
  EXPECT_NEAR(a1v::maxAbsJointDelta({0.0, 1.0, 2.0}, {0.1, 1.0, 1.5}), 0.5, 1e-12);
  EXPECT_NEAR(a1v::maxAbsJointDelta({0.0}, {0.0}), 0.0, 1e-12);
}

TEST(A1SampleResidual, MeasuresPositionAndGeodesicAngle)
{
  a1v::ArmTarget target;
  target.position = {0.5, 0.1, 0.28};
  target.orientation = a1v::quatFromRpy(a1v::kPi, 0.0, a1v::kPi / 4.0);

  a1v::ArmTarget actual = target;
  a1v::PoseError zero = a1v::poseError(target, actual);
  EXPECT_NEAR(zero.pos_m, 0.0, 1e-12);
  EXPECT_NEAR(zero.angle_rad, 0.0, 1e-12);

  actual.position.x += 0.003;  // 3 mm
  const a1v::PoseError e = a1v::poseError(target, actual);
  EXPECT_NEAR(e.pos_m, 0.003, 1e-12);
}

// q and -q are the same rotation, so the residual must not punish the sign.
TEST(A1SampleResidual, TreatsNegatedQuaternionAsNoError)
{
  a1v::ArmTarget target;
  target.orientation = a1v::quatFromRpy(a1v::kPi, 0.0, 0.0);
  a1v::ArmTarget actual = target;
  actual.orientation.x = -target.orientation.x;
  actual.orientation.y = -target.orientation.y;
  actual.orientation.z = -target.orientation.z;
  actual.orientation.w = -target.orientation.w;
  EXPECT_NEAR(a1v::poseError(target, actual).angle_rad, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// ACM classification
// ---------------------------------------------------------------------------

TEST(A1SampleAcm, ClassifiesCrossArmRodAndFingerPairs)
{
  EXPECT_TRUE(a1v::isCrossArmPair("mj_left_link5", "mj_right_link2"));
  EXPECT_TRUE(a1v::isCrossArmPair("mj_right_link4", "mj_left_link3"));
  EXPECT_FALSE(a1v::isCrossArmPair("mj_left_link5", "mj_left_link6"));
  EXPECT_FALSE(a1v::isCrossArmPair("mj_right_link5", "mj_right_link6"));
  EXPECT_FALSE(a1v::isCrossArmPair("base_link", "mj_left_link0"));

  EXPECT_TRUE(a1v::isRodPair("aluminum_rod", "mj_left_link5", "aluminum_rod"));
  EXPECT_TRUE(a1v::isRodPair("mj_right_hand", "aluminum_rod", "aluminum_rod"));
  EXPECT_FALSE(a1v::isRodPair("aluminum_rod", "aluminum_rod", "aluminum_rod"));
  EXPECT_FALSE(a1v::isRodPair("mj_left_link5", "mj_right_link5", "aluminum_rod"));

  EXPECT_TRUE(a1v::isFingerLink("mj_left_leftfinger"));
  EXPECT_TRUE(a1v::isFingerLink("mj_right_rightfinger"));
  EXPECT_FALSE(a1v::isFingerLink("mj_left_hand"));
  EXPECT_FALSE(a1v::isFingerLink("mj_left_link8"));
}

// The support surface is a distinct world object from the rod, and only the
// rigidly mounted links may be exempted from touching it.
TEST(A1SampleAcm, ClassifiesSupportPairsSeparatelyFromRodPairs)
{
  const std::string rod = "aluminum_rod";
  const std::string floor = "a1_support_plane";

  EXPECT_TRUE(a1v::isSupportPair("a1_support_plane", "mj_left_link5", floor));
  EXPECT_TRUE(a1v::isSupportPair("mj_right_hand", "a1_support_plane", floor));
  EXPECT_FALSE(a1v::isSupportPair("aluminum_rod", "mj_left_link5", floor));
  EXPECT_FALSE(a1v::isSupportPair("mj_left_link5", "mj_right_link5", floor));

  // A rod pair must not be mistaken for a support pair, and vice versa.
  EXPECT_FALSE(a1v::isRodPair("a1_support_plane", "mj_left_link5", rod));
  EXPECT_FALSE(a1v::isSupportPair("aluminum_rod", "mj_left_link5", floor));

  EXPECT_TRUE(a1v::involvesObject("x", "aluminum_rod", rod));
  EXPECT_FALSE(a1v::involvesObject("aluminum_rod", "aluminum_rod", rod));
}

TEST(A1SampleAcm, OnlyRigidlyMountedLinksAreGroundExempt)
{
  EXPECT_TRUE(a1v::isMountingLink("base_link"));
  EXPECT_TRUE(a1v::isMountingLink("mj_left_link0"));
  EXPECT_TRUE(a1v::isMountingLink("mj_right_link0"));
  // link1's lowest sphere is centred exactly on the mounting plane and joint1
  // only spins about the vertical axis, so it is part of the base assembly.
  EXPECT_TRUE(a1v::isMountingLink("mj_left_link1"));
  EXPECT_TRUE(a1v::isMountingLink("mj_right_link1"));

  // No free arm link or gripper link may be exempted from the ground.
  EXPECT_FALSE(a1v::isMountingLink("mj_left_link2"));
  EXPECT_FALSE(a1v::isMountingLink("mj_left_link5"));
  EXPECT_FALSE(a1v::isMountingLink("mj_left_link8"));
  EXPECT_FALSE(a1v::isMountingLink("mj_left_hand"));
  EXPECT_FALSE(a1v::isMountingLink("mj_right_leftfinger"));
  EXPECT_FALSE(a1v::isMountingLink(""));
}

// ---------------------------------------------------------------------------
// Hand AABB substitution helpers
// ---------------------------------------------------------------------------

TEST(A1SampleAabb, ComputesBoundsAndOffsetsTheCentreOffTheOrigin)
{
  // A box deliberately NOT centred on the origin: the substitution must use
  // (min+max)/2, not the hand origin.
  const std::vector<a1v::Vec3> pts = {
      {-0.0316, -0.1040, -0.0259},
      {0.0316, 0.1004, 0.0660},
      {0.0, 0.0, 0.0},
  };
  const a1v::Aabb b = a1v::aabbOfPoints(pts);
  ASSERT_TRUE(b.valid);
  EXPECT_NEAR(b.min.x, -0.0316, 1e-12);
  EXPECT_NEAR(b.min.y, -0.1040, 1e-12);
  EXPECT_NEAR(b.min.z, -0.0259, 1e-12);
  EXPECT_NEAR(b.max.x, 0.0316, 1e-12);

  const a1v::Vec3 c = b.center();
  const a1v::Vec3 s = b.size();
  EXPECT_NEAR(c.x, 0.0, 1e-12);
  EXPECT_NEAR(c.y, -0.0018, 1e-12);
  EXPECT_NEAR(c.z, 0.02005, 1e-12);
  EXPECT_NEAR(s.y, 0.2044, 1e-12);
  EXPECT_NEAR(s.z, 0.0919, 1e-12);

  EXPECT_TRUE(a1v::aabbContainsAll(b, pts));
}

TEST(A1SampleAabb, RejectsAnEmptyPointSetAndDetectsEscapingPoints)
{
  EXPECT_FALSE(a1v::aabbOfPoints({}).valid);

  const a1v::Aabb b = a1v::aabbOfPoints({{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}});
  EXPECT_TRUE(a1v::aabbContainsAll(b, {{0.5, 0.5, 0.5}}));
  EXPECT_FALSE(a1v::aabbContainsAll(b, {{1.5, 0.5, 0.5}}));
  EXPECT_FALSE(a1v::aabbContainsAll(b, {{0.5, -0.001, 0.5}}));
  EXPECT_FALSE(a1v::aabbContainsAll(a1v::Aabb{}, {{0.0, 0.0, 0.0}}));  // invalid box
}

namespace {

// Writes a minimal binary STL with the given triangles to /tmp.
std::string writeTestStl(const std::string& name, const std::vector<std::array<float, 9>>& tris)
{
  const std::string path = "/tmp/" + name;
  std::ofstream out(path, std::ios::binary);
  char header[80] = {0};
  out.write(header, 80);
  const std::uint32_t n = static_cast<std::uint32_t>(tris.size());
  out.write(reinterpret_cast<const char*>(&n), 4);
  for (const auto& t : tris) {
    float record[12] = {0, 0, 0, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7], t[8]};
    out.write(reinterpret_cast<const char*>(record), 50 - 2);  // pad to 50
    char pad[2] = {0, 0};
    out.write(pad, 2);
  }
  out.close();
  return path;
}

}  // namespace

TEST(A1SampleStl, ReadsVerticesFromABinaryStl)
{
  const std::string path =
      writeTestStl("a1_test_ok.stl", {{{-1, -2, -3, 4, 5, 6, 7, 8, 9}}});
  std::vector<a1v::Vec3> verts;
  std::string err;
  ASSERT_TRUE(a1v::readBinaryStlVertices(path, verts, err)) << err;
  ASSERT_EQ(verts.size(), 3u);
  EXPECT_FLOAT_EQ(static_cast<float>(verts[0].x), -1.0f);
  EXPECT_FLOAT_EQ(static_cast<float>(verts[2].z), 9.0f);

  const a1v::Aabb b = a1v::aabbOfPoints(verts);
  EXPECT_TRUE(a1v::aabbContainsAll(b, verts));
}

// A load failure must be reported, never silently turned into an empty model.
TEST(A1SampleStl, ReportsFailuresInsteadOfReturningAnEmptyModel)
{
  std::vector<a1v::Vec3> verts;
  std::string err;

  EXPECT_FALSE(a1v::readBinaryStlVertices("/tmp/a1_does_not_exist.stl", verts, err));
  EXPECT_FALSE(err.empty());
  EXPECT_TRUE(verts.empty());

  // Declared triangle count inconsistent with the file size (e.g. an ASCII STL
  // or a truncated file) must be rejected, not partially read.
  const std::string bad = "/tmp/a1_test_bad.stl";
  {
    std::ofstream out(bad, std::ios::binary);
    char header[80] = {0};
    out.write(header, 80);
    const std::uint32_t n = 5;  // claims 5 triangles, supplies none
    out.write(reinterpret_cast<const char*>(&n), 4);
  }
  EXPECT_FALSE(a1v::readBinaryStlVertices(bad, verts, err));
  EXPECT_FALSE(err.empty());
  EXPECT_TRUE(verts.empty());
}

// ---------------------------------------------------------------------------
// Path segment sampling (P2b)
// ---------------------------------------------------------------------------

TEST(A1SamplePath, SubdividesSoNoStepExceedsTheLimit)
{
  EXPECT_EQ(a1v::subdivisionsFor(0.0, 0.05), 1u);        // zero-length segment
  EXPECT_EQ(a1v::subdivisionsFor(0.01, 0.05), 1u);       // already inside one step
  EXPECT_EQ(a1v::subdivisionsFor(0.10, 0.05), 2u);
  EXPECT_EQ(a1v::subdivisionsFor(0.11, 0.05), 3u);       // rounds up, never down
  EXPECT_EQ(a1v::subdivisionsFor(1.0, 0.05), 20u);

  // A degenerate step must not produce zero intervals (which would skip a whole
  // segment) or divide by zero.
  EXPECT_EQ(a1v::subdivisionsFor(0.5, 0.0), 1u);
  EXPECT_EQ(a1v::subdivisionsFor(0.5, -0.1), 1u);
  EXPECT_EQ(a1v::subdivisionsFor(std::nan(""), 0.05), 1u);
}

TEST(A1SamplePath, InterpolationHitsBothEndsAndIsClamped)
{
  const std::vector<double> a{0.0, -1.0};
  const std::vector<double> b{1.0, 1.0};

  const std::vector<double> mid = a1v::interpolateJoints(a, b, 0.5);
  ASSERT_EQ(mid.size(), 2u);
  EXPECT_DOUBLE_EQ(mid[0], 0.5);
  EXPECT_DOUBLE_EQ(mid[1], 0.0);

  EXPECT_DOUBLE_EQ(a1v::interpolateJoints(a, b, 0.0)[0], 0.0);
  EXPECT_DOUBLE_EQ(a1v::interpolateJoints(a, b, 1.0)[0], 1.0);
  // Out-of-range t is clamped, not extrapolated past the endpoints.
  EXPECT_DOUBLE_EQ(a1v::interpolateJoints(a, b, -5.0)[0], 0.0);
  EXPECT_DOUBLE_EQ(a1v::interpolateJoints(a, b, 7.0)[0], 1.0);

  // Mismatched dimensions must not silently produce a wrong-length state.
  EXPECT_TRUE(a1v::interpolateJoints({1.0}, {1.0, 2.0}, 0.5).empty());
}

// ---------------------------------------------------------------------------
// Aggregation
// ---------------------------------------------------------------------------

TEST(A1SampleAggregate, RequiresAllThreeStaticChecksAndAlwaysLeavesPathNotRun)
{
  a1v::SampleValidation v;
  EXPECT_FALSE(v.allStaticPassed());
  EXPECT_EQ(v.path_validation.status, a1v::CheckStatus::NOT_RUN);

  v.geometry.status = a1v::CheckStatus::PASSED;
  v.ik.status = a1v::CheckStatus::PASSED;
  v.collision.status = a1v::CheckStatus::PASSED;
  EXPECT_TRUE(v.allStaticPassed());
  EXPECT_EQ(v.summary(), "geometry=passed ik=passed collision=passed path_validation=NOT_RUN");

  v.collision.status = a1v::CheckStatus::FAILED;
  EXPECT_FALSE(v.allStaticPassed());
}

TEST(A1SampleAggregate, StatusNamesAreStable)
{
  EXPECT_STREQ(a1v::statusName(a1v::CheckStatus::PASSED), "passed");
  EXPECT_STREQ(a1v::statusName(a1v::CheckStatus::FAILED), "failed");
  EXPECT_STREQ(a1v::statusName(a1v::CheckStatus::NOT_RUN), "NOT_RUN");
}
