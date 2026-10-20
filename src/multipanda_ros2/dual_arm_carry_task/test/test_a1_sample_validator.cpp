// A1 P2a offline tests: shared target construction, rotated footprint,
// workspace/support judgement, joint-limit and residual helpers, and the
// ACM-entry classification. No ROS, no simulation.

#include "dual_arm_carry_task/a1_sample_validator.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <stdexcept>
#include <fstream>
#include <iomanip>
#include <sstream>
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

TEST(A1FingerSnapshot, MapsFourDistinctMeasuredValuesByJointName)
{
  const a1v::FingerFeedbackSample left{0.011, 0.033, 11, 1000000001LL, 4.10};
  const a1v::FingerFeedbackSample right{0.027, 0.013, 22, 1000000002LL, 4.11};
  std::map<std::string, double> global;
  global["mj_left_finger_joint1"] = left.q1;
  global["mj_left_finger_joint2"] = left.q2;
  global["mj_right_finger_joint1"] = right.q1;
  global["mj_right_finger_joint2"] = right.q2;
  a1v::FingerJointSnapshot snapshot;
  std::string reason;
  ASSERT_TRUE(a1v::makeFingerJointSnapshot(left, right, global, 1000000003LL, 4.12,
                                           4.00, 4.20, {{10, 21}}, 0.5, 1e-6,
                                           snapshot, reason)) << reason;
  EXPECT_EQ(snapshot.positions, (std::array<double, 4>{{0.011, 0.033, 0.027, 0.013}}));
  EXPECT_EQ(snapshot.source_seq, (std::array<std::uint64_t, 2>{{11, 22}}));

  std::map<std::string, double> mapped;
  ASSERT_TRUE(a1v::mapMeasuredFingerPositions(snapshot.positions, mapped, reason)) << reason;
  ASSERT_EQ(mapped.size(), 4u);
  const auto& names = a1v::fingerJointNames();
  for (std::size_t i = 0; i < names.size(); ++i) {
    EXPECT_DOUBLE_EQ(mapped.at(names[i]), snapshot.positions[i]);
  }
}

TEST(A1FingerSnapshot, RejectsMissingOldAndContradictoryFeedback)
{
  const a1v::FingerFeedbackSample left{0.040, 0.039, 11, 1000000001LL, 4.10};
  const a1v::FingerFeedbackSample right{0.040, 0.039, 22, 1000000002LL, 4.10};
  const std::map<std::string, double> complete = {
      {"mj_left_finger_joint1", 0.040}, {"mj_left_finger_joint2", 0.039},
      {"mj_right_finger_joint1", 0.040}, {"mj_right_finger_joint2", 0.039}};
  a1v::FingerJointSnapshot out;
  std::string reason;

  auto missing = complete;
  missing.erase("mj_right_finger_joint2");
  EXPECT_FALSE(a1v::makeFingerJointSnapshot(left, right, missing, 1000000003LL, 4.12,
                                            4.00, 4.20, {{10, 21}}, 0.5, 1e-6,
                                            out, reason));

  const std::map<std::string, double> no_global_fingers;
  ASSERT_TRUE(a1v::makeFingerJointSnapshot(left, right, no_global_fingers, 0, 0.0,
                                           4.00, 4.20, {{10, 21}}, 0.5, 1e-6,
                                           out, reason)) << reason;
  EXPECT_FALSE(out.global_joint_state_matched);

  const a1v::FingerFeedbackSample absent{0.040, 0.039, 0, 1000000001LL, 4.10};
  EXPECT_FALSE(a1v::makeFingerJointSnapshot(absent, right, no_global_fingers, 0, 0.0,
                                            4.00, 4.20, {{0, 21}}, 0.5, 1e-6,
                                            out, reason));

  EXPECT_FALSE(a1v::makeFingerJointSnapshot(left, right, complete, 1000000003LL, 4.12,
                                            4.00, 4.20, {{11, 21}}, 0.5, 1e-6,
                                            out, reason));

  const a1v::FingerFeedbackSample stale{left.q1, left.q2, 12, left.stamp_ns, 3.0};
  EXPECT_FALSE(a1v::makeFingerJointSnapshot(stale, right, complete, 1000000003LL, 4.12,
                                            4.00, 4.20, {{11, 21}}, 0.5, 1e-6,
                                            out, reason));

  auto conflict = complete;
  conflict["mj_left_finger_joint2"] = 0.0;
  EXPECT_FALSE(a1v::makeFingerJointSnapshot(left, right, conflict, 1000000003LL, 4.12,
                                            4.00, 4.20, {{10, 21}}, 0.5, 1e-6,
                                            out, reason));
}

TEST(A1P2bStartState, MapsFreshArmAndTypedFingerFeedbackWithoutDefaults)
{
  std::map<std::string, double> arms;
  for (const std::string side : {"mj_left", "mj_right"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      arms.emplace(side + "_joint" + std::to_string(joint), 0.01 * joint);
    }
  }
  const std::array<double, 4> fingers{{0.011, 0.019, 0.027, 0.033}};
  std::map<std::string, double> mapped;
  std::string reason;
  ASSERT_TRUE(a1v::mapP2bStartPositions(arms, fingers, 0.01, {{0.02, 0.03}}, 0.5,
                                        mapped, reason)) << reason;
  ASSERT_EQ(mapped.size(), 18u);
  EXPECT_DOUBLE_EQ(mapped.at("mj_left_joint4"), 0.04);
  EXPECT_DOUBLE_EQ(mapped.at("mj_right_joint7"), 0.07);
  EXPECT_DOUBLE_EQ(mapped.at("mj_left_finger_joint1"), 0.011);
  EXPECT_DOUBLE_EQ(mapped.at("mj_left_finger_joint2"), 0.019);
  EXPECT_DOUBLE_EQ(mapped.at("mj_right_finger_joint1"), 0.027);
  EXPECT_DOUBLE_EQ(mapped.at("mj_right_finger_joint2"), 0.033);

  auto missing_arm = arms;
  missing_arm.erase("mj_right_joint5");
  EXPECT_FALSE(a1v::mapP2bStartPositions(missing_arm, fingers, 0.01, {{0.02, 0.03}}, 0.5,
                                         mapped, reason));
  EXPECT_NE(reason.find("mj_right_joint5"), std::string::npos);
  EXPECT_FALSE(a1v::mapP2bStartPositions(arms, fingers, 0.01, {{0.02, 0.51}}, 0.5,
                                         mapped, reason));
  EXPECT_NE(reason.find("right typed finger"), std::string::npos);
}

TEST(A1P2bStartState, AcceptsB05MicroscopicNegativeFingerValuesWithoutChangingThem)
{
  std::map<std::string, double> arms;
  for (const std::string side : {"mj_left", "mj_right"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      arms.emplace(side + "_joint" + std::to_string(joint), 0.01 * joint);
    }
  }
  const std::array<double, 4> b05{{
      -1.5315476842799908e-08, 1.5315476842799905e-08,
      -1.5315530607144407e-08, 1.5315530607144407e-08}};
  std::map<std::string, double> mapped;
  std::string reason;
  ASSERT_TRUE(a1v::mapP2bStartPositions(arms, b05, 0.01, {{0.02, 0.03}}, 0.5,
                                        mapped, reason)) << reason;
  EXPECT_EQ(reason, "ok_with_numeric_tolerance");
  const auto& names = a1v::fingerJointNames();
  for (std::size_t i = 0; i < names.size(); ++i) {
    EXPECT_DOUBLE_EQ(mapped.at(names[i]), b05[i]);
  }

  std::map<std::string, double> direct;
  ASSERT_TRUE(a1v::mapMeasuredFingerPositions(b05, direct, reason)) << reason;
  EXPECT_EQ(reason, "ok_with_numeric_tolerance");
  for (std::size_t i = 0; i < names.size(); ++i) {
    EXPECT_DOUBLE_EQ(direct.at(names[i]), b05[i]);
  }
}

TEST(A1P2bStartState, FingerNumericToleranceIsLimitedToApprovedLowerBoundary)
{
  std::map<std::string, double> mapped;
  std::string reason;
  const std::array<double, 4> at_lower{{-1e-6, 0.0, 0.0401, 0.02}};
  ASSERT_TRUE(a1v::mapMeasuredFingerPositions(at_lower, mapped, reason)) << reason;
  EXPECT_DOUBLE_EQ(mapped.at("mj_left_finger_joint1"), -1e-6);
  EXPECT_DOUBLE_EQ(mapped.at("mj_right_finger_joint1"), 0.0401);
  EXPECT_EQ(reason, "ok_with_numeric_tolerance");

  auto below_lower = at_lower;
  below_lower[0] = -1.0001e-6;
  EXPECT_FALSE(a1v::mapMeasuredFingerPositions(below_lower, mapped, reason));
  EXPECT_NE(reason.find("mj_left_finger_joint1"), std::string::npos);

  auto above_upper = at_lower;
  above_upper[2] = 0.0401000001;
  EXPECT_FALSE(a1v::mapMeasuredFingerPositions(above_upper, mapped, reason));
  EXPECT_NE(reason.find("mj_right_finger_joint1"), std::string::npos);
}

TEST(A1P2bFingerDiagnostic, ClassifiesMissingStaleAndRawValueFailures)
{
  const auto classify = [](double raw, bool present, double age,
                           const std::string& actual = "mj_left_finger_joint1") {
    return a1v::classifyP2bFingerInput(raw, present, age, 0.5, actual,
                                      "mj_left_finger_joint1",
                                      a1v::kFingerNumericLowerBoundM,
                                      a1v::kFingerAcceptedUpperBoundM);
  };

  const double missing_value = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(classify(missing_value, false, missing_value), a1v::FingerInputClass::MISSING);
  EXPECT_EQ(classify(0.01, true, 0.5001), a1v::FingerInputClass::STALE);
  EXPECT_EQ(classify(std::numeric_limits<double>::quiet_NaN(), true, 0.01),
            a1v::FingerInputClass::NONFINITE);
  EXPECT_EQ(classify(std::numeric_limits<double>::infinity(), true, 0.01),
            a1v::FingerInputClass::NONFINITE);
  EXPECT_EQ(classify(-0.0002, true, 0.01), a1v::FingerInputClass::BELOW_LOWER);
  EXPECT_EQ(classify(-1e-6, true, 0.01),
            a1v::FingerInputClass::ACCEPTED_WITH_NUMERIC_TOLERANCE);
  EXPECT_EQ(classify(-1.5315476842799908e-08, true, 0.01),
            a1v::FingerInputClass::ACCEPTED_WITH_NUMERIC_TOLERANCE);
  EXPECT_EQ(classify(0.0, true, 0.01), a1v::FingerInputClass::VALID);
  EXPECT_EQ(classify(0.0402, true, 0.01), a1v::FingerInputClass::ABOVE_UPPER);
  EXPECT_EQ(classify(0.01, true, 0.01, "mj_right_finger_joint1"),
            a1v::FingerInputClass::MAPPING_MISMATCH);
  EXPECT_EQ(classify(0.0401, true, 0.01), a1v::FingerInputClass::VALID);

  const double raw_out_of_range = 0.0402;
  const auto rejected = classify(raw_out_of_range, true, 0.01);
  EXPECT_EQ(rejected, a1v::FingerInputClass::ABOVE_UPPER);
  EXPECT_DOUBLE_EQ(raw_out_of_range, 0.0402);
  EXPECT_STREQ(a1v::fingerInputClassName(rejected), "above_upper_bound");
}

TEST(A1P2bFingerDiagnostic, WritesFailedRawValuesWithoutDefaultingOrClamping)
{
  const std::array<double, 4> raw{{
      std::numeric_limits<double>::quiet_NaN(), -0.000123456789012345,
      0.04023456789012345, 0.0123456789012345}};
  const std::array<bool, 4> present{{false, true, true, true}};
  const std::array<a1v::FingerInputClass, 4> classes{{
      a1v::FingerInputClass::MISSING, a1v::FingerInputClass::BELOW_LOWER,
      a1v::FingerInputClass::ABOVE_UPPER, a1v::FingerInputClass::VALID}};
  std::ostringstream output;
  a1v::writeP2bFingerRawDiagnosticCsv(output, raw, present, classes);
  const std::string csv = output.str();
  EXPECT_NE(csv.find("0,mj_left_finger_joint1,0,NA,missing"), std::string::npos);
  std::ostringstream expected_negative;
  expected_negative << std::setprecision(17) << raw[1];
  std::ostringstream expected_high;
  expected_high << std::setprecision(17) << raw[2];
  EXPECT_NE(csv.find("1,mj_left_finger_joint2,1," + expected_negative.str()), std::string::npos);
  EXPECT_NE(csv.find("2,mj_right_finger_joint1,1," + expected_high.str()), std::string::npos);
  EXPECT_NE(csv.find("above_upper_bound"), std::string::npos);
  std::ostringstream expected_valid;
  expected_valid << std::setprecision(17) << raw[3];
  EXPECT_NE(csv.find("3,mj_right_finger_joint2,1," + expected_valid.str() + ",valid"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(raw[1], -0.000123456789012345);
  EXPECT_DOUBLE_EQ(raw[2], 0.04023456789012345);

  const std::array<double, 4> nonfinite_raw{{
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0}};
  const std::array<bool, 4> nonfinite_present{{true, false, false, false}};
  const std::array<a1v::FingerInputClass, 4> nonfinite_classes{{
      a1v::FingerInputClass::NONFINITE, a1v::FingerInputClass::MISSING,
      a1v::FingerInputClass::MISSING, a1v::FingerInputClass::MISSING}};
  std::ostringstream nonfinite_output;
  a1v::writeP2bFingerRawDiagnosticCsv(nonfinite_output, nonfinite_raw,
                                      nonfinite_present, nonfinite_classes);
  EXPECT_NE(nonfinite_output.str().find(
      "0,mj_left_finger_joint1,1,NaN,nonfinite"), std::string::npos);
}

TEST(A1FingerAxis, NonCommutingRuntimeRotationChainClosesAcrossRodWidth)
{
  const a1v::Vec3 rod_x_world_base{1.0, 0.0, 0.0};
  for (double yaw_deg : {-90.0, -15.0, 0.0, 10.0, 45.0, 90.0, 170.0}) {
    const double yaw = yaw_deg * kDeg;
    const a1v::Vec3 closure = a1v::fingerClosureAxisForRodYaw(yaw);
    const a1v::Vec3 rod_long = a1v::rotateZ({0.0, 1.0, 0.0}, yaw);
    const a1v::Vec3 rod_width = a1v::rotateZ(rod_x_world_base, yaw);
    EXPECT_NEAR(closure.x, rod_width.x, 1e-12) << "yaw=" << yaw_deg;
    EXPECT_NEAR(closure.y, rod_width.y, 1e-12) << "yaw=" << yaw_deg;
    EXPECT_NEAR(closure.x * rod_long.x + closure.y * rod_long.y, 0.0, 1e-12)
        << "yaw=" << yaw_deg;
  }
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
// Trajectory structural validation (P3a-1)
// ---------------------------------------------------------------------------

namespace {

std::vector<std::string> dualArmJoints()
{
  std::vector<std::string> v;
  for (int i = 1; i <= 7; ++i) {
    v.push_back("mj_left_joint" + std::to_string(i));
  }
  for (int i = 1; i <= 7; ++i) {
    v.push_back("mj_right_joint" + std::to_string(i));
  }
  return v;
}

}  // namespace

TEST(A1SampleTrajNames, AcceptsBothArmsNamedExactlyOnce)
{
  const auto expected = dualArmJoints();
  ASSERT_EQ(expected.size(), 14u);
  const a1v::JointNameCheck r = a1v::validateJointNames(expected, expected);
  EXPECT_TRUE(r.ok) << r.reason_code << ": " << r.detail;
  EXPECT_EQ(r.missing, 0u);
  EXPECT_EQ(r.duplicates, 0u);
  EXPECT_EQ(r.unknown, 0u);
}

// Dropping one arm's joints would leave half the robot uncommanded.
TEST(A1SampleTrajNames, RejectsAMissingJoint)
{
  auto given = dualArmJoints();
  given.erase(given.begin() + 3);  // drop mj_left_joint4
  const a1v::JointNameCheck r = a1v::validateJointNames(given, dualArmJoints());
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_TRAJ_MISSING_JOINT");
  EXPECT_EQ(r.missing, 1u);
}

TEST(A1SampleTrajNames, RejectsDuplicatesAndUnknownNames)
{
  auto dup = dualArmJoints();
  dup[7] = dup[0];  // repeat mj_left_joint1, losing a right-arm joint
  const a1v::JointNameCheck r = a1v::validateJointNames(dup, dualArmJoints());
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_TRAJ_DUPLICATE_JOINT");
  EXPECT_EQ(r.first_duplicate, "mj_left_joint1");

  auto unknown = dualArmJoints();
  unknown[2] = "mj_left_joint99";
  const a1v::JointNameCheck r2 = a1v::validateJointNames(unknown, dualArmJoints());
  EXPECT_FALSE(r2.ok);
  EXPECT_EQ(r2.reason_code, "E_TRAJ_UNKNOWN_JOINT");
  EXPECT_EQ(r2.first_unknown, "mj_left_joint99");
}

TEST(A1SampleTrajNames, RejectsAnEmptyExpectationRatherThanAcceptingAnything)
{
  const a1v::JointNameCheck r = a1v::validateJointNames({"a"}, {});
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_TRAJ_NO_EXPECTED_JOINTS");
}

TEST(A1SampleTrajTime, AcceptsAStrictlyIncreasingFiniteSequenceStartingAtZero)
{
  const a1v::TimeCheck r = a1v::validateTimeFromStart({0.0, 0.5, 1.25, 2.0});
  EXPECT_TRUE(r.ok) << r.reason_code << ": " << r.detail;
  EXPECT_DOUBLE_EQ(r.first_sec, 0.0);
  EXPECT_DOUBLE_EQ(r.last_sec, 2.0);
}

TEST(A1SampleTrajTime, RejectsNonMonotonicAndNonFiniteTimes)
{
  {
    const a1v::TimeCheck r = a1v::validateTimeFromStart({0.0, 1.0, 1.0, 2.0});
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_TIME_NOT_INCREASING");
    EXPECT_EQ(r.bad_index, 2u);
  }
  {
    const a1v::TimeCheck r = a1v::validateTimeFromStart({0.0, 1.0, 0.5});
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_TIME_NOT_INCREASING");
  }
  {
    const a1v::TimeCheck r = a1v::validateTimeFromStart({0.0, std::nan(""), 2.0});
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_TIME_NOT_FINITE");
  }
  {
    const a1v::TimeCheck r = a1v::validateTimeFromStart({0.0, std::numeric_limits<double>::infinity()});
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_TIME_NOT_FINITE");
  }
}

TEST(A1SampleTrajTime, RejectsEmptyNegativeAndNonZeroStart)
{
  EXPECT_EQ(a1v::validateTimeFromStart({}).reason_code, "E_TIME_EMPTY");
  EXPECT_EQ(a1v::validateTimeFromStart({-0.1, 0.5}).reason_code, "E_TIME_NEGATIVE");
  // A first point that does not start at zero means the offsets are relative to
  // something other than the execution start.
  const a1v::TimeCheck r = a1v::validateTimeFromStart({0.4, 0.9});
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "E_TIME_NOT_STARTING_AT_ZERO");
}

// ---------------------------------------------------------------------------
// Bounded dispatch: late results stay safe and never become a success
// ---------------------------------------------------------------------------

TEST(A1SampleBoundedExec, ReturnsTheValueWhenTheWorkerFinishesInTime)
{
  int stops = 0;
  const a1v::BoundedExecOutcome r =
      a1v::runBoundedExec([]() { return 1; }, [&stops]() { ++stops; }, 1.0, 0.5, 0.005);
  EXPECT_EQ(r.status, a1v::BoundedExecOutcome::Status::COMPLETED);
  EXPECT_EQ(r.value, 1);
  EXPECT_TRUE(r.worker_finished);
  EXPECT_FALSE(r.stop_requested);  // the timeout path was never taken
  EXPECT_EQ(stops, 0);
}

TEST(A1SampleBoundedExec, ReportsExceptionsAsFailureNotCompletion)
{
  const a1v::BoundedExecOutcome r = a1v::runBoundedExec(
      []() -> int { throw std::runtime_error("boom"); }, []() {}, 1.0, 0.2, 0.005);
  EXPECT_EQ(r.status, a1v::BoundedExecOutcome::Status::EXCEPTION);
  EXPECT_TRUE(r.worker_finished);
}

// A success that arrives after the timeout must NOT be reported as an in-time
// success -- that is what keeps a late result from becoming PREGRASP_REACHED.
TEST(A1SampleBoundedExec, ALateSuccessIsStillReportedAsTimeout)
{
  int stops = 0;
  const auto begin = std::chrono::steady_clock::now();
  const a1v::BoundedExecOutcome r = a1v::runBoundedExec(
      []() {
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        return 1;  // MoveIt SUCCESS, but too late
      },
      [&stops]() { ++stops; }, 0.02, 1.0, 0.005);
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

  EXPECT_EQ(r.status, a1v::BoundedExecOutcome::Status::TIMEOUT);
  EXPECT_EQ(r.value, 1);                // the late value is recorded ...
  EXPECT_GE(r.elapsed_sec, 0.02);       // ... but only after the bound expired
  EXPECT_LT(elapsed, 1.0);
  EXPECT_TRUE(r.stop_requested);        // a stop was requested on timeout
  EXPECT_TRUE(r.worker_finished);       // it ended inside the grace period
}

// The worker outliving both the timeout and the grace period must be safe: it
// writes to shared state, never to a dead stack frame.
TEST(A1SampleBoundedExec, AWorkerOutlivingTheGracePeriodIsSafeAndDetached)
{
  auto late_writer_ran = std::make_shared<std::atomic<bool>>(false);
  int stops = 0;
  {
    const a1v::BoundedExecOutcome r = a1v::runBoundedExec(
        [late_writer_ran]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
          late_writer_ran->store(true);  // writes to shared state after our scope
          return 1;
        },
        [&stops]() { ++stops; }, 0.02, 0.05, 0.005);
    EXPECT_EQ(r.status, a1v::BoundedExecOutcome::Status::TIMEOUT);
    EXPECT_TRUE(r.stop_requested);
    EXPECT_FALSE(r.worker_finished);  // detached rather than joined
  }
  // The shared slot and this flag both outlive the call; the late write lands
  // safely. (A stack-captured result would be a use-after-scope here.)
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  EXPECT_TRUE(late_writer_ran->load());
  EXPECT_EQ(stops, 1);
}

// The deadline must be judged from the worker's own completion instant, not
// from when the polling loop woke up. Here the worker finishes AFTER the budget
// but BEFORE the next poll wake-up, so a naive "is the slot filled?" check would
// report COMPLETED. It must stay TIMEOUT.
TEST(A1SampleBoundedExec, AResultCompletingAfterTheDeadlineButBeforeTheNextPollIsTimeout)
{
  int stops = 0;
  const a1v::BoundedExecOutcome r = a1v::runBoundedExec(
      []() {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        return 1;  // MoveIt SUCCESS, but past the 10 ms budget
      },
      [&stops]() { ++stops; }, 0.010, 1.0, 0.050);

  EXPECT_EQ(r.status, a1v::BoundedExecOutcome::Status::TIMEOUT);
  EXPECT_EQ(r.value, 1);  // late value kept for diagnostics only
  EXPECT_GT(r.completed_at_sec, r.timeout_sec);  // finished after the budget
  EXPECT_GE(r.completed_at_sec, 0.020);
  EXPECT_TRUE(r.worker_finished);
  EXPECT_GE(r.elapsed_sec, 0.050);   // observed only at the next poll wake-up
  EXPECT_GT(r.elapsed_sec, r.completed_at_sec);
  // The worker had already ended, so a stop is neither needed nor requested --
  // only a still-running worker gets one. The status is TIMEOUT regardless.
  EXPECT_FALSE(r.stop_requested);
  EXPECT_EQ(stops, 0);
}

// The mirror image: the worker finishes well inside the budget, but the long
// poll interval means it is only noticed much later. That must NOT be turned
// into a timeout by the late observation.
TEST(A1SampleBoundedExec, AnOnTimeResultObservedLateIsStillCompleted)
{
  int stops = 0;
  const a1v::BoundedExecOutcome r = a1v::runBoundedExec(
      []() {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return 1;
      },
      [&stops]() { ++stops; }, 1.0, 0.5, 0.200);

  EXPECT_EQ(r.status, a1v::BoundedExecOutcome::Status::COMPLETED);
  EXPECT_EQ(r.value, 1);
  EXPECT_LE(r.completed_at_sec, r.timeout_sec);  // finished inside the budget
  EXPECT_TRUE(r.worker_finished);
  EXPECT_FALSE(r.stop_requested);                // never entered the stop path
  EXPECT_EQ(stops, 0);
  // It was observed at the poll wake-up, well after it actually finished.
  EXPECT_GT(r.elapsed_sec, r.completed_at_sec);
}

TEST(A1SampleBoundedExec, ZeroTimeoutStillRequestsStopAndDoesNotHang)
{
  const auto begin = std::chrono::steady_clock::now();
  const a1v::BoundedExecOutcome r = a1v::runBoundedExec(
      []() { std::this_thread::sleep_for(std::chrono::milliseconds(60)); return 7; }, []() {},
      0.0, 0.005, 0.002);
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  EXPECT_EQ(r.status, a1v::BoundedExecOutcome::Status::TIMEOUT);
  EXPECT_LT(elapsed, 1.0);  // bounded, no unbounded join
}

// ---------------------------------------------------------------------------
// Final dispatched velocity / acceleration fields
// ---------------------------------------------------------------------------

namespace {

std::vector<std::string> twoJoints() { return {"j1", "j2"}; }

}  // namespace

TEST(A1SampleDerivative, AcceptsAFieldInsideTheDeclaredLimits)
{
  const a1v::DerivativeCheck r =
      a1v::checkDerivativeField({0.5, -1.0}, {1.0, 2.0}, {true, true}, twoJoints());
  EXPECT_TRUE(r.ok) << r.reason_code << ": " << r.detail;
  EXPECT_EQ(r.checked, 2u);
  EXPECT_EQ(r.unbounded, 0u);
  EXPECT_NEAR(r.worst_ratio, 0.5, 1e-12);
}

// A joint with no declared limit must be counted, never treated as a zero
// limit and never claimed as compliant.
TEST(A1SampleDerivative, CountsUndeclaredLimitsInsteadOfAssumingZero)
{
  const a1v::DerivativeCheck r =
      a1v::checkDerivativeField({0.5, 9.0}, {1.0, 0.0}, {true, false}, twoJoints());
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.checked, 1u);
  EXPECT_EQ(r.unbounded, 1u);

  // A declared-but-zero limit is unusable, so it is treated as undeclared too.
  const a1v::DerivativeCheck z =
      a1v::checkDerivativeField({0.5}, {0.0}, {true}, {"j1"});
  EXPECT_TRUE(z.ok);
  EXPECT_EQ(z.unbounded, 1u);
  EXPECT_EQ(z.checked, 0u);
}

TEST(A1SampleDerivative, RejectsNonFiniteOversizedAndMismatchedFields)
{
  {
    const a1v::DerivativeCheck r =
        a1v::checkDerivativeField({1.5, 0.0}, {1.0, 1.0}, {true, true}, twoJoints());
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_DERIV_LIMIT_EXCEEDED");
  }
  {
    const a1v::DerivativeCheck r =
        a1v::checkDerivativeField({std::nan(""), 0.0}, {1.0, 1.0}, {true, true}, twoJoints());
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_DERIV_NOT_FINITE");
  }
  {
    // Dimensions must match the joint names; a short field would leave joints
    // unchecked while looking valid.
    const a1v::DerivativeCheck r = a1v::checkDerivativeField({1.0}, {1.0, 1.0}, {true, true},
                                                             twoJoints());
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_DERIV_DIM_MISMATCH");
  }
  {
    const a1v::DerivativeCheck r =
        a1v::checkDerivativeField({}, {1.0}, {true}, {"j1"});
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_DERIV_DIM_MISMATCH");
  }
  {
    // Negative limits are compared by magnitude.
    const a1v::DerivativeCheck r =
        a1v::checkDerivativeField({-1.5}, {-1.0}, {true}, {"j1"});
    EXPECT_FALSE(r.ok);
    EXPECT_EQ(r.reason_code, "E_DERIV_LIMIT_EXCEEDED");
  }
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

// ---------------------------------------------------------------------------
// A1 P3a-2b: the descent schedule and its straightness measure
// ---------------------------------------------------------------------------

TEST(A1DescentSchedule, NonPositiveStepsYieldsAnEmptySchedule)
{
  // An empty schedule must be rejected by the caller rather than being read as
  // "descend in one jump".
  const a1t::TruthPayload rod = rodAt(0.50, 0.0, 0.02, 0.0);
  a1v::GraspGeometry g;
  EXPECT_TRUE(a1v::buildDescentSchedule(rod, g, 0).empty());
  EXPECT_TRUE(a1v::buildDescentSchedule(rod, g, -3).empty());
}

TEST(A1DescentSchedule, IncludesBothEndsAndReproducesTheExistingTargets)
{
  const a1t::TruthPayload rod = rodAt(0.50, 0.0, 0.02, 0.0);
  a1v::GraspGeometry g;
  const auto sched = a1v::buildDescentSchedule(rod, g, 8);
  ASSERT_EQ(sched.size(), 9u);

  const a1v::DualTarget pre = a1v::buildPreGraspTarget(rod, g);
  const a1v::DualTarget grasp = a1v::buildGraspTarget(rod, g);
  // Same generator, same definitions: the schedule's ends ARE the two targets
  // this round already validated.
  EXPECT_NEAR(a1v::poseError(pre.left, sched.front().left).pos_m, 0.0, 1e-12);
  EXPECT_NEAR(a1v::poseError(grasp.left, sched.back().left).pos_m, 0.0, 1e-12);
  EXPECT_NEAR(a1v::poseError(pre.right, sched.front().right).pos_m, 0.0, 1e-12);
  EXPECT_NEAR(a1v::poseError(grasp.right, sched.back().right).pos_m, 0.0, 1e-12);
}

TEST(A1DescentSchedule, OnlyTheHeightChangesAcrossTheSchedule)
{
  const a1t::TruthPayload rod = rodAt(0.48, 0.03, 0.02, 12.0);
  a1v::GraspGeometry g;
  const auto sched = a1v::buildDescentSchedule(rod, g, 10);
  ASSERT_EQ(sched.size(), 11u);
  for (const auto& t : sched) {
    EXPECT_NEAR(t.left.position.x, sched.front().left.position.x, 1e-12);
    EXPECT_NEAR(t.left.position.y, sched.front().left.position.y, 1e-12);
    EXPECT_NEAR(t.right.position.x, sched.front().right.position.x, 1e-12);
    EXPECT_NEAR(t.right.position.y, sched.front().right.position.y, 1e-12);
    EXPECT_NEAR(a1t::rotationDistanceRad(sched.front().left.orientation, t.left.orientation), 0.0,
                1e-12);
    EXPECT_NEAR(a1t::rotationDistanceRad(sched.front().right.orientation, t.right.orientation), 0.0,
                1e-12);
  }
  // Strictly descending, and covering the configured span.
  for (std::size_t i = 1; i < sched.size(); ++i) {
    EXPECT_LT(sched[i].left.position.z, sched[i - 1].left.position.z);
  }
  EXPECT_NEAR(sched.front().left.position.z, g.pre_grasp_height_m, 1e-12);
  EXPECT_NEAR(sched.back().left.position.z, g.grasp_height_m, 1e-12);
  EXPECT_NEAR(sched.front().left.position.z - sched.back().left.position.z,
              g.pre_grasp_height_m - g.grasp_height_m, 1e-12);
}

TEST(A1DescentDeviation, AStraightVerticalLineHasNoDeviation)
{
  const a1t::TruthPayload rod = rodAt(0.50, 0.0, 0.02, 0.0);
  a1v::GraspGeometry g;
  const auto sched = a1v::buildDescentSchedule(rod, g, 12);
  std::vector<a1v::ArmTarget> left;
  for (const auto& t : sched) {
    left.push_back(t.left);
  }
  const a1v::StraightDescentDeviation d = a1v::straightDescentDeviation(left);
  EXPECT_TRUE(d.ok);
  EXPECT_NEAR(d.max_xy_m, 0.0, 1e-12);
  EXPECT_NEAR(d.max_angle_rad, 0.0, 1e-12);
}

TEST(A1DescentDeviation, HorizontalDriftAndRotationAreBothMeasured)
{
  std::vector<a1v::ArmTarget> seq;
  for (int i = 0; i < 5; ++i) {
    a1v::ArmTarget t;
    // 1 mm of horizontal drift per step, so the worst is 4 mm from the first.
    t.position = {0.5 + 0.001 * i, 0.0, 0.28 - 0.01 * i};
    t.orientation = a1v::quatFromRpy(a1v::kPi, 0.0, 0.5 * kDeg * i);
    seq.push_back(t);
  }
  const a1v::StraightDescentDeviation d = a1v::straightDescentDeviation(seq);
  EXPECT_TRUE(d.ok);
  EXPECT_NEAR(d.max_xy_m, 0.004, 1e-9);
  EXPECT_EQ(d.worst_xy_index, 4u);
  EXPECT_NEAR(d.max_angle_rad, 2.0 * kDeg, 1e-9);
  EXPECT_EQ(d.worst_angle_index, 4u);
}

TEST(A1DescentDeviation, EmptyAndNonFiniteSequencesAreRejected)
{
  const a1v::StraightDescentDeviation empty = a1v::straightDescentDeviation({});
  EXPECT_FALSE(empty.ok);
  EXPECT_EQ(empty.reason_code, "E_DESCENT_EMPTY");

  std::vector<a1v::ArmTarget> bad(2);
  bad[0].position = {0.5, 0.0, 0.2};
  bad[0].orientation = a1v::quatFromRpy(a1v::kPi, 0.0, 0.0);
  bad[1].position = {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1};
  bad[1].orientation = bad[0].orientation;
  const a1v::StraightDescentDeviation d = a1v::straightDescentDeviation(bad);
  EXPECT_FALSE(d.ok);
  EXPECT_EQ(d.reason_code, "E_DESCENT_NOT_FINITE");
}

// ---------------------------------------------------------------------------
// A1 P3a-2b: the arrival window must count only NEW feedback
// ---------------------------------------------------------------------------

namespace {

std::vector<std::string> armPair()
{
  return {"mj_left_joint1", "mj_right_joint1"};
}

std::map<std::string, double> heldSample(double v)
{
  return {{"mj_left_joint1", v}, {"mj_right_joint1", v}};
}

}  // namespace

TEST(A1FreshWindow, ReReadingOneSampleNeverFillsTheWindow)
{
  // The defect this replaces: a loop that slept and then called getCurrentState()
  // counted iterations, so one cached state read 30 times looked like 30 samples.
  a1v::FreshWindow w(armPair(), 2, 0.30, 0.002);
  std::string reject;
  ASSERT_TRUE(w.add(1ULL, 0.0, heldSample(0.1), reject));
  for (unsigned long long i = 0; i < 50; ++i) {
    EXPECT_FALSE(w.add(1ULL, 0.01 * static_cast<double>(i), heldSample(0.1), reject));
    EXPECT_EQ(reject, "E_FRESH_STALE_SEQ");
  }
  const a1v::FreshWindow::Verdict v = w.verdict();
  EXPECT_FALSE(v.ready);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.samples, 1u);
  EXPECT_EQ(v.reason_code, "E_FRESH_WINDOW_INCOMPLETE");
}

TEST(A1FreshWindow, NewSamplesOverTheWindowAreAccepted)
{
  a1v::FreshWindow w(armPair(), 2, 0.30, 0.002);
  std::string reject;
  for (int i = 0; i < 8; ++i) {
    ASSERT_TRUE(w.add(static_cast<unsigned long long>(i + 1), 0.05 * i, heldSample(0.1), reject));
  }
  EXPECT_TRUE(w.ready());
  const a1v::FreshWindow::Verdict v = w.verdict();
  EXPECT_TRUE(v.ok);
  EXPECT_EQ(v.reason_code, "E_OK");
  // The window slides: the oldest sample falls out once the span exceeds the
  // window length, so 8 fed samples leave a 0.30 s window of 7.
  EXPECT_EQ(v.samples, 7u);
  EXPECT_NEAR(v.span_sec, 0.30, 1e-12);
  EXPECT_NEAR(v.worst_change_rad, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(v.last_positions.at("mj_left_joint1"), 0.1);
}

TEST(A1FreshWindow, WholeWindowDriftIsJudgedNotAdjacentSteps)
{
  // Each step is 4e-4, a fifth of the 2e-3 tolerance, so an adjacent-step test
  // would pass. The slid window still walks 2.4e-3 and must fail.
  a1v::FreshWindow w(armPair(), 2, 0.30, 0.002);
  std::string reject;
  for (int i = 0; i < 8; ++i) {
    ASSERT_TRUE(w.add(static_cast<unsigned long long>(i + 1), 0.05 * i,
                      heldSample(0.1 + 0.0004 * i), reject));
  }
  const a1v::FreshWindow::Verdict v = w.verdict();
  EXPECT_TRUE(v.ready);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_FRESH_NOT_SETTLED");
  EXPECT_NEAR(v.worst_change_rad, 0.0024, 1e-12);
  EXPECT_EQ(v.worst_joint, "mj_left_joint1");
}

TEST(A1FreshWindow, ASampleMissingAJointIsRejectedAndDoesNotCount)
{
  a1v::FreshWindow w(armPair(), 2, 0.0, 0.002);
  std::string reject;
  ASSERT_TRUE(w.add(1ULL, 0.0, heldSample(0.1), reject));
  std::map<std::string, double> partial {{"mj_left_joint1", 0.1}};
  EXPECT_FALSE(w.add(2ULL, 0.01, partial, reject));
  EXPECT_EQ(reject, "E_FRESH_INCOMPLETE");
  EXPECT_EQ(w.verdict().samples, 1u);

  std::map<std::string, double> non_finite = heldSample(0.1);
  non_finite["mj_right_joint1"] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(w.add(3ULL, 0.02, non_finite, reject));
  EXPECT_EQ(reject, "E_FRESH_INCOMPLETE");
  EXPECT_EQ(w.verdict().samples, 1u);
}

TEST(A1FreshWindow, NoSamplesAtAllIsNotArrival)
{
  a1v::FreshWindow w(armPair(), 2, 0.30, 0.002);
  const a1v::FreshWindow::Verdict v = w.verdict();
  EXPECT_FALSE(v.ready);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_FRESH_NO_SAMPLES");
  EXPECT_EQ(v.samples, 0u);
}
