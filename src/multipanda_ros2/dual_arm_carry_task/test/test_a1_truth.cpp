// A1 P1b offline tests: truth message validation, latch-once freeze,
// frame mapping and planning-scene read-back verification.
//
// These drive the pure logic directly; no ROS, no simulation.

#include "dual_arm_carry_task/a1_truth.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

namespace {

constexpr double kMinSize = 0.005;
constexpr double kMaxSize = 1.0;
const std::string kRunId = "a1_p1b_prepare_01";

a1t::TruthPayload validPayload()
{
  a1t::TruthPayload p;
  p.run_id = kRunId;
  p.frame_id = "world";
  p.position = {0.52, -0.03, 0.020654};
  // yaw = -10 deg, as a unit quaternion
  const double half = (-10.0 * M_PI / 180.0) * 0.5;
  p.orientation = {0.0, 0.0, std::sin(half), std::cos(half)};
  p.size = {0.04, 0.40, 0.04};
  p.mass_kg = 0.64;
  p.has_header_stamp = true;
  return p;
}

}  // namespace

// ---------------------------------------------------------------------------
// Message validation
// ---------------------------------------------------------------------------

TEST(A1TruthValidate, AcceptsAWellFormedPayload)
{
  const a1t::Verdict v = a1t::validateTruth(validPayload(), kRunId, kMinSize, kMaxSize);
  EXPECT_TRUE(v.ok) << v.reason_code << ": " << v.detail;
}

TEST(A1TruthValidate, RejectsRunIdMismatch)
{
  a1t::TruthPayload p = validPayload();
  p.run_id = "a1_some_other_run";
  const a1t::Verdict v = a1t::validateTruth(p, kRunId, kMinSize, kMaxSize);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_TRUTH_RUN_ID_MISMATCH");
}

TEST(A1TruthValidate, RejectsAnUnsetExpectationRatherThanAcceptingAnything)
{
  const a1t::Verdict v = a1t::validateTruth(validPayload(), "", kMinSize, kMaxSize);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_TRUTH_EXPECTED_RUN_ID_EMPTY");
}

TEST(A1TruthValidate, RejectsEmptyFrame)
{
  a1t::TruthPayload p = validPayload();
  p.frame_id.clear();
  const a1t::Verdict v = a1t::validateTruth(p, kRunId, kMinSize, kMaxSize);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_TRUTH_FRAME_EMPTY");
}

TEST(A1TruthValidate, RejectsNonFinitePositionAndSize)
{
  {
    a1t::TruthPayload p = validPayload();
    p.position.z = std::numeric_limits<double>::quiet_NaN();
    const a1t::Verdict v = a1t::validateTruth(p, kRunId, kMinSize, kMaxSize);
    EXPECT_FALSE(v.ok);
    EXPECT_EQ(v.reason_code, "E_TRUTH_NOT_FINITE");
  }
  {
    a1t::TruthPayload p = validPayload();
    p.position.y = std::numeric_limits<double>::infinity();
    EXPECT_EQ(a1t::validateTruth(p, kRunId, kMinSize, kMaxSize).reason_code,
              "E_TRUTH_NOT_FINITE");
  }
  {
    a1t::TruthPayload p = validPayload();
    p.size.y = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(a1t::validateTruth(p, kRunId, kMinSize, kMaxSize).reason_code, "E_TRUTH_BAD_SIZE");
  }
}

TEST(A1TruthValidate, RejectsNonUnitOrDegenerateOrientation)
{
  {
    a1t::TruthPayload p = validPayload();
    p.orientation = {0.0, 0.0, 0.0, 2.0};
    const a1t::Verdict v = a1t::validateTruth(p, kRunId, kMinSize, kMaxSize);
    EXPECT_FALSE(v.ok);
    EXPECT_EQ(v.reason_code, "E_TRUTH_BAD_ORIENTATION");
  }
  {
    a1t::TruthPayload p = validPayload();
    p.orientation = {0.0, 0.0, 0.0, 0.0};
    EXPECT_EQ(a1t::validateTruth(p, kRunId, kMinSize, kMaxSize).reason_code,
              "E_TRUTH_BAD_ORIENTATION");
  }
  {
    a1t::TruthPayload p = validPayload();
    p.orientation.w = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(a1t::validateTruth(p, kRunId, kMinSize, kMaxSize).reason_code,
              "E_TRUTH_BAD_ORIENTATION");
  }
}

TEST(A1TruthValidate, RejectsSizesOutsideTheAcceptedRange)
{
  {
    a1t::TruthPayload p = validPayload();
    p.size = {0.001, 0.40, 0.04};  // too small: a different object
    const a1t::Verdict v = a1t::validateTruth(p, kRunId, kMinSize, kMaxSize);
    EXPECT_FALSE(v.ok);
    EXPECT_EQ(v.reason_code, "E_TRUTH_BAD_SIZE");
  }
  {
    a1t::TruthPayload p = validPayload();
    p.size = {0.04, 5.0, 0.04};  // too large
    EXPECT_EQ(a1t::validateTruth(p, kRunId, kMinSize, kMaxSize).reason_code, "E_TRUTH_BAD_SIZE");
  }
  {
    a1t::TruthPayload p = validPayload();
    p.size = {0.04, 0.0, 0.04};  // degenerate
    EXPECT_EQ(a1t::validateTruth(p, kRunId, kMinSize, kMaxSize).reason_code, "E_TRUTH_BAD_SIZE");
  }
}

// ---------------------------------------------------------------------------
// Latch-once freeze
// ---------------------------------------------------------------------------

TEST(A1TruthLatch, StartsUnfrozenAndFreezesOnTheFirstValidPayload)
{
  a1t::TruthLatch latch;
  EXPECT_FALSE(latch.frozen());

  const a1t::Verdict v = latch.offer(validPayload(), kRunId, kMinSize, kMaxSize);
  EXPECT_TRUE(v.ok) << v.reason_code;
  EXPECT_TRUE(latch.frozen());
  EXPECT_DOUBLE_EQ(latch.value().position.x, 0.52);
  EXPECT_EQ(latch.value().run_id, kRunId);
  EXPECT_EQ(latch.rejected_count(), 0u);
}

TEST(A1TruthLatch, InvalidPayloadDoesNotFreezeTheReference)
{
  a1t::TruthLatch latch;
  a1t::TruthPayload bad = validPayload();
  bad.run_id = "wrong_run";

  const a1t::Verdict v = latch.offer(bad, kRunId, kMinSize, kMaxSize);
  EXPECT_FALSE(v.ok);
  EXPECT_FALSE(latch.frozen());
  EXPECT_EQ(latch.rejected_count(), 1u);
  EXPECT_EQ(latch.last_rejection_code(), "E_TRUTH_RUN_ID_MISMATCH");
}

// The key property: once frozen, a later message must not move the reference.
TEST(A1TruthLatch, MessageArrivingAfterFreezeNeverChangesTheReference)
{
  a1t::TruthLatch latch;
  ASSERT_TRUE(latch.offer(validPayload(), kRunId, kMinSize, kMaxSize).ok);
  const a1t::TruthPayload first = latch.value();

  a1t::TruthPayload later = validPayload();
  later.position = {0.10, 0.10, 0.10};
  later.orientation = {0.0, 0.0, 0.0, 1.0};
  const a1t::Verdict v = latch.offer(later, kRunId, kMinSize, kMaxSize);

  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.reason_code, "E_TRUTH_ALREADY_FROZEN");
  EXPECT_TRUE(latch.frozen());
  EXPECT_DOUBLE_EQ(latch.value().position.x, first.position.x);
  EXPECT_DOUBLE_EQ(latch.value().position.y, first.position.y);
  EXPECT_DOUBLE_EQ(latch.value().position.z, first.position.z);
  EXPECT_DOUBLE_EQ(latch.value().orientation.z, first.orientation.z);
  EXPECT_EQ(latch.rejected_count(), 1u);
}

TEST(A1TruthLatch, RepeatedIdenticalMessagesAfterFreezeAreAlsoRejected)
{
  a1t::TruthLatch latch;
  ASSERT_TRUE(latch.offer(validPayload(), kRunId, kMinSize, kMaxSize).ok);
  ASSERT_FALSE(latch.offer(validPayload(), kRunId, kMinSize, kMaxSize).ok);
  ASSERT_FALSE(latch.offer(validPayload(), kRunId, kMinSize, kMaxSize).ok);
  EXPECT_EQ(latch.rejected_count(), 2u);
}

TEST(A1TruthLatch, MissingMessageLeavesTheLatchUnfrozen)
{
  const a1t::TruthLatch latch;
  EXPECT_FALSE(latch.frozen());
  // The bounded wait for this condition is what the controller times out on.
}

// ---------------------------------------------------------------------------
// Frame mapping
// ---------------------------------------------------------------------------

TEST(A1TruthFrame, SameFrameNeedsNoTransform)
{
  EXPECT_EQ(a1t::resolveFrameMapping("world", "world"), a1t::FrameMapping::SAME_FRAME);
  EXPECT_EQ(a1t::resolveFrameMapping("base_link", "base_link"), a1t::FrameMapping::SAME_FRAME);
}

TEST(A1TruthFrame, WorldAndBaseLinkMapByIdentity)
{
  EXPECT_EQ(a1t::resolveFrameMapping("world", "base_link"),
            a1t::FrameMapping::WORLD_TO_BASE_LINK);
  EXPECT_EQ(a1t::resolveFrameMapping("base_link", "world"),
            a1t::FrameMapping::WORLD_TO_BASE_LINK);
}

TEST(A1TruthFrame, UnknownOrEmptyFramesAreUnsupported)
{
  EXPECT_EQ(a1t::resolveFrameMapping("camera_optical", "base_link"),
            a1t::FrameMapping::UNSUPPORTED);
  EXPECT_EQ(a1t::resolveFrameMapping("", "base_link"), a1t::FrameMapping::UNSUPPORTED);
  EXPECT_EQ(a1t::resolveFrameMapping("world", ""), a1t::FrameMapping::UNSUPPORTED);
}

// ---------------------------------------------------------------------------
// Planning-scene read-back verification
// ---------------------------------------------------------------------------

TEST(A1TruthScene, AcceptsAnExactReadBack)
{
  const a1t::TruthPayload expected = validPayload();
  const a1t::SceneCheck c = a1t::verifySceneReadback(
      expected, expected.position, expected.orientation, expected.size,
      0.001, 0.5 * M_PI / 180.0, 1e-6);
  EXPECT_TRUE(c.ok) << c.reason_code << ": " << c.detail;
  EXPECT_LT(c.pos_err_m, 1e-12);
  EXPECT_LT(c.angle_err_rad, 1e-9);
  EXPECT_LT(c.size_err_m, 1e-12);
}

TEST(A1TruthScene, AcceptsAnEquivalentNegatedQuaternion)
{
  const a1t::TruthPayload expected = validPayload();
  a1t::Quat negated {-expected.orientation.x, -expected.orientation.y,
                     -expected.orientation.z, -expected.orientation.w};
  const a1t::SceneCheck c =
      a1t::verifySceneReadback(expected, expected.position, negated, expected.size,
                               0.001, 0.5 * M_PI / 180.0, 1e-6);
  EXPECT_TRUE(c.ok) << c.reason_code << ": " << c.detail;
}

TEST(A1TruthScene, DetectsPositionMismatchAndReportsTheError)
{
  const a1t::TruthPayload expected = validPayload();
  a1t::Vec3 moved = expected.position;
  moved.x += 0.005;  // 5 mm
  const a1t::SceneCheck c = a1t::verifySceneReadback(expected, moved, expected.orientation,
                                                     expected.size, 0.001, 0.5 * M_PI / 180.0, 1e-6);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "E_SCENE_POS_MISMATCH");
  EXPECT_NEAR(c.pos_err_m, 0.005, 1e-12);
}

TEST(A1TruthScene, DetectsOrientationMismatchAndReportsTheError)
{
  const a1t::TruthPayload expected = validPayload();  // yaw = -10 deg
  // 2 degrees away from the EXPECTED orientation (yaw -8 deg), not from zero.
  const double half = ((-8.0 * M_PI / 180.0)) * 0.5;
  a1t::Quat rotated {0.0, 0.0, std::sin(half), std::cos(half)};
  const a1t::SceneCheck c = a1t::verifySceneReadback(expected, expected.position, rotated,
                                                     expected.size, 0.001, 0.5 * M_PI / 180.0, 1e-6);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "E_SCENE_ANGLE_MISMATCH");
  EXPECT_NEAR(c.angle_err_rad * 180.0 / M_PI, 2.0, 1e-6);
}

TEST(A1TruthScene, DetectsSizeMismatch)
{
  const a1t::TruthPayload expected = validPayload();
  a1t::Vec3 wrong = {0.05, 0.40, 0.04};
  const a1t::SceneCheck c = a1t::verifySceneReadback(expected, expected.position,
                                                     expected.orientation, wrong,
                                                     0.001, 0.5 * M_PI / 180.0, 1e-6);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "E_SCENE_SIZE_MISMATCH");
  EXPECT_NEAR(c.size_err_m, 0.01, 1e-12);
}

TEST(A1TruthScene, RejectsNonFiniteReadBack)
{
  const a1t::TruthPayload expected = validPayload();
  a1t::Vec3 nan_pos = expected.position;
  nan_pos.z = std::numeric_limits<double>::quiet_NaN();
  const a1t::SceneCheck c = a1t::verifySceneReadback(expected, nan_pos, expected.orientation,
                                                     expected.size, 0.001, 0.5 * M_PI / 180.0, 1e-6);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "E_SCENE_READBACK_NOT_FINITE");
}
