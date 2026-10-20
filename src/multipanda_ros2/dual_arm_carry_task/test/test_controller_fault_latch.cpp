// Copyright (c) 2026 The Franka Dual Arm Project
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "dual_arm_carry_task/controller_fault_latch.hpp"

using dual_arm_carry_task::ControllerFaultLatch;

TEST(ControllerFaultLatch, HealthyFreshActiveStatusPasses)
{
  ControllerFaultLatch latch;
  EXPECT_TRUE(latch.observe(true, true, true, false, "", 0, 0, 1000, 1001));
}

TEST(ControllerFaultLatch, WatchdogFaultLatchesAndCannotBecomeHealthyAgain)
{
  ControllerFaultLatch latch;
  EXPECT_FALSE(latch.observe(true, true, true, true, "reference_watchdog", 1, 900, 1000, 1001));
  EXPECT_EQ(latch.failure(), "controller fault latched: reference_watchdog sequence=1 time_ns=900");
  EXPECT_FALSE(latch.observe(true, true, true, false, "", 0, 0, 1002, 1003));
}

TEST(ControllerFaultLatch, MissingOrStaleStatusFailsClosed)
{
  ControllerFaultLatch missing;
  EXPECT_FALSE(missing.observe(false, false, false, false, "", 0, 0, 0, 1000));

  ControllerFaultLatch stale;
  EXPECT_FALSE(stale.observe(true, true, true, false, "", 0, 0,
                             1000, 1000 + ControllerFaultLatch::kMaxStatusAgeNs + 1));
}

TEST(ControllerFaultLatch, ControllerManagerDeactivationFailsClosed)
{
  ControllerFaultLatch latch;
  EXPECT_FALSE(latch.observe(true, true, false, false, "", 0, 0, 1000, 1001));
  EXPECT_EQ(latch.failure(), "MMC is not active");
}

TEST(ControllerFaultLatch, SingleQueryMissWithinFreshnessBudgetDoesNotLatch)
{
  ControllerFaultLatch latch;
  const std::uint64_t good = 1000000000ULL;
  ASSERT_TRUE(latch.observe(true, true, true, false, "", 0, 0, good, good));
  // A response that is merely late or missing is not a controller fault while
  // a recent complete observation is still fresh.
  EXPECT_TRUE(latch.observeQueryMiss(good + 100000000ULL));  // +100 ms
  EXPECT_TRUE(latch.observeQueryMiss(good + 200000000ULL));  // +200 ms
  EXPECT_TRUE(latch.healthy());
  EXPECT_EQ(latch.failure(), "");
}

TEST(ControllerFaultLatch, SustainedQueryLossLatchesAtFreshnessBudget)
{
  ControllerFaultLatch latch;
  const std::uint64_t good = 1000000000ULL;
  ASSERT_TRUE(latch.observe(true, true, true, false, "", 0, 0, good, good));
  EXPECT_TRUE(latch.observeQueryMiss(good + 100000000ULL));
  EXPECT_FALSE(latch.observeQueryMiss(good + ControllerFaultLatch::kMaxStatusAgeNs + 1));
  EXPECT_NE(latch.failure().find("freshness budget"), std::string::npos);
  // The latch is permanent.
  EXPECT_FALSE(latch.observeQueryMiss(good + 100000000ULL));
  EXPECT_FALSE(latch.healthy());
}

TEST(ControllerFaultLatch, FreshObservationAfterMissExtendsTheBudget)
{
  ControllerFaultLatch latch;
  const std::uint64_t good = 1000000000ULL;
  ASSERT_TRUE(latch.observe(true, true, true, false, "", 0, 0, good, good));
  EXPECT_TRUE(latch.observeQueryMiss(good + 200000000ULL));
  const std::uint64_t refreshed = good + 220000000ULL;
  ASSERT_TRUE(latch.observe(true, true, true, false, "", 0, 0, refreshed, refreshed));
  EXPECT_TRUE(latch.observeQueryMiss(refreshed + 200000000ULL));
  EXPECT_TRUE(latch.healthy());
}

TEST(ControllerFaultLatch, ExplicitLongerBudgetToleratesAnArmedWait)
{
  ControllerFaultLatch latch;
  const std::uint64_t good = 1000000000ULL;
  ASSERT_TRUE(latch.observe(true, true, true, false, "", 0, 0, good, good));
  // While the bridge is armed before the lift, a 2 s budget is allowed; the
  // HOLD path still uses the default 250 ms budget.
  EXPECT_TRUE(latch.observeQueryMiss(good + 1000000000ULL, 2000000000ULL));
  EXPECT_TRUE(latch.observeQueryMiss(good + 1900000000ULL, 2000000000ULL));
  EXPECT_FALSE(latch.observeQueryMiss(good + 2000000001ULL, 2000000000ULL));
  EXPECT_NE(latch.failure().find("2000 ms freshness budget"), std::string::npos);
}

TEST(ControllerFaultLatch, ControllerStateConflictLatchesImmediately)
{
  ControllerFaultLatch latch;
  EXPECT_FALSE(latch.latchControllerStateConflict(
      "expected MMC=active,JTC=inactive but got MMC=inactive,JTC=active"));
  EXPECT_NE(latch.failure().find("controller state conflict"), std::string::npos);
  EXPECT_FALSE(latch.latchControllerStateConflict("anything"));
}
