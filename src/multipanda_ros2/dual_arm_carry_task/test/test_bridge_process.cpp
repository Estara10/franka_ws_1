#include <gtest/gtest.h>
#include <dual_arm_carry_task/bridge_process.hpp>

#include <chrono>
#include <thread>
#include <sys/wait.h>

using namespace std::chrono_literals;

TEST(BridgeProcess, ProtectionKeepsRunningAndFailureStopsAndReapsPublisher)
{
  dual_arm_carry_task::BridgeProcess bridge;
  std::string error;
  ASSERT_TRUE(bridge.start("/bin/sh", {"-c", "exec sleep 30"}, error)) << error;
  int monitor_calls = 0;
  int status = -1;
  bool reaped = false;
  const auto result = dual_arm_carry_task::runProtectedChild(
      bridge,
      [&]() {
        // The production helper invokes the task's physical observer on every
        // cycle while the reference publisher remains live.
        if (bridge.poll() != -1) return true;
        ++monitor_calls;
        return monitor_calls < 4;
      },
      []() { return false; },
      [](int code) { return WIFEXITED(code) && WEXITSTATUS(code) == 0; },
      1s, status, reaped);
  EXPECT_EQ(result, dual_arm_carry_task::ProtectedChildResult::PROTECTION_FAILED);
  EXPECT_EQ(monitor_calls, 4);
  EXPECT_TRUE(reaped);
  ASSERT_NE(status = bridge.poll(), -1);
  EXPECT_TRUE(WIFSIGNALED(status) || WIFEXITED(status));
  EXPECT_FALSE(bridge.start("/bin/true", {}, error));  // no automatic restart
}

TEST(BridgeProcess, NormalExitAndBridgeFailureAreDistinguished)
{
  dual_arm_carry_task::BridgeProcess normal;
  std::string error;
  ASSERT_TRUE(normal.start("/bin/true", {}, error)) << error;
  int status = -1;
  int monitor_calls=0; bool reaped=false;
  auto result=dual_arm_carry_task::runProtectedChild(
      normal,
      [&]() { ++monitor_calls; return true; },
      [&]() { return monitor_calls>=5; },
      [](int code) { return WIFEXITED(code) && WEXITSTATUS(code)==0; },
      1s,status,reaped);
  EXPECT_EQ(result,dual_arm_carry_task::ProtectedChildResult::COMPLETED);
  EXPECT_GE(monitor_calls,5);
  ASSERT_NE(status, -1);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);

  dual_arm_carry_task::BridgeProcess failed;
  ASSERT_TRUE(failed.start("/bin/sh", {"-c", "exit 7"}, error)) << error;
  status = -1; monitor_calls=0;
  result=dual_arm_carry_task::runProtectedChild(
      failed,
      [&]() { ++monitor_calls; return true; },
      []() { return false; },
      [](int code) { return WIFEXITED(code) && WEXITSTATUS(code)==0; },
      1s,status,reaped);
  EXPECT_EQ(result,dual_arm_carry_task::ProtectedChildResult::CHILD_FAILED);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 7);
}
