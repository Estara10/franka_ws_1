#include <gtest/gtest.h>
#include "dual_arm_carry_task/mmc_activation_handshake.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

using dual_arm_carry_task::MmcActivationResult;
using namespace std::chrono_literals;

namespace
{
struct FakeControllerInterface
{
  bool jtc_active{true};
  bool mmc_active{false};
  bool subscriber{false};
  bool switch_ok{true};
  bool create_subscriber_on_activation{true};
  bool controller_state_valid{true};
  bool accept_reference{true};
  bool protection_ok{true};
  bool healthy{true};
  bool publisher_active{true};
  std::uint64_t accepted_sequence{0};
  std::vector<std::string> events;

  bool switchStrict(std::chrono::steady_clock::time_point)
  {
    events.emplace_back("strict_switch_request");
    if (!switch_ok) return false;
    jtc_active = false;
    mmc_active = true;
    // This emulates ControllerRosInterface::start_(): startROSCom() creates
    // the goal subscription only after the controller becomes active.
    subscriber = create_subscriber_on_activation;
    if (subscriber) events.emplace_back("subscription_created_on_activation");
    return true;
  }
  bool controllersMutuallyExclusive(std::chrono::steady_clock::time_point)
  {
    events.emplace_back("actual_controller_state_query");
    return controller_state_valid && mmc_active && !jtc_active;
  }
  bool referenceSequence(std::chrono::steady_clock::time_point, std::uint64_t& sequence)
  {
    events.emplace_back("status_reference_sequence_query");
    sequence = accepted_sequence;
    return true;
  }
  bool protectionHealthy() { events.emplace_back("protection_check"); return protection_ok; }
  bool controllerHealthy() { events.emplace_back("watchdog_check"); return healthy && mmc_active && !jtc_active; }
  bool goalSubscriberReady() { events.emplace_back("subscription_check"); return subscriber; }
  void publishInitialReference()
  {
    events.emplace_back("initial_reference_publish");
    if (publisher_active && mmc_active && !jtc_active && subscriber && accept_reference)
      ++accepted_sequence;
  }
  void waitTick(std::chrono::steady_clock::time_point deadline)
  {
    std::this_thread::sleep_until(std::min(deadline, std::chrono::steady_clock::now() + 1ms));
  }
  void stopPublishing() { publisher_active = false; events.emplace_back("publisher_stopped"); }
};

auto deadlineAfter(std::chrono::milliseconds duration)
{
  return std::chrono::steady_clock::now() + duration;
}
}

TEST(MmcActivationHandshake, InactiveHasNoSubscriberThenActivationCreatesItAndFirstGoalIsAccepted)
{
  FakeControllerInterface fake;
  ASSERT_FALSE(fake.subscriber);
  const auto result = dual_arm_carry_task::activateAndPrimeMmc(fake, deadlineAfter(300ms));
  EXPECT_EQ(result, MmcActivationResult::ACCEPTED);
  EXPECT_FALSE(fake.jtc_active);
  EXPECT_TRUE(fake.mmc_active);
  EXPECT_TRUE(fake.publisher_active);
  EXPECT_EQ(fake.accepted_sequence, 1u);
  const auto switched = std::find(fake.events.begin(), fake.events.end(), "strict_switch_request");
  const auto subscribed = std::find(fake.events.begin(), fake.events.end(), "subscription_created_on_activation");
  const auto sent = std::find(fake.events.begin(), fake.events.end(), "initial_reference_publish");
  ASSERT_NE(switched, fake.events.end());
  ASSERT_NE(subscribed, fake.events.end());
  ASSERT_NE(sent, fake.events.end());
  EXPECT_LT(switched, subscribed);
  EXPECT_LT(subscribed, sent);
}

TEST(MmcActivationHandshake, FailedStrictSwitchDoesNotCheckOrPublishGoal)
{
  FakeControllerInterface fake;
  fake.switch_ok = false;
  EXPECT_EQ(dual_arm_carry_task::activateAndPrimeMmc(fake, deadlineAfter(100ms)),
            MmcActivationResult::SWITCH_FAILED);
  EXPECT_TRUE(fake.jtc_active);
  EXPECT_FALSE(fake.mmc_active);
  EXPECT_EQ(fake.accepted_sequence, 0u);
  EXPECT_FALSE(fake.publisher_active);
}

TEST(MmcActivationHandshake, MissingPostActivationSubscriberFailsWithinSingleBudget)
{
  FakeControllerInterface fake;
  fake.create_subscriber_on_activation = false;
  const auto began = std::chrono::steady_clock::now();
  EXPECT_EQ(dual_arm_carry_task::activateAndPrimeMmc(fake, deadlineAfter(60ms)),
            MmcActivationResult::FIRST_REFERENCE_TIMEOUT);
  EXPECT_LT(std::chrono::steady_clock::now() - began, 200ms);
  EXPECT_EQ(fake.accepted_sequence, 0u);
  EXPECT_FALSE(fake.publisher_active);
}

TEST(MmcActivationHandshake, ProtectionFailureDuringSubscriptionWaitStopsPublisher)
{
  FakeControllerInterface fake;
  fake.create_subscriber_on_activation = false;
  fake.protection_ok = false;
  EXPECT_EQ(dual_arm_carry_task::activateAndPrimeMmc(fake, deadlineAfter(100ms)),
            MmcActivationResult::PROTECTION_FAILED);
  EXPECT_FALSE(fake.publisher_active);
  EXPECT_EQ(fake.accepted_sequence, 0u);
}

TEST(MmcActivationHandshake, RejectsAmbiguousActualControllerStateBeforePublishing)
{
  FakeControllerInterface fake;
  fake.controller_state_valid = false;  // Both active or neither active is rejected.
  EXPECT_EQ(dual_arm_carry_task::activateAndPrimeMmc(fake, deadlineAfter(100ms)),
            MmcActivationResult::CONTROLLER_STATE_INVALID);
  EXPECT_EQ(fake.accepted_sequence, 0u);
  EXPECT_FALSE(fake.publisher_active);
}

TEST(MmcActivationHandshake, SubscriptionAloneIsInsufficientUntilControllerAcceptsReference)
{
  FakeControllerInterface fake;
  fake.accept_reference = false;
  EXPECT_EQ(dual_arm_carry_task::activateAndPrimeMmc(fake, deadlineAfter(60ms)),
            MmcActivationResult::FIRST_REFERENCE_TIMEOUT);
  EXPECT_TRUE(fake.subscriber);
  EXPECT_GT(std::count(fake.events.begin(), fake.events.end(), "initial_reference_publish"), 1);
  EXPECT_EQ(fake.accepted_sequence, 0u);
  EXPECT_FALSE(fake.publisher_active);
}
