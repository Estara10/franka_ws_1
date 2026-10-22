#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include "dual_arm_carry_task/mmc_state_validation.hpp"
#include "dual_arm_carry_task/bridge_process.hpp"
#include "dual_arm_carry_task/async_truth_guard.hpp"
#include "dual_arm_carry_task/async_service_query.hpp"
#include "dual_arm_carry_task/mmc_controller_preflight.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <limits>
#include <vector>

using Service = multi_mode_control_msgs::srv::GetRobotStates;
using namespace std::chrono_literals;

namespace
{

// gtest ASSERT_* returns from the test body. A joinable std::thread destroyed
// on that path calls std::terminate, so every test-owned thread is joined by
// these guards even when an assertion fails early.
class ScopedExecutorSpin
{
public:
  explicit ScopedExecutorSpin(rclcpp::executors::SingleThreadedExecutor& executor)
  : executor_(executor), thread_([this] { executor_.spin(); })
  {}
  ~ScopedExecutorSpin()
  {
    executor_.cancel();
    if (thread_.joinable()) thread_.join();
  }

private:
  rclcpp::executors::SingleThreadedExecutor& executor_;
  std::thread thread_;
};

class ScopedThreadJoin
{
public:
  explicit ScopedThreadJoin(std::thread& thread) : thread_(thread) {}
  ~ScopedThreadJoin()
  {
    if (thread_.joinable()) thread_.join();
  }

private:
  std::thread& thread_;
};

franka_msgs::msg::FrankaState validArm(double x)
{
  franka_msgs::msg::FrankaState state;
  state.o_t_ee = {1,0,0,0, 0,1,0,0, 0,0,1,0, x,0,0,1};
  state.q.fill(0.1);
  state.time = 10.0;
  state.header.frame_id = x > 0.0 ? "mj_left_link0" : "mj_right_link0";
  const auto stamp = dual_arm_carry_task::systemWallNowNs();
  state.header.stamp.sec = static_cast<std::int32_t>(stamp / 1000000000LL);
  state.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % 1000000000LL);
  return state;
}

std::vector<franka_msgs::msg::FrankaState> validPair()
{
  return {validArm(0.4), validArm(-0.4)};
}

class MmcStateServiceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

struct JointSampleForCacheTest
{
  unsigned long long seq{0};
  double recv_steady_sec{0.0};
  bool stamp_valid{false};
  double position{0.0};
};

struct LostReplyClient
{
  using Request = Service::Request;
  using SharedResponse = Service::Response::SharedPtr;
  int calls{0};
  int removed{0};
  int reply_on{2};
  std::vector<std::shared_ptr<std::promise<SharedResponse>>> pending;

  std::shared_future<SharedResponse> async_send_request(Request::SharedPtr)
  {
    auto promise = std::make_shared<std::promise<SharedResponse>>();
    auto future = promise->get_future().share();
    pending.push_back(promise);
    if (++calls == reply_on) {
      auto response = std::make_shared<Service::Response>();
      response->states = validPair();
      promise->set_value(response);
    }
    return future;
  }

  void remove_pending_request(const std::shared_future<SharedResponse>&) { ++removed; }
};
}

TEST(MmcStateRequest, LostFirstReplyIsRetriedWithinTheOriginalDeadline)
{
  LostReplyClient client;
  std::size_t attempts = 0;
  const auto response = dual_arm_carry_task::requestServiceUntil(
      client, std::chrono::steady_clock::now() + 250ms, 5ms,
      []() { return true; }, attempts);
  ASSERT_TRUE(response);
  EXPECT_EQ(response->states.size(), 2u);
  EXPECT_EQ(attempts, 2u);
  EXPECT_EQ(client.removed, 1);
}

TEST(MmcStateRequest, MissingRepliesStopAtTotalDeadlineAndRemovePendingRequests)
{
  LostReplyClient client;
  client.reply_on = -1;
  std::size_t attempts = 0;
  const auto began = std::chrono::steady_clock::now();
  EXPECT_FALSE(dual_arm_carry_task::requestServiceUntil(
      client, began + 20ms, 5ms, []() { return true; }, attempts));
  EXPECT_GT(attempts, 0u);
  EXPECT_EQ(client.removed, static_cast<int>(attempts));
  EXPECT_LT(std::chrono::steady_clock::now() - began, 200ms);

  const int previous_calls = client.calls;
  EXPECT_FALSE(dual_arm_carry_task::requestServiceUntil(
      client, std::chrono::steady_clock::now() - 1ms, 5ms,
      []() { return true; }, attempts));
  EXPECT_EQ(client.calls, previous_calls);
}

TEST_F(MmcStateServiceTest, DelayedServiceAppearsAndExecutorProcessesRequest)
{
  auto client_node = std::make_shared<rclcpp::Node>("mmc_service_client_delayed");
  auto server_node = std::make_shared<rclcpp::Node>("mmc_service_server_delayed");
  rclcpp::executors::SingleThreadedExecutor executor;
  auto client = client_node->create_client<Service>("/test_mmc_state_delayed");
  rclcpp::Service<Service>::SharedPtr server;
  // The service is created after the client has started discovery. The node is
  // added to the executor only after it exists, so the executor is never
  // mutated from a second thread while spinning.
  std::thread appear([&] {
    std::this_thread::sleep_for(120ms);
    server = server_node->create_service<Service>("/test_mmc_state_delayed",
      [](const std::shared_ptr<rmw_request_id_t>, const Service::Request::SharedPtr,
         Service::Response::SharedPtr response) { response->states = validPair(); });
  });
  ScopedThreadJoin join_appear(appear);

  const auto deadline = std::chrono::steady_clock::now() + 2000ms;
  const auto discovery = dual_arm_carry_task::waitForServiceUntil(
      deadline, [&]() { return client->service_is_ready(); },
      [](auto next) { std::this_thread::sleep_until(next); });
  appear.join();
  ASSERT_EQ(discovery, dual_arm_carry_task::ServiceDiscoveryResult::READY);
  ASSERT_TRUE(server);

  executor.add_node(client_node);
  executor.add_node(server_node);
  ScopedExecutorSpin spin(executor);
  auto future = client->async_send_request(std::make_shared<Service::Request>());
  ASSERT_EQ(future.wait_for(500ms), std::future_status::ready);
  EXPECT_TRUE(dual_arm_carry_task::validateMmcRobotStates(future.get()->states, 10.0).valid);
}

TEST_F(MmcStateServiceTest, DelayedResponseDoesNotBlockThePollingLoop)
{
  auto client_node = std::make_shared<rclcpp::Node>("mmc_service_client_slow_poll");
  auto server_node = std::make_shared<rclcpp::Node>("mmc_service_server_slow_poll");
  rclcpp::executors::SingleThreadedExecutor executor;
  auto client = client_node->create_client<Service>("/test_mmc_state_slow_poll");
  auto server = server_node->create_service<Service>("/test_mmc_state_slow_poll",
    [](const std::shared_ptr<rmw_request_id_t>, const Service::Request::SharedPtr,
       Service::Response::SharedPtr response) {
      std::this_thread::sleep_for(150ms);
      response->states = validPair();
    });
  executor.add_node(client_node);
  executor.add_node(server_node);
  ScopedExecutorSpin spin(executor);
  ASSERT_TRUE(client->wait_for_service(500ms));

  dual_arm_carry_task::AsyncServiceQuery<rclcpp::Client<Service>> query;
  ASSERT_TRUE(query.start(*client, std::chrono::steady_clock::now()));

  // Simulate the 100 Hz reference loop while the service response is delayed.
  int loop_publish_count = 0;
  double max_poll_ms = 0.0;
  const auto began = std::chrono::steady_clock::now();
  using QueryStatus = dual_arm_carry_task::AsyncServiceStatus;
  bool ready = false;
  while (std::chrono::steady_clock::now() - began < 260ms) {
    const auto poll_started = std::chrono::steady_clock::now();
    const auto status = query.poll(poll_started, 500ms);
    max_poll_ms = std::max(max_poll_ms, std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - poll_started).count());
    if (status == QueryStatus::READY) {
      ready = true;
      break;
    }
    ++loop_publish_count;
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_LT(max_poll_ms, 20.0);
  EXPECT_GE(loop_publish_count, 8);
  EXPECT_TRUE(ready);
  ASSERT_TRUE(query.response());
  EXPECT_TRUE(dual_arm_carry_task::validateMmcRobotStates(
      query.response()->states, 10.0).valid);
}

TEST_F(MmcStateServiceTest, MissingServiceFailsWithinDiscoveryBudget)
{
  auto node = std::make_shared<rclcpp::Node>("mmc_service_client_missing");
  auto client = node->create_client<Service>("/test_mmc_state_missing");
  const auto began = std::chrono::steady_clock::now();
  const auto deadline = began + 120ms;
  const auto discovery = dual_arm_carry_task::waitForServiceUntil(
      deadline, [&]() { return client->service_is_ready(); },
      [](auto next) { std::this_thread::sleep_until(next); });
  EXPECT_EQ(discovery, dual_arm_carry_task::ServiceDiscoveryResult::DEADLINE_EXPIRED);
  EXPECT_FALSE(client->service_is_ready());
  EXPECT_LT(std::chrono::steady_clock::now() - began, 250ms);
}

TEST(MmcControllerPreflight, RawControllerStatesAreReportedAndMismatchIsRejected)
{
  dual_arm_carry_task::ControllerStateObservation observed;
  const auto result = dual_arm_carry_task::evaluateControllerStates(
      {{"dual_panda_arm_controller", "active"}, {"sim_multi_mode_controller", "inactive"}},
      "dual_panda_arm_controller", "sim_multi_mode_controller", true, observed);
  EXPECT_EQ(result, dual_arm_carry_task::ControllerStateResult::UNEXPECTED_STATE);
  EXPECT_EQ(observed.jtc_state, "active");
  EXPECT_EQ(observed.mmc_state, "inactive");
}

TEST(JointFeedbackCache, RejectsBadClockWithoutReplacingOrRefreshingValidSample)
{
  JointSampleForCacheTest cached{7, 100.0, true, 0.42};
  const auto bad = dual_arm_carry_task::acceptJointSampleIfFresh(
      JointSampleForCacheTest{0, 101.0, false, 0.99}, cached,
      1792478357.988610029, 36.684, 101.0);
  EXPECT_FALSE(bad.valid);
  EXPECT_EQ(cached.seq, 7u);
  EXPECT_DOUBLE_EQ(cached.recv_steady_sec, 100.0);
  EXPECT_DOUBLE_EQ(cached.position, 0.42);

  // The cached sample's age continues from its original receipt time; a
  // rejected message cannot renew it.
  EXPECT_FALSE(dual_arm_carry_task::validateJointSampleFreshness(
      36.5, 36.7, 1.0).valid);
}

TEST(JointFeedbackCache, AcceptsSimulationClockAndIncrementsSequence)
{
  JointSampleForCacheTest cached{2, 10.0, true, 0.1};
  const auto result = dual_arm_carry_task::acceptJointSampleIfFresh(
      JointSampleForCacheTest{0, 0.0, false, 0.2}, cached, 36.5, 36.7, 11.0);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(cached.seq, 3u);
  EXPECT_DOUBLE_EQ(cached.recv_steady_sec, 11.0);
  EXPECT_TRUE(cached.stamp_valid);
  EXPECT_DOUBLE_EQ(cached.position, 0.2);
}

TEST(JointFeedbackCache, RejectsZeroStampAsNotYetInitialized)
{
  JointSampleForCacheTest cached{0, 0.0, false, 0.0};
  const auto result = dual_arm_carry_task::acceptJointSampleIfFresh(
      JointSampleForCacheTest{0, 0.0, false, 0.4}, cached, 0.0, 0.0, 1.0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(cached.seq, 0u);
  EXPECT_DOUBLE_EQ(cached.position, 0.0);
}

TEST(MmcControllerPreflight, MissingControllerIsDistinctFromUnexpectedState)
{
  dual_arm_carry_task::ControllerStateObservation observed;
  const auto result = dual_arm_carry_task::evaluateControllerStates(
      {{"dual_panda_arm_controller", "active"}},
      "dual_panda_arm_controller", "sim_multi_mode_controller", false, observed);
  EXPECT_EQ(result, dual_arm_carry_task::ControllerStateResult::MISSING_MMC);
}

TEST_F(MmcStateServiceTest, ResponseTimeoutLatchesFailureAndLateResponseCannotRecover)
{
  auto client_node = std::make_shared<rclcpp::Node>("mmc_service_client_late");
  auto server_node = std::make_shared<rclcpp::Node>("mmc_service_server_late");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  auto client = client_node->create_client<Service>("/test_mmc_state_late");
  auto server = server_node->create_service<Service>("/test_mmc_state_late",
    [](const std::shared_ptr<rmw_request_id_t>, const Service::Request::SharedPtr,
       Service::Response::SharedPtr response) {
      std::this_thread::sleep_for(250ms);
      response->states = validPair();
    });
  executor.add_node(server_node);
  ScopedExecutorSpin spin(executor);
  ASSERT_TRUE(client->wait_for_service(500ms));
  auto future = client->async_send_request(std::make_shared<Service::Request>());
  dual_arm_carry_task::AsyncTruthGuard guard(60ms);
  const auto request_started = dual_arm_carry_task::AsyncTruthGuard::Clock::now();
  guard.seedSample(request_started);
  guard.beginRequest(request_started);
  EXPECT_FALSE(guard.canStartRequest());
  const auto until = request_started + 80ms;
  while (std::chrono::steady_clock::now() < until) {
    if (!guard.observe(dual_arm_carry_task::AsyncTruthGuard::Clock::now())) break;
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_FALSE(guard.observe(dual_arm_carry_task::AsyncTruthGuard::Clock::now()));
  EXPECT_EQ(guard.failure(), "request_timeout");
  std::this_thread::sleep_for(250ms);
  EXPECT_EQ(future.wait_for(0ms), std::future_status::ready);
  EXPECT_FALSE(guard.completeRequest(dual_arm_carry_task::AsyncTruthGuard::Clock::now()));
  EXPECT_TRUE(guard.faulted());
  EXPECT_EQ(guard.failure(), "request_timeout");
}

TEST(MmcStateValidation, RejectsMalformedOrWrongSourceState)
{
  auto pair = validPair();
  EXPECT_TRUE(dual_arm_carry_task::validateMmcRobotStates(pair, 10.0).valid);
  EXPECT_FALSE(dual_arm_carry_task::validateMmcRobotStates({pair.front()}, 10.0).valid);
  pair = validPair(); pair[1].header.frame_id = "mj_left_link0";
  EXPECT_EQ(dual_arm_carry_task::validateMmcRobotStates(pair, 10.0).reason, "source_mismatch_right");
  pair = validPair(); pair[0].q[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(dual_arm_carry_task::validateMmcRobotStates(pair, 10.0).reason, "invalid_non_finite_pose_or_q");
  pair = validPair(); pair[1].o_t_ee[0] = 2.0;
  EXPECT_EQ(dual_arm_carry_task::validateMmcRobotStates(pair, 10.0).reason, "invalid_transform_or_quaternion");
  pair = validPair(); pair[0].header.stamp.sec = 1; pair[0].header.stamp.nanosec = 0;
  EXPECT_EQ(dual_arm_carry_task::validateMmcRobotStates(pair, 10.0).reason, "stale_state_left");
  pair = validPair(); pair[0].time = 0.0;
  EXPECT_EQ(dual_arm_carry_task::validateMmcRobotStates(pair, 10.0).reason, "stale_state_sample_left");
  pair = validPair(); pair[0].time = -1.0;
  EXPECT_EQ(dual_arm_carry_task::validateMmcRobotStates(pair, 10.0).reason, "stale_state_sample_left");
}

TEST(MmcStateValidation, RejectsRecentlyReceivedJointSampleWithOldSimulationStamp)
{
  EXPECT_TRUE(dual_arm_carry_task::validateJointSampleFreshness(10.0, 10.2, 0.01).valid);
  const auto repeated_old_sample = dual_arm_carry_task::validateJointSampleFreshness(8.0, 10.0, 0.01);
  EXPECT_FALSE(repeated_old_sample.valid);
  EXPECT_EQ(repeated_old_sample.reason, "stale_joint_sample_simulation");
  EXPECT_EQ(dual_arm_carry_task::validateJointSampleFreshness(10.2, 10.0, 0.01).reason,
            "stale_joint_sample_simulation");
  EXPECT_EQ(dual_arm_carry_task::validateJointSampleFreshness(10.0, 10.0, 0.30).reason,
            "stale_joint_sample_wall");
}

TEST_F(MmcStateServiceTest, PendingTruthRequestDoesNotBlockProtectionOrChildStop)
{
  auto client_node = std::make_shared<rclcpp::Node>("mmc_service_client_pending");
  auto server_node = std::make_shared<rclcpp::Node>("mmc_service_server_pending");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(client_node);
  auto client = client_node->create_client<Service>("/test_mmc_state_pending");
  auto server = server_node->create_service<Service>("/test_mmc_state_pending",
    [](const std::shared_ptr<rmw_request_id_t>, const Service::Request::SharedPtr,
       Service::Response::SharedPtr response) {
      std::this_thread::sleep_for(700ms);
      response->states = validPair();
    });
  executor.add_node(server_node);
  ScopedExecutorSpin spin(executor);
  ASSERT_TRUE(client->wait_for_service(500ms));
  auto future = client->async_send_request(std::make_shared<Service::Request>());
  dual_arm_carry_task::AsyncTruthGuard truth_guard(150ms);
  const auto request_started=dual_arm_carry_task::AsyncTruthGuard::Clock::now();
  truth_guard.seedSample(request_started);
  truth_guard.beginRequest(request_started);
  dual_arm_carry_task::BridgeProcess child;
  std::string error;
  ASSERT_TRUE(child.start("/bin/sleep", {"10"}, error)) << error;
  std::atomic<int> monitor_calls{0};
  int status = -1;
  bool reaped = false;
  const auto result = dual_arm_carry_task::runProtectedChild(
      child,
      [&] {
        ++monitor_calls;
        if (!truth_guard.observe(dual_arm_carry_task::AsyncTruthGuard::Clock::now())) return false;
        return future.wait_for(0ms) != std::future_status::ready;
      },
      [] { return false; }, [](int) { return true; }, 2s, status, reaped);
  EXPECT_EQ(result, dual_arm_carry_task::ProtectedChildResult::PROTECTION_FAILED);
  EXPECT_GE(monitor_calls.load(), 4);
  EXPECT_TRUE(reaped);
  EXPECT_NE(child.poll(), -1);
  EXPECT_TRUE(truth_guard.faulted());
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(truth_guard.completeRequest(dual_arm_carry_task::AsyncTruthGuard::Clock::now()));
}

TEST_F(MmcStateServiceTest, BoundedRefreshCanReplaceAgedCacheBeforeFreshnessCheck)
{
  using Guard = dual_arm_carry_task::AsyncTruthGuard;
  const auto start = Guard::Clock::now();
  Guard guard(2s);
  guard.seedSample(start);
  const auto refresh_start = start + 2500ms;
  guard.beginRequest(refresh_start);
  EXPECT_TRUE(guard.inFlight());
  EXPECT_FALSE(guard.faulted());
  // The pre-lift path waits for this fresh response without first applying
  // observe() to the deliberately aged cache. Request age remains within the
  // existing two-second budget.
  EXPECT_TRUE(guard.completeRequest(refresh_start + 100ms));
  EXPECT_FALSE(guard.inFlight());
  EXPECT_FALSE(guard.faulted());
  EXPECT_TRUE(guard.observe(refresh_start + 101ms));
  EXPECT_NEAR(guard.sampleAge(refresh_start + 101ms), 0.001, 1e-9);
}
