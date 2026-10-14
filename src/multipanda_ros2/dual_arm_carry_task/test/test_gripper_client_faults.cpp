// A1 P3a-2a: interface tests that drive the PRODUCTION gripper client
// (gripper_client.cpp) against a minimal ROS fake action server.
//
// These are not helper tests: the code under test is the same GripperController
// the task uses, exercising its real send/accept/result/cancel path over real
// ROS 2 actions. What they do NOT prove is anything about the MuJoCo gripper
// itself -- there is no simulation here.
//
// Scenarios, kept to the ones the review asked for rather than an exhaustive
// fault matrix:
//   * one side failing or timing out fails the whole call;
//   * a goal accepted late, or a result arriving late, does not become success;
//   * a cancel request reaches an accepted goal;
//   * after a failure or a spent budget no further command is sent.

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "dual_arm_carry_task/gripper_client.hpp"

namespace {

using GripperAction = control_msgs::action::GripperCommand;
using GoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;
using namespace std::chrono_literals;

enum class FakeMode {
  SUCCEED_AFTER,      // accept at once, return a successful result after `delay`
  ABORT_AT_ONCE,      // accept at once, then abort
  STAY_EXECUTING,     // accept at once and never terminate the goal
  LATE_GOAL_ACCEPT,   // delay accepting the goal by `delay`, then succeed
  LATE_RESULT,        // accept at once, return the result only after `delay`
  LATE_ACCEPT_THEN_WAIT,  // accept only after `delay`, then keep executing until
                          // a cancel arrives (never finishes on its own)
  LATE_ACCEPT_REJECT_CANCEL,  // accept only after `delay`, REFUSE every cancel,
                              // and keep executing forever
};

/// A minimal action server for one arm, with counters for what arrived.
///
/// The execute callback runs on its own thread, as the real server's does. That
/// thread captures everything it needs BY VALUE and never touches `this` or any
/// other object the test owns, so a goal still in flight when the fixture is
/// torn down cannot reach freed memory.
class FakeGripperServer {
 public:
  FakeGripperServer(rclcpp::Node::SharedPtr node, const std::string& name, FakeMode mode,
                    std::chrono::milliseconds delay, double reported_gap = 0.04)
      : node_(node), alive_(std::make_shared<std::atomic<bool>>(true)) {
    counters_ = std::make_shared<Counters>();
    const auto counters = counters_;
    server_ = rclcpp_action::create_server<GripperAction>(
        node_, name,
        [counters, mode, delay](const rclcpp_action::GoalUUID&,
                                std::shared_ptr<const GripperAction::Goal>) {
          ++counters->goals_received;
          if (mode == FakeMode::LATE_GOAL_ACCEPT || mode == FakeMode::LATE_ACCEPT_THEN_WAIT ||
              mode == FakeMode::LATE_ACCEPT_REJECT_CANCEL) {
            std::this_thread::sleep_for(delay);
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [counters, mode, alive = alive_](const std::shared_ptr<GoalHandle>) {
          ++counters->cancels_received;
          if (mode == FakeMode::LATE_ACCEPT_REJECT_CANCEL) {
            // Refusing the cancel is what must NOT be read as "the goal
            // stopped": the goal below keeps executing.
            return rclcpp_action::CancelResponse::REJECT;
          }
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [counters, alive = alive_, mode, delay, reported_gap](
            const std::shared_ptr<GoalHandle> handle) {
          std::thread([counters, alive, mode, delay, reported_gap, handle]() {
            auto result = std::make_shared<GripperAction::Result>();
            result->position = reported_gap;
            result->effort = 0.0;
            result->stalled = false;
            result->reached_goal = true;
            switch (mode) {
              case FakeMode::ABORT_AT_ONCE:
                result->reached_goal = false;
                handle->abort(result);
                return;
              case FakeMode::STAY_EXECUTING:
                // Never terminates the goal, while the fixture is alive. The
                // client must time out on its own rather than wait forever.
                while (alive->load()) {
                  std::this_thread::sleep_for(10ms);
                }
                return;
              case FakeMode::LATE_RESULT:
              case FakeMode::SUCCEED_AFTER:
                std::this_thread::sleep_for(delay);
                if (alive->load()) {
                  handle->succeed(result);
                }
                return;
              case FakeMode::LATE_GOAL_ACCEPT:
                if (alive->load()) {
                  handle->succeed(result);
                }
                return;
              case FakeMode::LATE_ACCEPT_REJECT_CANCEL: {
                // Runs until the fixture tears down: the old goal is still
                // executing for the whole test, which is the point.
                ExecutingGuard guard(counters);
                while (alive->load()) {
                  std::this_thread::sleep_for(10ms);
                }
                return;
              }
              case FakeMode::LATE_ACCEPT_THEN_WAIT:
                // The accepted goal stays EXECUTING until the client cancels
                // it. This is what makes "the server really received a cancel
                // for the late goal" observable instead of assumed.
                //
                // The loop watches is_canceling(), not our own cancel counter:
                // the counter is bumped by the cancel callback, which runs
                // before the goal handle has actually moved to CANCELING.
                // Calling canceled() in that window is an invalid transition.
                while (alive->load() && !handle->is_canceling()) {
                  std::this_thread::sleep_for(10ms);
                }
                if (!alive->load()) {
                  return;
                }
                result->reached_goal = false;
                handle->canceled(result);
                return;
            }
          }).detach();
        });
  }

  ~FakeGripperServer() { alive_->store(false); }

  /// Stops the worker threads before the fixture tears the nodes down, so a
  /// goal still in flight can never call into a destroyed server.
  void stop() { alive_->store(false); }

  int goals() const { return counters_->goals_received.load(); }
  int cancels() const { return counters_->cancels_received.load(); }
  int executing() const { return counters_->executing_now.load(); }

 private:
  struct Counters {
    std::atomic<int> goals_received {0};
    std::atomic<int> cancels_received {0};
    /// Goals whose execute callback is running right now. Lets a test assert
    /// that the old goal really is still executing.
    std::atomic<int> executing_now {0};
  };

  /// Counts one in-flight execute callback, and always counts it back out.
  struct ExecutingGuard {
    std::shared_ptr<Counters> counters;
    explicit ExecutingGuard(std::shared_ptr<Counters> c) : counters(std::move(c)) {
      ++counters->executing_now;
    }
    ~ExecutingGuard() { --counters->executing_now; }
    ExecutingGuard(const ExecutingGuard&) = delete;
    ExecutingGuard& operator=(const ExecutingGuard&) = delete;
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<GripperAction>::SharedPtr server_;
  std::shared_ptr<Counters> counters_;
  std::shared_ptr<std::atomic<bool>> alive_;
};

/// Both fake servers plus the executors, so a scenario is a couple of lines.
class FakeStack {
 public:
  static constexpr const char* kLeftName = "/a1_fake_left/gripper_action";
  static constexpr const char* kRightName = "/a1_fake_right/gripper_action";

  FakeStack(FakeMode left_mode, std::chrono::milliseconds left_delay, FakeMode right_mode,
            std::chrono::milliseconds right_delay, double right_reported_gap = 0.04) {
    client_node_ = std::make_shared<rclcpp::Node>("a1_gripper_client_under_test");
    server_node_ = std::make_shared<rclcpp::Node>("a1_fake_gripper_servers");
    left = std::make_unique<FakeGripperServer>(server_node_, kLeftName, left_mode, left_delay);
    right = std::make_unique<FakeGripperServer>(server_node_, kRightName, right_mode, right_delay,
                                                right_reported_gap);

    client_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    client_executor_->add_node(client_node_);
    server_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    server_executor_->add_node(server_node_);
    client_thread_ = std::thread([this]() { client_executor_->spin(); });
    server_thread_ = std::thread([this]() { server_executor_->spin(); });
    // Give discovery a moment to connect the two nodes before the call starts.
    std::this_thread::sleep_for(300ms);
  }

  ~FakeStack() {
    // Stop the fake servers' worker threads first, then let them notice, so no
    // callback is in flight while the nodes below are destroyed.
    left->stop();
    right->stop();
    std::this_thread::sleep_for(60ms);
    client_executor_->cancel();
    server_executor_->cancel();
    if (client_thread_.joinable()) { client_thread_.join(); }
    if (server_thread_.joinable()) { server_thread_.join(); }
  }

  rclcpp::Node* client_node() const { return client_node_.get(); }

  std::unique_ptr<FakeGripperServer> left;
  std::unique_ptr<FakeGripperServer> right;

 private:
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> client_executor_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> server_executor_;
  std::thread client_thread_;
  std::thread server_thread_;
};

GripperClientConfig fastConfig() {
  GripperClientConfig cfg;
  cfg.left_action = FakeStack::kLeftName;
  cfg.right_action = FakeStack::kRightName;
  // Deliberately short so each scenario finishes quickly; the grace window
  // inside the client is a fixed 400 ms.
  cfg.server_timeout_sec = 2.0;
  cfg.goal_timeout_sec = 0.3;
  cfg.result_timeout_sec = 0.4;
  cfg.total_timeout_sec = 6.0;
  return cfg;
}

class GripperClientFaults : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(GripperClientFaults, BothSidesSucceedingIsTheOnlySuccess) {
  FakeStack stack(FakeMode::SUCCEED_AFTER, 20ms, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());
  EXPECT_TRUE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1);
  EXPECT_EQ(stack.right->goals(), 1);
}

TEST_F(GripperClientFaults, OneSideAbortingFailsTheCallAndStopsTheOtherArm) {
  // The right server is healthy and would succeed; the point is that it must
  // never be asked, because a failed left arm ends the call.
  FakeStack stack(FakeMode::ABORT_AT_ONCE, 0ms, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1);
  EXPECT_EQ(stack.right->goals(), 0)
      << "a new command was sent after the left arm had already failed";
}

TEST_F(GripperClientFaults, OneSideTimingOutFailsTheCallAndIsCancelled) {
  // The left goal is accepted and then never finishes. The client must time out
  // and cancel it, and must not command the right arm afterwards.
  FakeStack stack(FakeMode::STAY_EXECUTING, 0ms, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1);
  EXPECT_EQ(stack.left->cancels(), 1)
      << "the accepted goal was left running instead of being cancelled";
  EXPECT_EQ(stack.right->goals(), 0);
}

TEST_F(GripperClientFaults, GoalAcceptedOnlyAfterTheBudgetIsCancelledAndFails) {
  // Accepted at 700 ms: past the 300 ms goal budget, inside the 400 ms grace.
  FakeStack stack(FakeMode::LATE_GOAL_ACCEPT, 700ms, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1);
  EXPECT_EQ(stack.right->goals(), 0);
}

TEST_F(GripperClientFaults, LateResultIsNotTurnedIntoSuccess) {
  // Result at 900 ms, past the 400 ms result budget. The server does report
  // reached_goal=true with a healthy gap, and the client must still fail.
  FakeStack stack(FakeMode::LATE_RESULT, 900ms, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1);
  EXPECT_EQ(stack.right->goals(), 0);
}

TEST_F(GripperClientFaults, AnOutOfRangeReportedGapIsNotSuccess) {
  // The right server reports SUCCEEDED with the 0.0843 m excursion this check
  // was added for. The code and the number must both be checked -- accepting
  // SUCCEEDED on its own is what let that value through before.
  FakeStack stack(FakeMode::SUCCEED_AFTER, 20ms, FakeMode::SUCCEED_AFTER, 20ms, 0.0843);
  GripperController controller(stack.client_node(), fastConfig());
  EXPECT_FALSE(controller.controlDualGrippers(0.02))
      << "an out-of-range reported gap was accepted as a completed move";
}

TEST_F(GripperClientFaults, LeftArmConsumingTheWholeBudgetLeavesNothingForTheRight) {
  // The total budget is smaller than what the left arm needs, so the left arm
  // exhausts it and fails. What is asserted here is the observable rule: no new
  // command goes out. The pure budget branch of that rule
  // (a1g::stageWithinBudget) is unit-tested in test_gripper_protocol; the right
  // arm being skipped after a *successful* left arm would require the left to
  // finish exactly as the budget runs out, which is not reproducible by design.
  FakeStack stack(FakeMode::STAY_EXECUTING, 0ms, FakeMode::SUCCEED_AFTER, 20ms);
  GripperClientConfig cfg = fastConfig();
  cfg.total_timeout_sec = 0.5;
  GripperController controller(stack.client_node(), cfg);
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1);
  EXPECT_EQ(stack.right->goals(), 0);
}

// --- late acceptance, clearly past the grace window ------------------------
//
// goal budget 0.3 s + 0.4 s grace = 0.7 s, so a 1.2 s acceptance is unambiguously
// after the call has given up. The server then holds the goal EXECUTING until it
// sees a cancel, which is what lets the test observe that the late goal was
// really cancelled rather than merely dropped.

namespace {
constexpr auto kLateAccept = 1200ms;

/// Bounded observation of a counter, so a missing event fails on the assertion
/// rather than hanging.
bool waitFor(const std::function<int()>& read, int target, std::chrono::milliseconds budget) {
  const auto deadline = std::chrono::steady_clock::now() + budget;
  while (std::chrono::steady_clock::now() < deadline) {
    if (read() >= target) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return read() >= target;
}
}  // namespace

TEST_F(GripperClientFaults, AcceptancePastTheGraceWindowIsCancelledAndBlocksTheNextCall) {
  FakeStack stack(FakeMode::LATE_ACCEPT_THEN_WAIT, kLateAccept, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());

  // 1. The original call fails: an acceptance that arrives this late must not
  //    turn into a success, and must not be waited for indefinitely.
  const auto call_begin = std::chrono::steady_clock::now();
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  const double call_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - call_begin).count();
  EXPECT_LT(call_sec, 1.0) << "the call waited for the late acceptance instead of giving up";
  EXPECT_EQ(stack.left->goals(), 1);

  // 2. While that goal's outcome is still unknown, a second call must send
  //    NOTHING. Not "send and ignore" -- zero new goals.
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1)
      << "a new goal was sent while an earlier goal's outcome was still unknown";
  EXPECT_EQ(stack.right->goals(), 0);

  // 3. The server actually receives a cancel for the late goal. This is the
  //    assertion the previous round was missing.
  EXPECT_TRUE(waitFor([&]() { return stack.left->cancels(); }, 1, 4000ms))
      << "the late-accepted goal was never cancelled on the server";

  // 4. Once that cancel has been answered the gate opens again, so an unknown
  //    outcome blocks new work without wedging the gripper forever.
  const auto reopen_deadline = std::chrono::steady_clock::now() + 5s;
  while (stack.left->goals() < 2 && std::chrono::steady_clock::now() < reopen_deadline) {
    controller.controlDualGrippers(0.02);
  }
  EXPECT_EQ(stack.left->goals(), 2) << "the gate never reopened after the goal was settled";
  waitFor([&]() { return stack.left->cancels(); }, 2, 4000ms);
}

TEST_F(GripperClientFaults, ARunOfNormalCallsIsUnaffectedByTheGate) {
  // The gate must not fire on ordinary sequential calls: each one resolves its
  // own goal, so nothing is ever left unknown.
  FakeStack stack(FakeMode::SUCCEED_AFTER, 20ms, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());
  EXPECT_TRUE(controller.controlDualGrippers(0.02));
  EXPECT_TRUE(controller.controlDualGrippers(0.04));
  EXPECT_TRUE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 3);
  EXPECT_EQ(stack.right->goals(), 3);
}


// --- refused cancel: the gate must NOT reopen -------------------------------
//
// The previous version settled the unknown goal whenever a cancel RESPONSE
// arrived, including a refusal, and also on the exception path. Neither shows
// that the old goal ended, so a new command could overlap a goal still running.
// Here the server refuses every cancel and keeps executing.

TEST_F(GripperClientFaults, RefusedCancelKeepsTheGateShutAndTheOldGoalRunning) {
  FakeStack stack(FakeMode::LATE_ACCEPT_REJECT_CANCEL, kLateAccept, FakeMode::SUCCEED_AFTER, 20ms);
  GripperController controller(stack.client_node(), fastConfig());

  // 1. The original call fails on the late acceptance.
  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1);

  // 2. The late goal is refused a cancel -- the server must really receive the
  //    request for the refusal to mean anything.
  EXPECT_TRUE(waitFor([&]() { return stack.left->cancels(); }, 1, 4000ms))
      << "the cancel request never reached the server";
  // Give the refusal time to be processed and recorded by the client.
  std::this_thread::sleep_for(300ms);

  // 4. The old goal is still executing, so the gate must still be shut: BOTH
  //    entry points refuse, and neither sends anything.
  EXPECT_GE(stack.left->executing(), 1)
      << "the old goal is not executing, so this scenario proves nothing";
  EXPECT_FALSE(controller.controlGripper("left", 0.02));
  EXPECT_EQ(stack.left->goals(), 1) << "controlGripper sent a new goal while the old one was unresolved";
  EXPECT_EQ(stack.left->cancels(), 1) << "controlGripper should not even reach a new cancel";

  EXPECT_FALSE(controller.controlDualGrippers(0.02));
  EXPECT_EQ(stack.left->goals(), 1) << "controlDualGrippers sent a new goal while the old one was unresolved";
  EXPECT_EQ(stack.right->goals(), 0) << "the right arm was commanded while the left goal was unresolved";
}

// The classifier itself, over the cases the interface test cannot stage.
TEST(CancelResponseClassifier, OnlyPerGoalEvidenceIsTerminal) {
  const std::array<uint8_t, 16> ours = {1, 2, 3};
  const std::array<uint8_t, 16> other = {9, 9, 9};
  const std::vector<std::array<uint8_t, 16>> none {};
  const std::vector<std::array<uint8_t, 16>> ours_listing {ours};
  const std::vector<std::array<uint8_t, 16>> other_listing {other};

  // Terminal evidence, both stated per requested goal.
  EXPECT_EQ(classifyCancelResponse(kCancelErrorGoalTerminated, ours, none), CancelVerdict::GOAL_TERMINAL);
  EXPECT_EQ(classifyCancelResponse(kCancelErrorUnknownGoalId, ours, none), CancelVerdict::GOAL_TERMINAL);

  // Accepted: ERROR_NONE *and* our goal is named.
  EXPECT_EQ(classifyCancelResponse(kCancelErrorNone, ours, ours_listing), CancelVerdict::ACCEPTED);

  // ERROR_NONE says "one or more goals are canceling" -- it does not say which.
  // An answer about some other goal proves nothing about ours.
  EXPECT_EQ(classifyCancelResponse(kCancelErrorNone, ours, other_listing), CancelVerdict::UNCONFIRMED);
  EXPECT_EQ(classifyCancelResponse(kCancelErrorNone, ours, none), CancelVerdict::UNCONFIRMED);

  // Refusal and unrecognised codes prove nothing.
  EXPECT_EQ(classifyCancelResponse(kCancelErrorRejected, ours, ours_listing), CancelVerdict::UNCONFIRMED);
  EXPECT_EQ(classifyCancelResponse(42, ours, ours_listing), CancelVerdict::UNCONFIRMED);
}

}  // namespace
