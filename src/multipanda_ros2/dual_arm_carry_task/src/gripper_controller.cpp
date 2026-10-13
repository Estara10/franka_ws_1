// A1 P3a-2a: gripper action client with reused clients, bounded stages,
// explicit total-time semantics and cancellation. See the header.

#include "dual_arm_carry_task/gripper_controller.hpp"

#include <chrono>
#include <future>
#include <thread>

#include "dual_arm_carry_task/gripper_protocol.hpp"

namespace {

using namespace std::chrono_literals;

const char* kLeftAction = "/mj_left_gripper_sim_node/gripper_action";
const char* kRightAction = "/mj_right_gripper_sim_node/gripper_action";

// Waits up to `budget`. Returns true when ready inside the budget; otherwise
// reports whether it became ready during the short grace window.
template <typename FutureT>
bool waitBounded(FutureT& future, std::chrono::steady_clock::duration budget, bool& ready_in_grace)
{
    ready_in_grace = false;
    if (budget <= std::chrono::steady_clock::duration::zero()) {
        return false;
    }
    if (future.wait_for(budget) == std::future_status::ready) {
        return true;
    }
    // A little extra, only so a late response can be recognised and cancelled
    // rather than left dangling. A late response never makes the call succeed.
    if (future.wait_for(400ms) == std::future_status::ready) {
        ready_in_grace = true;
    }
    return false;
}

std::chrono::steady_clock::duration budgetFrom(double seconds,
                                               std::chrono::steady_clock::time_point begin,
                                               double total_budget_sec)
{
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    const double remaining = a1g::remainingBudget(elapsed, total_budget_sec);
    const double use = std::min(seconds, remaining);
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(use));
}

}  // namespace

rclcpp_action::Client<GripperController::GripperAction>::SharedPtr GripperController::clientFor(
    bool left)
{
    std::lock_guard<std::mutex> guard(client_mutex_);
    auto& slot = left ? left_client_ : right_client_;
    if (!slot) {
        slot = rclcpp_action::create_client<GripperAction>(ctx_, left ? kLeftAction : kRightAction);
        RCLCPP_INFO(ctx_->get_logger(), "  [夹爪] 创建并复用 %s action 客户端: %s",
                    left ? "左" : "右", left ? kLeftAction : kRightAction);
    }
    return slot;
}

void GripperController::requestCancel(const GripperGoalHandle::SharedPtr& handle,
                                      ArmOutcome& outcome)
{
    if (!handle) {
        return;
    }
    outcome.cancel_requested = true;
    try {
        auto future = clientFor(outcome.arm == "left")->async_cancel_goal(handle);
        if (future.wait_for(1500ms) == std::future_status::ready) {
            const auto response = future.get();
            outcome.cancel_confirmed = true;
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [夹爪] %s 取消返回码 %d，正在取消的目标数 %zu（确认的是取消请求已被接受，"
                        "不声称物理已停止）",
                        outcome.arm.c_str(), static_cast<int>(response->return_code),
                        response->goals_canceling.size());
        } else {
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [夹爪] %s 取消请求未在 1.5 s 内确认；不声称已停止",
                        outcome.arm.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(ctx_->get_logger(), "  [夹爪] %s 取消异常: %s", outcome.arm.c_str(), e.what());
    }
}

GripperController::ArmOutcome GripperController::runArm(
    bool left, double position, bool closing, double total_budget_sec,
    std::chrono::steady_clock::time_point begin)
{
    ArmOutcome out;
    out.arm = left ? "left" : "right";

    auto client = clientFor(left);
    const auto server_budget = budgetFrom(ctx_->gripper_server_timeout_sec_, begin, total_budget_sec);
    if (!client->wait_for_action_server(
            std::chrono::duration_cast<std::chrono::milliseconds>(server_budget))) {
        out.reason_code = "E_GRIPPER_SERVER_UNAVAILABLE";
        out.detail = "action server not available within the stage budget";
        return out;
    }
    out.server_ready = true;

    GripperAction::Goal goal;
    // command.position is the PER-FINGER displacement; the server derives the
    // target gap as 2 * position.
    goal.command.position = position;
    goal.command.max_effort = ctx_->gripper_max_effort_;

    auto goal_future = client->async_send_goal(goal);
    bool late = false;
    const auto goal_budget = budgetFrom(ctx_->gripper_goal_timeout_sec_, begin, total_budget_sec);
    const bool ready = waitBounded(goal_future, goal_budget, late);
    const bool handle_valid = (ready || late) && goal_future.valid() &&
                              goal_future.wait_for(0ms) == std::future_status::ready &&
                              goal_future.get() != nullptr;

    const a1g::LateAcceptance decision = a1g::decideLateAcceptance(ready, late, handle_valid);
    if (decision == a1g::LateAcceptance::REJECTED) {
        out.goal_rejected = true;
        out.reason_code = "E_GRIPPER_GOAL_REJECTED";
        out.detail = "goal rejected by the server";
        return out;
    }
    if (decision == a1g::LateAcceptance::CANCEL_LATE) {
        auto handle = goal_future.get();
        out.goal_accepted = true;
        requestCancel(handle, out);
        out.reason_code = "E_GRIPPER_GOAL_ACCEPTED_LATE";
        out.detail = "goal accepted only after the stage budget expired; cancelled, call fails";
        return out;
    }
    if (decision == a1g::LateAcceptance::UNKNOWN_STOP) {
        out.late_goal_unknown = true;
        out.reason_code = "E_GRIPPER_GOAL_UNKNOWN";
        out.detail = "no goal response within budget or grace; acceptance state unknown";
        return out;
    }
    auto handle = goal_future.get();
    out.goal_accepted = true;

    auto result_future = client->async_get_result(handle);
    bool late_result = false;
    const auto result_budget = budgetFrom(ctx_->gripper_result_timeout_sec_, begin, total_budget_sec);
    if (!waitBounded(result_future, result_budget, late_result)) {
        requestCancel(handle, out);
        out.reason_code = late_result ? "E_GRIPPER_RESULT_LATE" : "E_GRIPPER_RESULT_TIMEOUT";
        out.detail = late_result ? "result arrived only after the stage budget expired"
                                 : "no result within the stage budget";
        return out;
    }
    const auto result = result_future.get();
    out.result_received = true;
    out.result_code = static_cast<int>(result.code);
    out.achieved_gap_m = result.result ? result.result->position : 0.0;
    out.effort = result.result ? result.result->effort : 0.0;
    out.stalled = result.result ? result.result->stalled : false;
    out.reached_goal = result.result ? result.result->reached_goal : false;

    const a1g::ResultClassification cls = a1g::classifyResult(
        a1g::actionCodeFromInt(out.result_code), closing, out.reached_goal, out.achieved_gap_m,
        2.0 * position);
    out.reason_code = cls.reason_code;
    out.detail = cls.detail;
    if (!cls.ok) {
        requestCancel(handle, out);
    }
    return out;
}

bool GripperController::logOutcomes(const char* what, const ArmOutcome& left,
                                    const ArmOutcome& right) const
{
    const bool ok = left.reason_code == "E_OK" && right.reason_code == "E_OK";
    for (const ArmOutcome* o : {&left, &right}) {
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [夹爪] %s %-5s | server=%d accepted=%d rejected=%d result=%d code=%d "
                    "reached_goal=%d stalled=%d gap=%.4f m effort=%.2f | %s %s",
                    what, o->arm.c_str(), o->server_ready ? 1 : 0, o->goal_accepted ? 1 : 0,
                    o->goal_rejected ? 1 : 0, o->result_received ? 1 : 0, o->result_code,
                    o->reached_goal ? 1 : 0, o->stalled ? 1 : 0, o->achieved_gap_m, o->effort,
                    o->reason_code.c_str(), o->detail.c_str());
        if (o->cancel_requested) {
            RCLCPP_WARN(ctx_->get_logger(), "  [夹爪] %s 已请求取消，确认=%s", o->arm.c_str(),
                        o->cancel_confirmed ? "是" : "否");
        }
    }
    return ok;
}

bool GripperController::controlDualGrippers(double position)
{
    using namespace std::chrono_literals;

    // One open/close at a time: a simple rejection, not a queue.
    std::unique_lock<std::mutex> call_guard(call_mutex_, std::try_to_lock);
    if (!call_guard.owns_lock() || in_flight_.exchange(true)) {
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [夹爪] 已有一次开合在进行中，拒绝重叠调用");
        return false;
    }
    struct Release {
        std::atomic<bool>* flag;
        ~Release() { flag->store(false); }
    } release{&in_flight_};

    const auto begin = std::chrono::steady_clock::now();
    const double total = ctx_->gripper_total_timeout_sec_;
    const bool closing = position < ctx_->gripper_close_threshold_;
    RCLCPP_INFO(ctx_->get_logger(),
                "  [夹爪] 双臂指令 %.4f（单指位移；服务器目标开口 = 2×该值 = %.4f m），"
                "总预算 %.1f s，%s",
                position, 2.0 * position, total, closing ? "闭合" : "打开");

    const ArmOutcome left = runArm(true, position, closing, total, begin);
    const ArmOutcome right = runArm(false, position, closing, total, begin);
    const bool ok = logOutcomes(closing ? "闭合" : "打开", left, right);

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    RCLCPP_INFO(ctx_->get_logger(), "  [夹爪] 双臂总用时 %.3f s（总预算 %.1f s，阶段预算各自独立计入总预算）",
                elapsed, total);
    if (elapsed > total) {
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [夹爪] 总用时超过总预算，判失败");
        return false;
    }
    return ok;
}

bool GripperController::controlGripper(const std::string& arm_name, double position)
{
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> call_guard(call_mutex_, std::try_to_lock);
    if (!call_guard.owns_lock() || in_flight_.exchange(true)) {
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [夹爪] 已有一次开合在进行中，拒绝重叠调用");
        return false;
    }
    struct Release {
        std::atomic<bool>* flag;
        ~Release() { flag->store(false); }
    } release{&in_flight_};

    const bool left = arm_name != "mj_right" && arm_name != "right";
    const auto begin = std::chrono::steady_clock::now();
    const double total = ctx_->gripper_total_timeout_sec_;
    const bool closing = position < ctx_->gripper_close_threshold_;

    const ArmOutcome a = runArm(left, position, closing, total, begin);
    const ArmOutcome unused;
    const bool ok = a.reason_code == "E_OK";
    ArmOutcome other = unused;
    other.arm = left ? "right" : "left";
    other.reason_code = "E_NOT_INVOLVED";
    other.detail = "single-arm call";
    if (left) {
        logOutcomes("单臂", a, other);
    } else {
        logOutcomes("单臂", other, a);
    }
    return ok;
}
