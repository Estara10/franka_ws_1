// A1 P3a-2a: gripper action client with reused clients, bounded stages,
// explicit total-time semantics and cancellation. See the header.

#include "dual_arm_carry_task/gripper_client.hpp"

#include <chrono>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <thread>

#include "dual_arm_carry_task/gripper_protocol.hpp"

// One goal call's state, shared between the calling thread and the executor
// callbacks.
//
// The point of this struct is that a goal response may arrive AFTER the call
// that sent it has already given up and returned. The response callback then
// needs somewhere to record what happened and, if the goal was accepted, to ask
// for a cancel. It therefore owns everything it touches: nothing here captures
// a stack reference or the controller, so a late callback operates on a live
// shared_ptr rather than on whatever used to be at that address.
struct GripperGoalContext {
    std::mutex mutex;
    std::string arm;

    // --- set by the goal response callback -------------------------------
    bool response_arrived {false};
    bool accepted {false};
    bool rejected {false};
    rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>::SharedPtr handle;

    // --- caller state -----------------------------------------------------
    /// The call stopped waiting and returned UNKNOWN. A response that arrives
    /// from now on must be cancelled by the callback, not merely recorded.
    bool abandoned {false};

    // --- cancel bookkeeping, kept distinct on purpose ---------------------
    /// A cancel request was actually put on the wire.
    bool cancel_requested {false};
    /// The server answered the cancel request. NOT "the gripper stopped".
    bool cancel_response_received {false};
    /// That answer was accepted for THIS goal.
    bool cancel_accepted {false};
    /// That answer positively said this goal is terminal.
    bool goal_terminal_confirmed {false};
    /// The answer proved nothing (refused, did not name our goal, unknown
    /// code), or no answer arrived. The gate stays shut in that case.
    bool cancel_unconfirmed {false};

    /// Already removed from the unresolved registry.
    bool settled {false};
};

/// Goals whose outcome is still unknown.
///
/// A goal stays here from the moment its call gave up until either the server
/// rejected it or our cancel request was answered. While anything is in here a
/// new open/close is refused, so an unaccounted-for goal can never be joined by
/// a second one.
struct GripperPendingRegistry {
    std::mutex mutex;
    std::map<const GripperGoalContext*, std::shared_ptr<GripperGoalContext>> unresolved;

    /// Requires the context's own lock to be held (locking order: context, then
    /// registry).
    void add(const std::shared_ptr<GripperGoalContext>& ctx) {
        std::lock_guard<std::mutex> guard(mutex);
        unresolved[ctx.get()] = ctx;
    }

    /// Requires the context's own lock to be held.
    void remove(const GripperGoalContext* key) {
        std::lock_guard<std::mutex> guard(mutex);
        unresolved.erase(key);
    }

    int count() {
        std::lock_guard<std::mutex> guard(mutex);
        return static_cast<int>(unresolved.size());
    }
};

namespace {

using namespace std::chrono_literals;

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

/// Requires `ctx->mutex` to be held: drops the context from the unresolved
/// registry, because its outcome is now known.
void settleLocked(const std::shared_ptr<GripperPendingRegistry>& registry,
                  const std::shared_ptr<GripperGoalContext>& ctx)
{
    if (ctx->settled) {
        return;
    }
    ctx->settled = true;
    registry->remove(ctx.get());
}

/// Asks the server to cancel a goal that was accepted only after its call had
/// already returned UNKNOWN.
///
/// `wait_for_response` must be FALSE from an executor callback: blocking there
/// would stall the executor. From the caller's own thread a bounded wait is
/// fine and gives an immediate answer, so it is allowed there.
void requestCancelForLateGoal(
    const rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr& client,
    const std::shared_ptr<GripperGoalContext>& ctx,
    const std::shared_ptr<GripperPendingRegistry>& registry,
    const rclcpp::Logger& logger,
    bool wait_for_response)
{
    const std::string arm = ctx->arm;
    std::array<uint8_t, 16> our_goal_id {};
    {
        std::lock_guard<std::mutex> guard(ctx->mutex);
        if (!ctx->handle) {
            // Nothing to cancel: no goal handle was ever produced.
            ctx->cancel_unconfirmed = true;
            RCLCPP_WARN(logger,
                        "  [夹爪] %s 迟到目标没有可用的目标句柄，无法请求取消；保持门禁关闭",
                        arm.c_str());
            return;
        }
        our_goal_id = ctx->handle->get_goal_id();
    }
    try {
        auto future = client->async_cancel_goal(
            ctx->handle, [ctx, registry, logger, our_goal_id](const auto& response) {
                std::vector<std::array<uint8_t, 16>> canceling;
                int return_code = kCancelErrorRejected;
                if (response) {
                    return_code = response->return_code;
                    for (const auto& info : response->goals_canceling) {
                        canceling.push_back(info.goal_id.uuid);
                    }
                }
                const CancelVerdict verdict =
                    classifyCancelResponse(return_code, our_goal_id, canceling);

                std::lock_guard<std::mutex> guard(ctx->mutex);
                ctx->cancel_response_received = true;
                switch (verdict) {
                    case CancelVerdict::ACCEPTED:
                        // The server took THIS goal over. That is the accepted
                        // outcome, and it is still not a claim that the gripper
                        // has physically stopped.
                        ctx->cancel_accepted = true;
                        RCLCPP_WARN(logger,
                                    "  [夹爪] %s 迟到目标的取消请求已受理（应答中列出了本目标）；"
                                    "受理只说明服务器接管了取消，不声称物理已停止",
                                    ctx->arm.c_str());
                        settleLocked(registry, ctx);
                        return;
                    case CancelVerdict::GOAL_TERMINAL:
                        // Per-goal terminal evidence: the goal is finished or
                        // no longer exists. Nothing can still be running.
                        ctx->goal_terminal_confirmed = true;
                        RCLCPP_WARN(logger,
                                    "  [夹爪] %s 迟到目标已被服务器确认为终态"
                                    "（返回码 %d：已终止或不存在的目标）；解除未知门禁",
                                    ctx->arm.c_str(), return_code);
                        settleLocked(registry, ctx);
                        return;
                    case CancelVerdict::UNCONFIRMED:
                        break;
                }
                // Refused, about some other goal, or an unknown code. None of
                // those show that the old goal has ended, so the goal stays in
                // the registry and the gate stays shut. "Probably finished" is
                // not a reason to let a new command overlap it.
                ctx->cancel_unconfirmed = true;
                RCLCPP_WARN(logger,
                            "  [夹爪] %s 迟到目标的取消未被确认（返回码 %d，应答列出 %zu 个取消中目标，"
                            "本目标不在其中或请求被拒）；无法确认旧目标已结束，保持门禁关闭",
                            ctx->arm.c_str(), return_code, canceling.size());
            });
        if (wait_for_response) {
            if (future.wait_for(1500ms) == std::future_status::ready) {
                // The callback has already recorded the outcome.
                RCLCPP_WARN(logger, "  [夹爪] %s 迟到目标的取消请求已得到应答", arm.c_str());
            } else {
                // Stays in the registry: an unanswered cancel leaves the goal
                // unaccounted for, which keeps the gate shut.
                RCLCPP_WARN(logger,
                            "  [夹爪] %s 迟到目标的取消请求 1.5 s 内未得到应答；"
                            "目标仍记为未知，后续开合继续被拒绝",
                            arm.c_str());
            }
        }
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> guard(ctx->mutex);
        // The request never went out. That says nothing about the goal's state:
        // the usual cause is that the goal already finished, but "probably
        // finished" is exactly the inference that must not open the gate. The
        // goal stays registered.
        ctx->cancel_requested = false;
        ctx->cancel_unconfirmed = true;
        RCLCPP_WARN(logger,
                    "  [夹爪] %s 迟到目标未能发出取消请求: %s；"
                    "无法确认旧目标状态，保持门禁关闭（不移除未知项）",
                    arm.c_str(), e.what());
    }
}

}  // namespace

GripperController::GripperController(rclcpp::Node* node, const GripperClientConfig& config)
    : node_(node), cfg_(config), pending_(std::make_shared<GripperPendingRegistry>())
{
}

bool GripperController::refuseIfUnknownOutstanding(const char* what)
{
    const int outstanding = pending_->count();
    if (outstanding == 0) {
        return false;
    }
    RCLCPP_ERROR(node_->get_logger(),
                 "  ✗ [夹爪] 拒绝本次%s：仍有 %d 个早期目标结果未知（迟到接受尚未解决）；"
                 "不发送任何新目标",
                 what, outstanding);
    return true;
}

rclcpp_action::Client<GripperController::GripperAction>::SharedPtr GripperController::clientFor(
    bool left)
{
    std::lock_guard<std::mutex> guard(client_mutex_);
    auto& slot = left ? left_client_ : right_client_;
    if (!slot) {
        const std::string& action_name = left ? cfg_.left_action : cfg_.right_action;
        slot = rclcpp_action::create_client<GripperAction>(node_, action_name);
        RCLCPP_INFO(node_->get_logger(), "  [夹爪] 创建并复用 %s action 客户端: %s",
                    left ? "左" : "右", action_name.c_str());
    }
    return slot;
}

void GripperController::requestCancel(const GripperGoalHandle::SharedPtr& handle,
                                      ArmOutcome& outcome)
{
    if (!handle) {
        return;
    }
    try {
        auto future = clientFor(outcome.arm == "left")->async_cancel_goal(handle);
        // Recorded only once the request was actually issued: if the goal has
        // already reached a terminal state the client throws before sending
        // anything, and claiming a cancel request then would be untrue.
        outcome.cancel_requested = true;
        if (future.wait_for(1500ms) == std::future_status::ready) {
            const auto response = future.get();
            outcome.cancel_confirmed = true;
            RCLCPP_WARN(node_->get_logger(),
                        "  [夹爪] %s 取消返回码 %d，正在取消的目标数 %zu（确认的是取消请求已被接受，"
                        "不声称物理已停止）",
                        outcome.arm.c_str(), static_cast<int>(response->return_code),
                        response->goals_canceling.size());
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "  [夹爪] %s 取消请求未在 1.5 s 内确认；不声称已停止",
                        outcome.arm.c_str());
        }
    } catch (const std::exception& e) {
        // The usual reason is that the goal already finished, so there is
        // nothing left to cancel. That is not a cancel request, and it is not
        // reported as one.
        outcome.goal_already_finished = true;
        RCLCPP_WARN(node_->get_logger(),
                    "  [夹爪] %s 未发出取消请求（目标已处于终态或客户端不再持有该句柄）: %s",
                    outcome.arm.c_str(), e.what());
    }
}

GripperController::ArmOutcome GripperController::runArm(
    bool left, double position, bool closing, double total_budget_sec,
    std::chrono::steady_clock::time_point begin)
{
    ArmOutcome out;
    out.arm = left ? "left" : "right";

    auto client = clientFor(left);
    const auto server_budget = budgetFrom(cfg_.server_timeout_sec, begin, total_budget_sec);
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
    goal.command.max_effort = cfg_.max_effort;

    // The context outlives this call: if the response only arrives after we have
    // given up, the callback below still has everything it needs.
    auto ctx = std::make_shared<GripperGoalContext>();
    ctx->arm = out.arm;
    const auto registry = pending_;
    const auto client_shared = client;
    const rclcpp::Logger logger = node_->get_logger();

    rclcpp_action::Client<GripperAction>::SendGoalOptions options;
    options.goal_response_callback =
        [ctx, registry, client_shared, logger](GripperGoalHandle::SharedPtr handle) {
            bool must_cancel = false;
            {
                std::lock_guard<std::mutex> guard(ctx->mutex);
                ctx->response_arrived = true;
                if (!handle) {
                    // Rejected: nothing was started, so nothing is outstanding.
                    ctx->rejected = true;
                    settleLocked(registry, ctx);
                    return;
                }
                ctx->accepted = true;
                ctx->handle = handle;
                if (ctx->abandoned && !ctx->cancel_requested) {
                    ctx->cancel_requested = true;
                    must_cancel = true;
                }
            }
            if (!must_cancel) {
                // The caller is still waiting and will deal with this goal.
                return;
            }
            // Accepted after the call had already returned UNKNOWN. Cancel it
            // here, without blocking this executor callback.
            RCLCPP_WARN(logger,
                        "  [夹爪] %s 目标在原调用返回 UNKNOWN 之后才被接受；异步请求取消",
                        ctx->arm.c_str());
            requestCancelForLateGoal(client_shared, ctx, registry, logger,
                                     /*wait_for_response=*/false);
        };

    auto goal_future = client->async_send_goal(goal, options);

    bool late = false;
    const auto goal_budget = budgetFrom(cfg_.goal_timeout_sec, begin, total_budget_sec);
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

        // Mark the context abandoned so a response arriving from now on is
        // cancelled by the callback. Then re-check under the same lock: if the
        // response landed between the wait above and this line, the callback
        // saw abandoned==false and deliberately left the goal alone, so it is
        // this thread that has to cancel it. Without this re-check that goal
        // would be left with nobody responsible for it.
        //
        // `settled` is set exactly when the outcome became known (rejected, or
        // a cancel request answered). Anything not settled is outstanding, so
        // it is registered FIRST and only then cancelled -- if the cancel is
        // never answered the goal stays registered and the gate stays shut.
        bool cancel_here = false;
        {
            std::lock_guard<std::mutex> guard(ctx->mutex);
            ctx->abandoned = true;
            if (!ctx->settled) {
                registry->add(ctx);
                if (ctx->accepted && !ctx->cancel_requested) {
                    ctx->cancel_requested = true;
                    cancel_here = true;
                }
            }
        }
        if (cancel_here) {
            RCLCPP_WARN(logger,
                        "  [夹爪] %s 目标在原调用返回 UNKNOWN 的瞬间被接受；由调用方请求取消",
                        ctx->arm.c_str());
            requestCancelForLateGoal(client_shared, ctx, registry, logger,
                                     /*wait_for_response=*/true);
        }
        return out;
    }
    auto handle = goal_future.get();
    out.goal_accepted = true;

    auto result_future = client->async_get_result(handle);
    bool late_result = false;
    const auto result_budget = budgetFrom(cfg_.result_timeout_sec, begin, total_budget_sec);
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
    // A missing result is NOT a 0 mm gap: 0.0 is a legitimate full-close
    // reading, so an absent value must not be dressed up as one.
    out.achieved_gap_m = result.result ? result.result->position
                                       : std::numeric_limits<double>::quiet_NaN();
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
        RCLCPP_INFO(node_->get_logger(),
                    "  [夹爪] %s %-5s | server=%d accepted=%d rejected=%d result=%d code=%d "
                    "reached_goal=%d stalled=%d result_position=%.4f m effort=%.2f | %s %s",
                    what, o->arm.c_str(), o->server_ready ? 1 : 0, o->goal_accepted ? 1 : 0,
                    o->goal_rejected ? 1 : 0, o->result_received ? 1 : 0, o->result_code,
                    o->reached_goal ? 1 : 0, o->stalled ? 1 : 0, o->achieved_gap_m, o->effort,
                    o->reason_code.c_str(), o->detail.c_str());
        if (o->cancel_requested) {
            RCLCPP_WARN(node_->get_logger(), "  [夹爪] %s 已请求取消，确认=%s", o->arm.c_str(),
                        o->cancel_confirmed ? "是" : "否");
        } else if (o->goal_already_finished) {
            RCLCPP_WARN(node_->get_logger(),
                        "  [夹爪] %s 目标在请求取消前已处于终态，未发出取消请求", o->arm.c_str());
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
        RCLCPP_ERROR(node_->get_logger(), "  ✗ [夹爪] 已有一次开合在进行中，拒绝重叠调用");
        return false;
    }
    struct Release {
        std::atomic<bool>* flag;
        ~Release() { flag->store(false); }
    } release{&in_flight_};

    const auto begin = std::chrono::steady_clock::now();
    const double total = cfg_.total_timeout_sec;
    const bool closing = position < cfg_.close_threshold;

    // An earlier goal whose outcome is still unknown must be dealt with before
    // anything new is sent. Refusing sends nothing at all.
    if (refuseIfUnknownOutstanding(closing ? "闭合" : "打开")) {
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "  [夹爪] 双臂指令 %.4f（单指位移；服务器目标开口 = 2×该值 = %.4f m），"
                "总预算 %.1f s，%s",
                position, 2.0 * position, total, closing ? "闭合" : "打开");

    const ArmOutcome left = runArm(true, position, closing, total, begin);

    ArmOutcome right;
    right.arm = "right";
    if (left.reason_code != "E_OK") {
        // No new command after a failure: the second arm is not driven at all,
        // rather than driven and then ignored.
        right.not_attempted = true;
        right.reason_code = "E_GRIPPER_SKIPPED_AFTER_FAILURE";
        right.detail = "left arm failed; no further command was sent";
    } else if (!a1g::stageWithinBudget(
                   std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(),
                   total)) {
        // Budget already spent: sending another goal would only start a move
        // that cannot be waited for.
        right.not_attempted = true;
        right.reason_code = "E_GRIPPER_SKIPPED_NO_BUDGET";
        right.detail = "total budget exhausted after the left arm; no further command was sent";
    } else {
        right = runArm(false, position, closing, total, begin);
    }
    const bool ok = logOutcomes(closing ? "闭合" : "打开", left, right);

    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    RCLCPP_INFO(node_->get_logger(), "  [夹爪] 双臂总用时 %.3f s（总预算 %.1f s，阶段预算各自独立计入总预算）",
                elapsed, total);
    if (elapsed > total) {
        RCLCPP_ERROR(node_->get_logger(), "  ✗ [夹爪] 总用时超过总预算，判失败");
        return false;
    }
    return ok;
}

bool GripperController::controlDualGrippersContact(double force_per_finger_n)
{
    using ContactAction = franka_msgs::action::Grasp;
    using ContactHandle = rclcpp_action::ClientGoalHandle<ContactAction>;
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> call_guard(call_mutex_, std::try_to_lock);
    bool expected_idle = false;
    if (!call_guard.owns_lock() || !in_flight_.compare_exchange_strong(expected_idle, true)) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[夹爪接触] 有目标仍在执行或先前目标状态未知，拒绝发送闭合");
        return false;
    }
    struct Release { std::atomic<bool>* flag; ~Release() { flag->store(false); } } release{&in_flight_};
    if (contact_goal_unknown_.load()) {
        RCLCPP_ERROR(node_->get_logger(), "[夹爪接触] 先前目标状态未知，拒绝发送闭合");
        return false;
    }

    // The simulator's explicit contact protocol deadline is 12 s. Keep the
    // client result wait bounded at 13.5 s, leaving 1.5 s for the action result
    // to cross DDS after the server's 12 s contact deadline. This does not extend the
    // physical search interval; a server timeout remains a failure.
    constexpr double kContactClientResultBudgetSec = 13.5;
    const double contact_result_budget =
        std::min(cfg_.result_timeout_sec, kContactClientResultBudgetSec);
    if (!(contact_result_budget > 12.0)) {
        RCLCPP_ERROR(node_->get_logger(),
            "[P3A2C_BUDGET] configured client result wait %.2f s must exceed the 12.0 s server contact deadline; no goal sent",
            contact_result_budget);
        return false;
    }
    RCLCPP_INFO(node_->get_logger(),
        "[P3A2C_BUDGET] server_contact_deadline=12.0 s client_result_wait=%.2f s result_margin>=%.2f s",
        contact_result_budget, contact_result_budget - 12.0);

    const auto begin = std::chrono::steady_clock::now();
    auto run_one = [&](bool left) {
        auto& slot = left ? left_contact_client_ : right_contact_client_;
        const std::string arm = left ? "left" : "right";
        const std::string action_name = left ? "/mj_left_gripper_sim_node/grasp"
                                             : "/mj_right_gripper_sim_node/grasp";
        if (!slot) slot = rclcpp_action::create_client<ContactAction>(node_, action_name);
        const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        const double remain = std::max(0.0, cfg_.total_timeout_sec - elapsed);
        if (remain <= 0.0 || !slot->wait_for_action_server(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(std::min(cfg_.server_timeout_sec, remain))))) {
            RCLCPP_ERROR(node_->get_logger(), "[夹爪接触] %s action server unavailable/budget exhausted", arm.c_str());
            return false;
        }
        const double goal_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        const double goal_remain = std::max(0.0, cfg_.total_timeout_sec - goal_elapsed);
        if (goal_remain <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(), "[夹爪接触] %s total budget exhausted before goal", arm.c_str());
            return false;
        }
        ContactAction::Goal goal;
        goal.width = 0.0;  // desired total opening; progressive server command is bounded
        goal.speed = 0.005;  // explicit contact approach, total gap m/s
        goal.force = force_per_finger_n;  // positive: checked simulator tendon force hold, N per finger
        goal.epsilon.inner = 0.005;
        goal.epsilon.outer = 0.005;
        struct ContactGoalState {
            std::mutex mutex;
            bool abandoned {false};
            ContactHandle::SharedPtr handle;
        };
        auto goal_state = std::make_shared<ContactGoalState>();
        rclcpp_action::Client<ContactAction>::SendGoalOptions options;
        options.goal_response_callback = [slot, goal_state, logger=node_->get_logger()](
            ContactHandle::SharedPtr handle) {
            bool cancel_late = false;
            {
                std::lock_guard<std::mutex> lock(goal_state->mutex);
                goal_state->handle = handle;
                cancel_late = goal_state->abandoned && static_cast<bool>(handle);
            }
            if (cancel_late) {
                RCLCPP_ERROR(logger,
                    "[夹爪接触] 迟到接受的接触目标已异步请求取消；状态仍按未知失败处理");
                slot->async_cancel_goal(handle);
            }
        };
        auto goal_future = slot->async_send_goal(goal, options);
        if (goal_future.wait_for(std::chrono::duration<double>(
                std::min(cfg_.goal_timeout_sec, goal_remain))) != std::future_status::ready) {
            ContactHandle::SharedPtr late_handle;
            {
                std::lock_guard<std::mutex> lock(goal_state->mutex);
                goal_state->abandoned = true;
                late_handle = goal_state->handle;
            }
            if (late_handle) slot->async_cancel_goal(late_handle);
            contact_goal_unknown_.store(true);
            RCLCPP_ERROR(node_->get_logger(), "[夹爪接触] %s goal response timeout; outcome unknown", arm.c_str());
            return false;
        }
        auto handle = goal_future.get();
        if (!handle) {
            RCLCPP_ERROR(node_->get_logger(), "[夹爪接触] %s goal rejected", arm.c_str());
            return false;
        }
        auto result_future = slot->async_get_result(handle);
        const auto now_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        const double result_budget = std::max(0.0, std::min(
            contact_result_budget, cfg_.total_timeout_sec - now_elapsed));
        if (result_future.wait_for(std::chrono::duration<double>(result_budget)) !=
            std::future_status::ready) {
            slot->async_cancel_goal(handle);
            contact_goal_unknown_.store(true);
            RCLCPP_ERROR(node_->get_logger(), "[夹爪接触] %s result timeout; cancel requested, task failed",
                         arm.c_str());
            return false;
        }
        const auto wrapped = result_future.get();
        const bool ok = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
                        wrapped.result && wrapped.result->success;
        RCLCPP_INFO(node_->get_logger(),
                    "[夹爪接触] %s action code=%d success=%d result=%s",
                    arm.c_str(), static_cast<int>(wrapped.code), ok ? 1 : 0,
                    wrapped.result ? wrapped.result->error.c_str() : "<missing>");
        return ok;
    };

    // Start both contact approaches together.  A serial close lets the first
    // hand preload and nudge the free object before the second hand reaches it;
    // that creates exactly the intermittent edge contacts this protocol is
    // intended to reject.  Each action still has its own bounded result wait,
    // and both outcomes must be SUCCEEDED before the caller may lift.
    auto left_future = std::async(std::launch::async, [&run_one]() { return run_one(true); });
    auto right_future = std::async(std::launch::async, [&run_one]() { return run_one(false); });
    const bool left_ok = left_future.get();
    const bool right_ok = right_future.get();
    if (!left_ok || !right_ok) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[夹爪接触] bilateral result failed: left=%d right=%d",
                     left_ok ? 1 : 0, right_ok ? 1 : 0);
        return false;
    }
    return true;
}

bool GripperController::controlGripper(const std::string& arm_name, double position)
{
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> call_guard(call_mutex_, std::try_to_lock);
    if (!call_guard.owns_lock() || in_flight_.exchange(true)) {
        RCLCPP_ERROR(node_->get_logger(), "  ✗ [夹爪] 已有一次开合在进行中，拒绝重叠调用");
        return false;
    }
    struct Release {
        std::atomic<bool>* flag;
        ~Release() { flag->store(false); }
    } release{&in_flight_};

    const bool left = arm_name != "mj_right" && arm_name != "right";
    const auto begin = std::chrono::steady_clock::now();
    const double total = cfg_.total_timeout_sec;
    const bool closing = position < cfg_.close_threshold;

    if (refuseIfUnknownOutstanding("单臂指令")) {
        return false;
    }

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
