#include "dual_arm_carry_task/gripper_controller.hpp"

#include <future>

namespace {

bool isSuccessfulGripperResult(rclcpp_action::ResultCode code, bool closing) {
  if (code == rclcpp_action::ResultCode::SUCCEEDED) {
    return true;
  }

  // During closing, the simulated Franka gripper may report ABORTED when it
  // hits the object and stops before reaching the commanded finger position.
  return closing && code == rclcpp_action::ResultCode::ABORTED;
}

}  // namespace

bool GripperController::controlDualGrippers(double position) {
  using namespace std::chrono_literals;
  try {
    std::string left_action_name = "/mj_left_gripper_sim_node/gripper_action";
    std::string right_action_name = "/mj_right_gripper_sim_node/gripper_action";

    auto left_client = rclcpp_action::create_client<GripperAction>(ctx_, left_action_name);
    auto right_client = rclcpp_action::create_client<GripperAction>(ctx_, right_action_name);

    bool left_ok = left_client->wait_for_action_server(3s);
    bool right_ok = right_client->wait_for_action_server(3s);

    if (!left_ok || !right_ok) {
      RCLCPP_ERROR(ctx_->get_logger(),
                   "  ✗ 至少有一个 Gripper action server 不可用，双臂夹爪控制失败");
      return false;
    }

    auto goal_msg = GripperAction::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 100.0;

    RCLCPP_INFO(ctx_->get_logger(), "  发送双臂夹爪同步指令 -> %.3f", position);

    auto left_goal_future = left_client->async_send_goal(goal_msg);
    auto right_goal_future = right_client->async_send_goal(goal_msg);

    if (left_goal_future.wait_for(5s) != std::future_status::ready ||
        right_goal_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 等待双臂夹爪 goal 响应超时");
      return false;
    }

    auto left_goal_handle = left_goal_future.get();
    auto right_goal_handle = right_goal_future.get();
    if (!left_goal_handle || !right_goal_handle) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 双臂夹爪 goal 被拒绝 (left=%s right=%s)",
                   left_goal_handle ? "accepted" : "rejected",
                   right_goal_handle ? "accepted" : "rejected");
      return false;
    }

    auto left_result_future = left_client->async_get_result(left_goal_handle);
    auto right_result_future = right_client->async_get_result(right_goal_handle);

    if (left_result_future.wait_for(8s) != std::future_status::ready ||
        right_result_future.wait_for(8s) != std::future_status::ready) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 等待双臂夹爪执行结果超时");
      return false;
    }

    const bool closing = position < 0.03;
    const auto left_result = left_result_future.get();
    const auto right_result = right_result_future.get();
    const bool left_success = isSuccessfulGripperResult(left_result.code, closing);
    const bool right_success = isSuccessfulGripperResult(right_result.code, closing);

    if (!left_success || !right_success) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 双臂夹爪控制失败 (left_code=%d right_code=%d)",
                   static_cast<int>(left_result.code), static_cast<int>(right_result.code));
      return false;
    }

    RCLCPP_INFO(ctx_->get_logger(), "  ✓ 双臂夹爪同步控制完成");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 双臂夹爪控制异常: %s", e.what());
    return false;
  }
}

bool GripperController::controlGripper(const std::string& arm_name, double position) {
  using namespace std::chrono_literals;
  try {
    // 修正的 action 名称：直接使用节点名称
    std::string action_name = "/" + arm_name + "_gripper_sim_node/gripper_action";
    auto gripper_client = rclcpp_action::create_client<GripperAction>(ctx_, action_name);

    // 等待action server
    if (!gripper_client->wait_for_action_server(3s)) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ Gripper action server '%s' 不可用，夹爪控制失败",
                   action_name.c_str());
      return false;
    }

    // 构造goal
    auto goal_msg = GripperAction::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 100.0;  // 最大力 100N（Franka夹爪最大约60-100N）

    RCLCPP_INFO(ctx_->get_logger(), "  发送夹爪指令: %s -> %.3f", arm_name.c_str(), position);

    auto goal_future = gripper_client->async_send_goal(goal_msg);
    if (goal_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 等待夹爪 goal 响应超时: %s", arm_name.c_str());
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 夹爪 goal 被拒绝: %s", arm_name.c_str());
      return false;
    }

    auto result_future = gripper_client->async_get_result(goal_handle);
    if (result_future.wait_for(8s) != std::future_status::ready) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 等待夹爪执行结果超时: %s", arm_name.c_str());
      return false;
    }

    const bool closing = position < 0.03;
    const auto result = result_future.get();
    if (!isSuccessfulGripperResult(result.code, closing)) {
      RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 夹爪控制失败: %s (code=%d)", arm_name.c_str(),
                   static_cast<int>(result.code));
      return false;
    }

    RCLCPP_INFO(ctx_->get_logger(), "  ✓ 夹爪控制成功: %s", arm_name.c_str());
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 夹爪控制异常: %s", e.what());
    return false;
  }
}
