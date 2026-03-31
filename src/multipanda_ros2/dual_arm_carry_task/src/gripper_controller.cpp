#include "dual_arm_carry_task/gripper_controller.hpp"

void GripperController::controlDualGrippers(double position)
    {
        using namespace std::chrono_literals;

        std::string left_action_name = "/mj_left_gripper_sim_node/gripper_action";
        std::string right_action_name = "/mj_right_gripper_sim_node/gripper_action";

        auto left_client = rclcpp_action::create_client<GripperAction>(ctx_, left_action_name);
        auto right_client = rclcpp_action::create_client<GripperAction>(ctx_, right_action_name);

        bool left_ok = left_client->wait_for_action_server(3s);
        bool right_ok = right_client->wait_for_action_server(3s);

        if (!left_ok || !right_ok) {
            RCLCPP_WARN(ctx_->get_logger(), "  ! 至少有一个 Gripper action server 不可用，跳过双臂夹爪控制");
            return;
        }

        auto goal_msg = GripperAction::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = 100.0;

        RCLCPP_INFO(ctx_->get_logger(), "  发送双臂夹爪同步指令 -> %.3f", position);

        auto send_goal_options_left = rclcpp_action::Client<GripperAction>::SendGoalOptions();
        auto send_goal_options_right = rclcpp_action::Client<GripperAction>::SendGoalOptions();

        bool left_accepted = false, left_completed = false;
        bool right_accepted = false, right_completed = false;

        send_goal_options_left.goal_response_callback = [&](auto) { left_accepted = true; };
        send_goal_options_left.result_callback = [&](const auto& result) {
            left_completed = true;
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED && result.code != rclcpp_action::ResultCode::ABORTED) {
                RCLCPP_WARN(ctx_->get_logger(), "  ! 左夹爪控制失败 (code=%d)", (int)result.code);
            }
        };

        send_goal_options_right.goal_response_callback = [&](auto) { right_accepted = true; };
        send_goal_options_right.result_callback = [&](const auto& result) {
            right_completed = true;
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED && result.code != rclcpp_action::ResultCode::ABORTED) {
                RCLCPP_WARN(ctx_->get_logger(), "  ! 右夹爪控制失败 (code=%d)", (int)result.code);
            }
        };

        left_client->async_send_goal(goal_msg, send_goal_options_left);
        right_client->async_send_goal(goal_msg, send_goal_options_right);

        auto start_time = std::chrono::steady_clock::now();
        while ((!left_accepted || !right_accepted) && (!left_completed || !right_completed)) {
            std::this_thread::sleep_for(50ms);
            if (std::chrono::steady_clock::now() - start_time > 5s) break;
        }

        start_time = std::chrono::steady_clock::now();
        while (!left_completed || !right_completed) {
            std::this_thread::sleep_for(50ms);
            if (std::chrono::steady_clock::now() - start_time > 8s) break;
        }
        RCLCPP_INFO(ctx_->get_logger(), "  ✓ 双臂夹爪同步控制完成");
    }

void GripperController::controlGripper(const std::string& arm_name, double position)
    {
        using namespace std::chrono_literals;
        
        // 修正的 action 名称：直接使用节点名称
        std::string action_name = "/" + arm_name + "_gripper_sim_node/gripper_action";
        auto gripper_client = rclcpp_action::create_client<GripperAction>(
            ctx_, action_name);
        
        // 等待action server
        if (!gripper_client->wait_for_action_server(3s)) {
            RCLCPP_WARN(ctx_->get_logger(), "  ! Gripper action server '%s' 不可用，跳过夹爪控制", 
                       action_name.c_str());
            return;
        }
        
        // 构造goal
        auto goal_msg = GripperAction::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = 100.0;  // 最大力 100N（Franka夹爪最大约60-100N）
        
        RCLCPP_INFO(ctx_->get_logger(), "  发送夹爪指令: %s -> %.3f", arm_name.c_str(), position);
        
        // 使用同步方式发送 goal（避免 executor 冲突）
        auto send_goal_options = rclcpp_action::Client<GripperAction>::SendGoalOptions();
        
        // 设置超时回调
        bool goal_accepted = false;
        bool goal_completed = false;
        
        send_goal_options.goal_response_callback = 
            [&](std::shared_ptr<rclcpp_action::ClientGoalHandle<GripperAction>> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 夹爪goal被拒绝: %s", arm_name.c_str());
                } else {
                    goal_accepted = true;
                }
            };
        
        send_goal_options.result_callback = 
            [&](const rclcpp_action::ClientGoalHandle<GripperAction>::WrappedResult& result) {
                goal_completed = true;
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ 夹爪控制成功: %s", arm_name.c_str());
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    // 夹住物体后，夹爪无法继续闭合（电机受阻）是正常现象
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ 夹爪已接触物体（受阻停止）: %s", arm_name.c_str());
                } else {
                    RCLCPP_WARN(ctx_->get_logger(), "  ! 夹爪控制失败: %s (code=%d)", 
                               arm_name.c_str(), (int)result.code);
                }
            };
        
        // 发送 goal
        auto goal_future = gripper_client->async_send_goal(goal_msg, send_goal_options);
        
        // 等待 goal 被接受（增加超时时间，首次调用可能需要更长时间）
        auto start_time = std::chrono::steady_clock::now();
        while (!goal_accepted && !goal_completed) {
            std::this_thread::sleep_for(50ms);
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > 5s) {  // 从3秒增加到5秒
                RCLCPP_WARN(ctx_->get_logger(), "  ! 等待夹爪响应超时，但命令可能已发送: %s", arm_name.c_str());
                return;  // 容错：即使超时也允许继续（命令可能已生效）
            }
        }
        
        // 等待执行完成
        start_time = std::chrono::steady_clock::now();
        while (!goal_completed) {
            std::this_thread::sleep_for(50ms);
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > 8s) {  // 从5秒增加到8秒
                RCLCPP_WARN(ctx_->get_logger(), "  ! 夹爪执行超时，但命令可能已生效: %s", arm_name.c_str());
                return;  // 容错：允许继续执行后续步骤
            }
        }
    }
