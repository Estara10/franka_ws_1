import re

cpp_file = "src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp"
with open(cpp_file, 'r') as f:
    orig = f.read()

# find executeRetreat
hook = r"void executeRetreat\(\)\s*\{"
replacement = """void executeRetreat()
    {
        // === [FIX: RETREAT Collision] ===
        // Force fully disable collision checking between left link 4/5 and right link 6/7/8/hand
        // by applying an ACM diff to the planning scene explicitly BEFORE retreating.
        {
            moveit_msgs::msg::PlanningScene planning_scene_msg;
            planning_scene_msg.is_diff = true;
            
            std::vector<std::string> link_names = {
                "mj_left_link4", "mj_left_link5", "mj_left_link6",
                "mj_right_link4", "mj_right_link5", "mj_right_link6", 
                "mj_right_link7", "mj_right_link8", "mj_right_hand"
            };
            
            planning_scene_msg.allowed_collision_matrix.entry_names = link_names;
            for (size_t i = 0; i < link_names.size(); ++i) {
                moveit_msgs::msg::AllowedCollisionEntry entry;
                entry.enabled.resize(link_names.size(), true); // true = ALLOW collision (do NOT check)
                planning_scene_msg.allowed_collision_matrix.entry_values.push_back(entry);
            }
            
            auto apply_planning_scene_client = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
            if (apply_planning_scene_client->wait_for_service(std::chrono::seconds(2))) {
                auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
                request->scene = planning_scene_msg;
                apply_planning_scene_client->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "  ✓ RETREAT阶段防误报 ACM diff 已下发 (免检左右臂特定交叉关节)");
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
        // ================================
"""

new_text = re.sub(hook, replacement, orig)

with open(cpp_file, 'w') as f:
    f.write(new_text)

print("Patch applied!")
