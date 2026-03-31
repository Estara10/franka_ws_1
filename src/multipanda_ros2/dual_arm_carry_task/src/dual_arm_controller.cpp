#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include "dual_arm_carry_task/dual_arm_controller.hpp"
#include "dual_arm_carry_task/gripper_controller.hpp"

void DualArmController::initialize()
    {
        RCLCPP_INFO(ctx_->get_logger(), "初始化 MoveIt 接口...");
        
        try {
            // 主要使用双臂协同组
            ctx_->dual_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                ctx_->shared_from_this(), ctx_->dual_arm_group_);
            RCLCPP_INFO(ctx_->get_logger(), "✓ 双臂协同组已加载");
            
            // 设置规划参数
            ctx_->dual_arm_->setPlanningTime(10.0);
            ctx_->dual_arm_->setNumPlanningAttempts(10);
            ctx_->dual_arm_->setMaxVelocityScalingFactor(0.5);
            ctx_->dual_arm_->setMaxAccelerationScalingFactor(0.5);
            
            // 设置更长的执行超时时间（默认是轨迹时间的1.5倍，我们设置为5倍）
            ctx_->dual_arm_->setGoalPositionTolerance(0.01);
            ctx_->dual_arm_->setGoalOrientationTolerance(0.01);
            
            // 同时创建单臂接口用于获取运动学信息
            ctx_->left_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                ctx_->shared_from_this(), ctx_->left_arm_group_);
            ctx_->right_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                ctx_->shared_from_this(), ctx_->right_arm_group_);
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ MoveIt 接口初始化完成");
            RCLCPP_INFO(ctx_->get_logger(), "  左臂末端: %s", ctx_->left_arm_->getEndEffectorLink().c_str());
            RCLCPP_INFO(ctx_->get_logger(), "  右臂末端: %s", ctx_->right_arm_->getEndEffectorLink().c_str());
            RCLCPP_INFO(ctx_->get_logger(), "  双臂组关节数: %zu", ctx_->dual_arm_->getJointNames().size());
            
            // 检查 SRDF 碰撞规则是否正确加载
            auto robot_model = ctx_->dual_arm_->getRobotModel();
            if (robot_model) {
                auto srdf = robot_model->getSRDF();
                if (srdf) {
                    const auto& disabled_pairs = srdf->getDisabledCollisionPairs();
                    RCLCPP_INFO(ctx_->get_logger(), "  SRDF 禁用碰撞规则数: %zu", disabled_pairs.size());
                    
                    // 检查关键规则
                    bool found_right_0_1 = false;
                    bool found_left_0_1 = false;
                    for (const auto& pair : disabled_pairs) {
                        if ((pair.link1_ == "mj_right_link0" && pair.link2_ == "mj_right_link1") ||
                            (pair.link1_ == "mj_right_link1" && pair.link2_ == "mj_right_link0")) {
                            found_right_0_1 = true;
                        }
                        if ((pair.link1_ == "mj_left_link0" && pair.link2_ == "mj_left_link1") ||
                            (pair.link1_ == "mj_left_link1" && pair.link2_ == "mj_left_link0")) {
                            found_left_0_1 = true;
                        }
                    }
                    RCLCPP_INFO(ctx_->get_logger(), "  [SRDF检查] mj_right_link0<->mj_right_link1: %s", 
                               found_right_0_1 ? "✓ 存在" : "✗ 缺失");
                    RCLCPP_INFO(ctx_->get_logger(), "  [SRDF检查] mj_left_link0<->mj_left_link1: %s", 
                               found_left_0_1 ? "✓ 存在" : "✗ 缺失");
                } else {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ SRDF 未加载!");
                }
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "MoveIt 初始化失败: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
            return;
        }
        
        RCLCPP_INFO(ctx_->get_logger(), "=== 初始化完成，准备执行任务 ===\n");
    }

/**
     * @brief 诊断碰撞状态 - 输出详细的碰撞信息
     */
    void DualArmController::diagnoseCollision()
    {
        RCLCPP_INFO(ctx_->get_logger(), "  [诊断] 检查当前状态碰撞...");
        
        // 获取当前机器人状态
        auto current_state = ctx_->dual_arm_->getCurrentState();
        auto robot_model = ctx_->dual_arm_->getRobotModel();
        
        // 创建规划场景用于碰撞检测
        auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
        
        // 从 MoveGroupInterface 同步当前规划场景
        // 注意：这里使用 PlanningSceneInterface 获取当前场景中的物体
        moveit::planning_interface::PlanningSceneInterface psi;
        auto objects = psi.getObjects();
        RCLCPP_INFO(ctx_->get_logger(), "  [诊断] 场景中有 %zu 个物体", objects.size());
        
        // 获取 ACM
        collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
        
        // 检查关键碰撞对
        std::vector<std::pair<std::string, std::string>> key_pairs = {
            {"mj_right_link0", "mj_right_link1"},
            {"mj_left_link0", "mj_left_link1"},
            {"base_link", "mj_left_link0"},
            {"base_link", "mj_right_link0"},
        };
        
        for (const auto& pair : key_pairs) {
            collision_detection::AllowedCollision::Type allowed_type;
            bool found = acm.getEntry(pair.first, pair.second, allowed_type);
            if (found) {
                std::string type_str = (allowed_type == collision_detection::AllowedCollision::ALWAYS) ? "ALWAYS_ALLOWED" :
                                       (allowed_type == collision_detection::AllowedCollision::NEVER) ? "NEVER_ALLOWED" :
                                       "CONDITIONAL";
                RCLCPP_INFO(ctx_->get_logger(), "  [诊断] %s <-> %s: %s", 
                           pair.first.c_str(), pair.second.c_str(), type_str.c_str());
            } else {
                RCLCPP_WARN(ctx_->get_logger(), "  [诊断] %s <-> %s: 未在ACM中找到", 
                           pair.first.c_str(), pair.second.c_str());
            }
        }
        
        

        // 检查自碰撞
        collision_detection::CollisionRequest collision_request;
        collision_request.contacts = true;
        collision_request.max_contacts = 100;
        collision_detection::CollisionResult collision_result;
        
        planning_scene->checkSelfCollision(collision_request, collision_result, *current_state, acm);
        
        if (collision_result.collision) {
            RCLCPP_ERROR(ctx_->get_logger(), "  [诊断] 检测到自碰撞！碰撞对:");
            for (const auto& contact : collision_result.contacts) {
                RCLCPP_ERROR(ctx_->get_logger(), "    - %s <-> %s", 
                            contact.first.first.c_str(), contact.first.second.c_str());
            }
        } else {
            RCLCPP_INFO(ctx_->get_logger(), "  [诊断] ✓ 无自碰撞");
        }
    }

void DualArmController::initializeCollisionMatrix()
    {
        RCLCPP_INFO(ctx_->get_logger(), "  初始化碰撞矩阵（禁用相邻关节碰撞）...");
        
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.is_diff = true;
        
        // 定义所有需要禁用碰撞的链接对
        std::vector<std::pair<std::string, std::string>> collision_pairs = {
            // 左臂相邻关节
            {"mj_left_link0", "mj_left_link1"},
            {"mj_left_link1", "mj_left_link2"},
            {"mj_left_link2", "mj_left_link3"},
            {"mj_left_link3", "mj_left_link4"},
            {"mj_left_link4", "mj_left_link5"},
            {"mj_left_link5", "mj_left_link6"},
            {"mj_left_link6", "mj_left_link7"},
            {"mj_left_link7", "mj_left_link8"},
            {"mj_left_link8", "mj_left_hand"},
            {"mj_left_hand", "mj_left_leftfinger"},
            {"mj_left_hand", "mj_left_rightfinger"},
            {"mj_left_leftfinger", "mj_left_rightfinger"},
            // 左臂非相邻但不会碰撞的
            {"mj_left_link0", "mj_left_link2"},
            {"mj_left_link0", "mj_left_link3"},
            {"mj_left_link0", "mj_left_link4"},
            {"mj_left_link1", "mj_left_link3"},
            {"mj_left_link1", "mj_left_link4"},
            {"mj_left_link2", "mj_left_link4"},
            {"mj_left_link2", "mj_left_link6"},
            {"mj_left_link3", "mj_left_link5"},
            {"mj_left_link3", "mj_left_link6"},
            {"mj_left_link3", "mj_left_link7"},
            {"mj_left_link4", "mj_left_link6"},
            {"mj_left_link4", "mj_left_link7"},
            {"mj_left_link5", "mj_left_link7"},
            {"mj_left_link5", "mj_left_link8"},
            {"mj_left_link6", "mj_left_link8"},
            {"mj_left_link7", "mj_left_hand"},
            
            // 右臂相邻关节
            {"mj_right_link0", "mj_right_link1"},
            {"mj_right_link1", "mj_right_link2"},
            {"mj_right_link2", "mj_right_link3"},
            {"mj_right_link3", "mj_right_link4"},
            {"mj_right_link4", "mj_right_link5"},
            {"mj_right_link5", "mj_right_link6"},
            {"mj_right_link6", "mj_right_link7"},
            {"mj_right_link7", "mj_right_link8"},
            {"mj_right_link8", "mj_right_hand"},
            {"mj_right_hand", "mj_right_leftfinger"},
            {"mj_right_hand", "mj_right_rightfinger"},
            {"mj_right_leftfinger", "mj_right_rightfinger"},
            // 右臂非相邻但不会碰撞的
            {"mj_right_link0", "mj_right_link2"},
            {"mj_right_link0", "mj_right_link3"},
            {"mj_right_link0", "mj_right_link4"},
            {"mj_right_link1", "mj_right_link3"},
            {"mj_right_link1", "mj_right_link4"},
            {"mj_right_link2", "mj_right_link4"},
            {"mj_right_link2", "mj_right_link6"},
            {"mj_right_link3", "mj_right_link5"},
            {"mj_right_link3", "mj_right_link6"},
            {"mj_right_link3", "mj_right_link7"},
            {"mj_right_link4", "mj_right_link6"},
            {"mj_right_link4", "mj_right_link7"},
            {"mj_right_link5", "mj_right_link7"},
            {"mj_right_link5", "mj_right_link8"},
            {"mj_right_link6", "mj_right_link8"},
            {"mj_right_link7", "mj_right_hand"},
            
            // base_link 与 link0
            {"base_link", "mj_left_link0"},
            {"base_link", "mj_right_link0"},
            
            // 双臂之间的底座和低位连杆
            {"mj_left_link0", "mj_right_link0"},
            {"mj_left_link0", "mj_right_link1"},
            {"mj_left_link0", "mj_right_link2"},
            {"mj_left_link1", "mj_right_link0"},
            {"mj_left_link1", "mj_right_link1"},
            {"mj_left_link2", "mj_right_link0"},
            {"mj_left_link2", "mj_right_link1"},
            
            // 双臂躯干碰撞禁用
            {"mj_left_link2", "mj_right_link2"},
            {"mj_left_link2", "mj_right_link3"},
            {"mj_left_link2", "mj_right_link4"},
            {"mj_left_link2", "mj_right_link5"},
            {"mj_left_link3", "mj_right_link2"},
            {"mj_left_link3", "mj_right_link3"},
            {"mj_left_link3", "mj_right_link4"},
            {"mj_left_link3", "mj_right_link5"},
            {"mj_left_link4", "mj_right_link2"},
            {"mj_left_link4", "mj_right_link3"},
            {"mj_left_link4", "mj_right_link4"},
            {"mj_left_link4", "mj_right_link5"},
            {"mj_left_link5", "mj_right_link2"},
            {"mj_left_link5", "mj_right_link3"},
            {"mj_left_link5", "mj_right_link4"},
            {"mj_left_link5", "mj_right_link5"},
        };
        
        // 收集所有唯一的链接名称
        std::set<std::string> link_set;
        for (const auto& pair : collision_pairs) {
            link_set.insert(pair.first);
            link_set.insert(pair.second);
        }
        std::vector<std::string> all_links(link_set.begin(), link_set.end());
        
        // 创建链接名称到索引的映射
        std::map<std::string, size_t> link_to_idx;
        for (size_t i = 0; i < all_links.size(); ++i) {
            link_to_idx[all_links[i]] = i;
        }
        
        // 初始化 ACM 矩阵（默认不允许碰撞）
        planning_scene_msg.allowed_collision_matrix.entry_names = all_links;
        size_t n = all_links.size();
        for (size_t i = 0; i < n; ++i) {
            moveit_msgs::msg::AllowedCollisionEntry entry;
            entry.enabled.resize(n, false);  // 默认不允许
            planning_scene_msg.allowed_collision_matrix.entry_values.push_back(entry);
        }
        
        // 设置允许碰撞的对
        for (const auto& pair : collision_pairs) {
            size_t i = link_to_idx[pair.first];
            size_t j = link_to_idx[pair.second];
            planning_scene_msg.allowed_collision_matrix.entry_values[i].enabled[j] = true;
            planning_scene_msg.allowed_collision_matrix.entry_values[j].enabled[i] = true;  // 对称
        }
        
        // 使用 /apply_planning_scene 服务来确保 ACM 正确应用
        auto apply_planning_scene_client = ctx_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
            "/apply_planning_scene");
        
        if (!apply_planning_scene_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ /apply_planning_scene 服务不可用，回退到 topic 发布");
            // 回退到 topic 发布
            auto planning_scene_diff_pub = ctx_->create_publisher<moveit_msgs::msg::PlanningScene>(
                "/planning_scene", 10);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            planning_scene_diff_pub->publish(planning_scene_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
            request->scene = planning_scene_msg;
            
            auto future = apply_planning_scene_client->async_send_request(request);
            
            // 等待服务响应（使用简单的等待，避免 executor 冲突）
            auto status = future.wait_for(std::chrono::seconds(5));
            if (status == std::future_status::ready) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ ACM 通过服务成功应用");
                } else {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ ACM 应用失败");
                }
            } else {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ /apply_planning_scene 服务调用超时");
            }
        }
        
        RCLCPP_INFO(ctx_->get_logger(), "  ✓ ACM 已初始化：禁用 %zu 对链接的碰撞检测", collision_pairs.size());
    }

void DualArmController::allow_gripper_collision(bool allow)
    {
        using namespace std::chrono_literals;
        
        // 1. 获取当前完整的 PlanningScene（包含 SRDF 的所有规则）
        auto get_scene_client = ctx_->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/get_planning_scene");
        
        if (!get_scene_client->wait_for_service(3s)) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ /get_planning_scene 服务不可用");
            return;
        }
        
        auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        get_request->components.components = 
            moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        
        // 使用同步调用（不会触发 executor 冲突）
        auto get_future = get_scene_client->async_send_request(get_request);
        
        // 简单等待而不是 spin
        auto status = get_future.wait_for(3s);
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 获取当前 PlanningScene 超时");
            return;
        }
        
        auto get_response = get_future.get();
        auto& current_scene = get_response->scene;
        
        // 2. 在完整 ACM 基础上，只修改夹爪-物体的碰撞对
        std::vector<std::string> gripper_links = {
            "mj_left_hand", "mj_left_leftfinger", "mj_left_rightfinger",
            "mj_right_hand", "mj_right_leftfinger", "mj_right_rightfinger"
        };
        std::string object_id = "aluminum_rod";
        
        auto& acm = current_scene.allowed_collision_matrix;
        
        // 查找物体在 ACM 中的索引
        auto obj_it = std::find(acm.entry_names.begin(), acm.entry_names.end(), object_id);
        if (obj_it == acm.entry_names.end()) {
            RCLCPP_WARN(ctx_->get_logger(), "  ! 物体 %s 不在 ACM 中，跳过更新", object_id.c_str());
            return;
        }
        size_t obj_idx = std::distance(acm.entry_names.begin(), obj_it);
        
        // 修改每个夹爪与物体的碰撞
        for (const auto& gripper : gripper_links) {
            auto grip_it = std::find(acm.entry_names.begin(), acm.entry_names.end(), gripper);
            if (grip_it != acm.entry_names.end()) {
                size_t grip_idx = std::distance(acm.entry_names.begin(), grip_it);
                // 对称设置
                acm.entry_values[grip_idx].enabled[obj_idx] = allow;
                acm.entry_values[obj_idx].enabled[grip_idx] = allow;
            }
        }
        
        // 3. 应用修改后的完整 ACM
        auto apply_client = ctx_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
            "/apply_planning_scene");
        
        if (!apply_client->wait_for_service(2s)) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ /apply_planning_scene 服务不可用");
            return;
        }
        
        auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        apply_request->scene.is_diff = false;  // 发送完整场景，不是 diff
        apply_request->scene.allowed_collision_matrix = acm;
        
        auto apply_future = apply_client->async_send_request(apply_request);
        
        // 使用同步等待
        auto apply_status = apply_future.wait_for(3s);
        if (apply_status == std::future_status::ready) {
            auto apply_response = apply_future.get();
            if (apply_response->success) {
                RCLCPP_INFO(ctx_->get_logger(), "  ✓ ACM已安全更新：%s夹爪与物体碰撞（保留所有SRDF规则）", 
                           allow ? "允许" : "禁止");
            } else {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ ACM更新失败");
            }
        } else {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ ACM更新服务调用超时");
        }
    }

void DualArmController::publishAluminumRod()
    {
        using namespace std::chrono_literals;
        
        // 创建 Planning Scene Interface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        // 创建碰撞物体消息
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "aluminum_rod";
        
        // 定义铝棒形状 (横向放置: 4cm(X) x 40cm(Y) x 4cm(Z))
        // 注意：现在长轴在Y方向
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.04;  // X方向宽度
        primitive.dimensions[1] = 0.40;  // Y方向长度 (长轴)
        primitive.dimensions[2] = 0.04;  // Z方向高度
        
        // 定义铝棒位置 (与MuJoCo中objects.xml一致)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.50;  // 与MuJoCo中的位置一致
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.02;  // 物体中心高度 (底部接触地面)
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        
        // 发布到 Planning Scene
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);
        planning_scene_interface.addCollisionObjects(collision_objects);
        
        RCLCPP_INFO(ctx_->get_logger(), "  ✓ 铝棒碰撞物体已添加到 Planning Scene");
        
        // 等待碰撞物体被处理
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 【修复】手动将物体添加到 ACM 中
        // PlanningSceneInterface 不会自动将物体添加到 ACM entry_names，需要手动操作
        auto get_scene_client = ctx_->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/get_planning_scene");
        
        if (!get_scene_client->wait_for_service(3s)) {
            RCLCPP_WARN(ctx_->get_logger(), "  ! /get_planning_scene 服务不可用，无法将物体添加到ACM");
            return;
        }
        
        // 获取当前完整场景
        auto get_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        get_request->components.components = 
            moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        
        auto get_future = get_scene_client->async_send_request(get_request);
        auto status = get_future.wait_for(3s);
        
        if (status != std::future_status::ready) {
            RCLCPP_WARN(ctx_->get_logger(), "  ! 获取 PlanningScene 超时");
            return;
        }
        
        auto get_response = get_future.get();
        auto& acm = get_response->scene.allowed_collision_matrix;
        
        // 检查物体是否已在ACM中
        auto obj_it = std::find(acm.entry_names.begin(), acm.entry_names.end(), "aluminum_rod");
        if (obj_it == acm.entry_names.end()) {
            // 物体不在ACM中，需要手动添加
            size_t current_size = acm.entry_names.size();
            
            // 添加物体名称到entry_names
            acm.entry_names.push_back("aluminum_rod");
            
            // 为现有所有entry添加与新物体的碰撞关系（默认禁止碰撞）
            for (size_t i = 0; i < current_size; ++i) {
                acm.entry_values[i].enabled.push_back(false);  // 禁止碰撞
            }
            
            // 为新物体创建entry_values行
            moveit_msgs::msg::AllowedCollisionEntry new_entry;
            new_entry.enabled.resize(current_size + 1, false);  // 包括自己，全部禁止碰撞
            acm.entry_values.push_back(new_entry);
            
            // 应用更新后的ACM
            auto apply_client = ctx_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
                "/apply_planning_scene");
            
            if (!apply_client->wait_for_service(2s)) {
                RCLCPP_WARN(ctx_->get_logger(), "  ! /apply_planning_scene 服务不可用");
                return;
            }
            
            auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
            apply_request->scene.is_diff = false;  // 发送完整ACM
            apply_request->scene.allowed_collision_matrix = acm;
            
            auto apply_future = apply_client->async_send_request(apply_request);
            auto apply_status = apply_future.wait_for(3s);
            
            if (apply_status == std::future_status::ready) {
                auto apply_response = apply_future.get();
                if (apply_response->success) {
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ 物体 aluminum_rod 已添加到 ACM（默认禁止所有碰撞）");
                } else {
                    RCLCPP_WARN(ctx_->get_logger(), "  ! ACM 更新失败");
                }
            } else {
                RCLCPP_WARN(ctx_->get_logger(), "  ! ACM 更新超时");
            }
        } else {
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 物体 aluminum_rod 已存在于 ACM 中");
        }
    }

void DualArmController::executeInit()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: INIT] 初始化...");
        
        try {
            // 跳过初始位置移动，直接使用当前位置
            RCLCPP_INFO(ctx_->get_logger(), "  使用当前机械臂位置作为起始点");
            
            // 设定RETREAT目标关节位置（Franka标准ready姿态）
            // 注意: MuJoCo启动时关节状态为[0,0,0,-1.571,0,0,0]（默认零位），
            // 此姿态不适合作为RETREAT目标（手臂完全伸直+向前）。
            // 使用Franka标准ready位置: [0, -π/4, 0, -3π/4, 0, π/2, π/4]
            ctx_->initial_left_joints_ = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
            ctx_->initial_right_joints_ = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ RETREAT目标已设为Franka标准ready姿态");
            
            // 【已禁用】不再手动初始化 ACM，完全依赖 SRDF 的默认碰撞规则
            // ctx_->arms_->initializeCollisionMatrix();
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 使用 SRDF 默认碰撞矩阵（无需手动初始化）");
            
            // 发布铝棒碰撞物体到 Planning Scene
            ctx_->arms_->publishAluminumRod();
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ 初始化完成，进入接近阶段\n");
            ctx_->current_state_ = TaskState::APPROACH;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "INIT 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::executeApproach()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: APPROACH] 接近物体...");
        
        // ========== 物体与机械臂参数定义 ==========
        // 物体位置 (从objects.xml: pos="0.5 0.0 0.02", 物体底部接触地面)
        double obj_x = 0.50;
        double obj_y = 0.0;
        
        // 杆件参数 (长轴在Y方向，横向放置)
           // Y方向半长 20cm
        
        // ========== Z轴高度参数（来自 launch 参数） ==========
        double grasp_z = ctx_->grasp_height_;
        double approach_z = ctx_->approach_height_;

        if (approach_z <= grasp_z) {
            RCLCPP_WARN(ctx_->get_logger(),
                        "  ! 参数关系异常: approach_height(%.3f) <= grasp_height(%.3f)，可能导致接近/下降轨迹不合理",
                        approach_z, grasp_z);
        }
        
        // ========== 一阶Y定位：到达即就绪 ==========
        double final_y_offset = 0.15;  // 预解算：直接对齐并到达收缩距离 (±0.15)
        
        try {
            // ========== 提前打开夹爪 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  ★ [同步动作] 提前满幅打开夹爪...");
            ctx_->grippers_->controlDualGrippers(0.04);
            
            // ========== 左臂目标位姿 (全局姿态预解算) ==========
            geometry_msgs::msg::Pose left_target;
            left_target.position.x = obj_x;
            left_target.position.y = obj_y + final_y_offset;  // Y=+0.15 
            left_target.position.z = approach_z; 
            
            // 全局预解算：直接设置目标Yaw为45度（使手指彻底平行于Y轴）
            tf2::Quaternion left_quat;
            left_quat.setRPY(M_PI, 0.0, M_PI/4.0);
            left_target.orientation = tf2::toMsg(left_quat);
            
            // ========== 右臂目标位姿 (全局姿态预解算) ==========
            geometry_msgs::msg::Pose right_target;
            right_target.position.x = obj_x;
            right_target.position.y = obj_y - final_y_offset;  // Y=-0.15
            right_target.position.z = approach_z; 
            
            // 全局预解算：右臂同样设置Yaw为45度（对称抓取物体两端）
            tf2::Quaternion right_quat;
            right_quat.setRPY(M_PI, 0.0, M_PI/4.0);
            right_target.orientation = tf2::toMsg(right_quat);
            
            RCLCPP_INFO(ctx_->get_logger(), "  左臂预定位目标: [%.2f, %.2f, %.2f] 姿态:(R=180°,P=0°,Y=45°)", 
                        left_target.position.x, left_target.position.y, left_target.position.z);
            RCLCPP_INFO(ctx_->get_logger(), "  右臂预定位目标: [%.2f, %.2f, %.2f] 姿态:(R=180°,P=0°,Y=45°)",
                        right_target.position.x, right_target.position.y, right_target.position.z);
            
            // ========== 使用setPoseTarget设置目标（关键改变！） ==========
            ctx_->left_arm_->setPoseTarget(left_target);
            ctx_->right_arm_->setPoseTarget(right_target);
            
            // ========== 使用双臂协同组规划 ==========
            // 获取当前状态
            auto robot_state = ctx_->dual_arm_->getCurrentState();
            
            // 从单臂目标构建双臂目标状态
            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto target_state = std::make_shared<moveit::core::RobotState>(robot_model);
            *target_state = *robot_state;
            
            // 为左臂设置目标位姿并计算IK
            Eigen::Isometry3d left_pose, right_pose;
            tf2::fromMsg(left_target, left_pose);
            tf2::fromMsg(right_target, right_pose);
            
            auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
            bool left_ik_ok = target_state->setFromIK(left_jmg, left_pose, 5.0);
            
            auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);
            bool right_ik_ok = target_state->setFromIK(right_jmg, right_pose, 5.0);
            
            if (!left_ik_ok || !right_ik_ok) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ IK求解失败 (左臂:%d 右臂:%d)", left_ik_ok, right_ik_ok);
                RCLCPP_ERROR(ctx_->get_logger(), "    左臂目标: [%.3f, %.3f, %.3f]", 
                            left_target.position.x, left_target.position.y, left_target.position.z);
                RCLCPP_ERROR(ctx_->get_logger(), "    右臂目标: [%.3f, %.3f, %.3f]",
                            right_target.position.x, right_target.position.y, right_target.position.z);
                RCLCPP_WARN(ctx_->get_logger(), "  ! 目标位姿可能超出工作空间或存在奇异点，跳过此阶段");
                ctx_->current_state_ = TaskState::DONE;
                return;
            }
            
            // 使用双臂组规划到IK解
            ctx_->dual_arm_->setJointValueTarget(*target_state);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto success = ctx_->dual_arm_->plan(plan);
            
            if (success != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 双臂规划失败");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 双臂规划成功，开始执行...");
            auto result = ctx_->dual_arm_->execute(plan);
            if (result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 双臂执行失败");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 双臂移动完成");
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ 接近完成，进入抓取阶段\n");
            ctx_->current_state_ = TaskState::GRASP;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "APPROACH 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::executeGrasp()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: GRASP] 夹紧物体...");
        RCLCPP_INFO(ctx_->get_logger(), "  新策略(一阶优化): 仅保留垂直下降介入 → 夹取");
        
        // ========== 高度参数 ==========
        double approach_z = ctx_->approach_height_;  // 当前APPROACH高度
        double grasp_z = ctx_->grasp_height_;        // 抓取高度
        
        try {
            // 已在 Approach 阶段完成全局位姿对齐和夹爪预先开启，且已经到位
            
            // ========== Step 1: 垂直直线下降介入 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [GRASP] Step 1: 垂直下降 (Z: %.2f → %.2f)", approach_z, grasp_z);
            
            // 获取当前末端位姿
            geometry_msgs::msg::PoseStamped current_left = ctx_->left_arm_->getCurrentPose();
            geometry_msgs::msg::PoseStamped current_right = ctx_->right_arm_->getCurrentPose();
            
            // 使用关节空间规划而非笛卡尔路径(避免单臂控制器问题)
            moveit::core::RobotStatePtr current_state = ctx_->dual_arm_->getCurrentState();
            
            // 设置目标仅在 Z 轴改变 (保持 XY 和姿态完全不变)
            geometry_msgs::msg::Pose left_target = current_left.pose;
            left_target.position.z = grasp_z;
            
            geometry_msgs::msg::Pose right_target = current_right.pose;
            right_target.position.z = grasp_z;
            
            // IK求解双臂目标关节角
            const moveit::core::JointModelGroup* left_jmg = current_state->getJointModelGroup(ctx_->left_arm_group_);
            const moveit::core::JointModelGroup* right_jmg = current_state->getJointModelGroup(ctx_->right_arm_group_);
            
            bool left_ik = current_state->setFromIK(left_jmg, left_target, 0.05);
            bool right_ik = current_state->setFromIK(right_jmg, right_target, 0.05);
            
            if (!left_ik || !right_ik) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 下降阶段IK无解! Left=%s, Right=%s", left_ik ? "OK":"FAIL", right_ik ? "OK":"FAIL");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            // 使用双臂组规划并执行
            ctx_->dual_arm_->setJointValueTarget(*current_state);
            moveit::planning_interface::MoveGroupInterface::Plan descent_plan;
            
            if (ctx_->dual_arm_->plan(descent_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 垂直下降轨迹规划失败!");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            if (ctx_->dual_arm_->execute(descent_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 垂直下降轨迹执行失败!");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 垂直介入完成");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // ========== Step 2: 碰撞免除同步 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [GRASP] Step 2: 更新世界ACM允许夹取碰撞");
            ctx_->arms_->allow_gripper_collision(true);
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ ACM 更新完毕");
            
            // ========== Step 3: 闭合夹爪并附着物体 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [GRASP] Step 3: Closing grippers and attaching object");
            
            // 闭合夹爪
            ctx_->grippers_->controlDualGrippers(ctx_->gripper_close_pos_);   // 同步闭合
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ Grippers closed");
            
            // 关键：等待MuJoCo物理引擎完全稳定
            // 夹爪闭合瞬间会产生反作用力，需要时间让物理引擎收敛
            RCLCPP_INFO(ctx_->get_logger(), "  [等待] 物理引擎稳定中（1.5秒）...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            
            // 激活MuJoCo weld约束 —— 将铝棒刚性绑定到左手
            // 原理：MoveIt的attachObject()只是规划层面的附着，MuJoCo物理引擎并不知道
            // weld约束是MuJoCo中实现刚性抓取的标准方法
            {
                auto weld_msg = std_msgs::msg::Bool();
                weld_msg.data = true;
                ctx_->weld_pub_->publish(weld_msg);
                RCLCPP_INFO(ctx_->get_logger(), "  [MuJoCo] Weld约束已激活（铝棒刚性绑定到双手）");
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            // 附着物体到机器人
            ctx_->arms_->attachObject();
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ Object attached to robot");
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ 抓取完成，进入提升阶段\n");
            ctx_->current_state_ = TaskState::LIFT;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "GRASP 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::attachObject()
    {
        using namespace std::chrono_literals;
        
        // 定义接触链接（允许这些链接与物体碰撞）
        std::vector<std::string> touch_links = {
            "mj_left_link5", "mj_left_link6", "mj_left_link7", "mj_left_link8", "mj_left_hand",
            "mj_left_leftfinger", "mj_left_rightfinger",
            "mj_right_link5", "mj_right_link6", "mj_right_link7", "mj_right_link8", "mj_right_hand",
            "mj_right_rightfinger", "mj_right_leftfinger"
        };
        
        // 附着目标连杆
        std::string attach_link = "mj_left_hand";
        
        RCLCPP_INFO(ctx_->get_logger(), "  [附着] 开始附着流程（最终修复版 - 全局坐标系）");
        RCLCPP_INFO(ctx_->get_logger(), "    附着到: %s", attach_link.c_str());
        
        auto planning_scene_pub = ctx_->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(10).transient_local());
        std::this_thread::sleep_for(100ms);
        
        // ========== 步骤1: 获取物体真实世界位姿（必须在移除之前！） ==========
        RCLCPP_INFO(ctx_->get_logger(), "  [步骤1] 从 PlanningScene 获取物体实际位姿...");
        
        moveit::planning_interface::PlanningSceneInterface psi;
        auto existing_objects = psi.getObjects(std::vector<std::string>{"aluminum_rod"});
        
        geometry_msgs::msg::Pose object_world_pose;
        if (existing_objects.count("aluminum_rod") > 0 && 
            !existing_objects["aluminum_rod"].primitive_poses.empty()) {
            object_world_pose = existing_objects["aluminum_rod"].primitive_poses[0];
            RCLCPP_INFO(ctx_->get_logger(), "    ✓ 从 PlanningScene 获取物体位姿: [%.3f, %.3f, %.3f]",
                       object_world_pose.position.x, object_world_pose.position.y, object_world_pose.position.z);
        } else {
            // 回退：使用 ctx_->arms_->publishAluminumRod() 中的已知初始位置
            RCLCPP_WARN(ctx_->get_logger(), "    ! 无法从PlanningScene获取物体，使用已知初始位置");
            object_world_pose.position.x = 0.50;
            object_world_pose.position.y = 0.0;
            object_world_pose.position.z = 0.02;
            object_world_pose.orientation.w = 1.0;
            object_world_pose.orientation.x = 0.0;
            object_world_pose.orientation.y = 0.0;
            object_world_pose.orientation.z = 0.0;
        }
        
        // 输出夹爪位置用于调试对比（hand 位于手腕处，finger 位于指尖处）
        auto current_state = ctx_->dual_arm_->getCurrentState();
        const Eigen::Isometry3d& left_hand_tf = current_state->getGlobalLinkTransform("mj_left_hand");
        const Eigen::Isometry3d& left_finger_tf = current_state->getGlobalLinkTransform("mj_left_leftfinger");
        const Eigen::Isometry3d& right_finger_tf = current_state->getGlobalLinkTransform("mj_right_leftfinger");
        RCLCPP_INFO(ctx_->get_logger(), "    [调试] 左hand world Z=%.3f, 左finger world Z=%.3f, 右finger world Z=%.3f",
                   left_hand_tf.translation().z(), left_finger_tf.translation().z(), right_finger_tf.translation().z());
        RCLCPP_INFO(ctx_->get_logger(), "    [调试] 物体实际Z=%.3f (来自PlanningScene，非hand中点！)", 
                   object_world_pose.position.z);
        
        // ========== 步骤2: 移除独立碰撞物体 ==========
        RCLCPP_INFO(ctx_->get_logger(), "  [步骤2] 移除独立碰撞物体...");
        
        moveit_msgs::msg::CollisionObject remove_object;
        remove_object.header.frame_id = "base_link";
        remove_object.id = "aluminum_rod";
        remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        
        moveit_msgs::msg::PlanningScene remove_scene;
        remove_scene.world.collision_objects.push_back(remove_object);
        remove_scene.is_diff = true;
        
        planning_scene_pub->publish(remove_scene);
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 发布 REMOVE 消息");
        std::this_thread::sleep_for(300ms);
        
        // ========== 步骤3: 发布附着物体（使用全局坐标系）==========
        RCLCPP_INFO(ctx_->get_logger(), "  [步骤3] 发布附着碰撞物体...");
        RCLCPP_INFO(ctx_->get_logger(), "    使用物体真实位姿: [%.3f, %.3f, %.3f]（非hand中点！）",
                   object_world_pose.position.x, object_world_pose.position.y, object_world_pose.position.z);
        
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = attach_link;
        
        // 使用 base_link（全局坐标系），MoveIt 自动计算相对于 link_name 的偏移
        attached_object.object.header.frame_id = "base_link";
        attached_object.object.header.stamp = ctx_->now();
        attached_object.object.id = "aluminum_rod";
        
        // 定义形状（与 ctx_->arms_->publishAluminumRod() 中一致）
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.04, 0.40, 0.04};  // X=4cm, Y=40cm(长轴), Z=4cm
        
        attached_object.object.primitives.push_back(primitive);
        // 使用从 PlanningScene 获取的真实世界位姿
        attached_object.object.primitive_poses.push_back(object_world_pose);
        attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        attached_object.touch_links = touch_links;
        
        // 发布附着消息
        moveit_msgs::msg::PlanningScene attach_scene;
        attach_scene.robot_state.attached_collision_objects.push_back(attached_object);
        attach_scene.is_diff = true;
        
        planning_scene_pub->publish(attach_scene);
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 发布 ATTACH 消息（使用全局坐标系）");
        std::this_thread::sleep_for(500ms);
        
        // ========== 步骤4: 验证结果 ==========
        RCLCPP_INFO(ctx_->get_logger(), "  [步骤4] 验证附着结果...");
        
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        auto objects = planning_scene_interface.getObjects();
        auto attached_objects = planning_scene_interface.getAttachedObjects();
        
        RCLCPP_INFO(ctx_->get_logger(), "    独立物体数量: %zu (期望=0)", objects.size());
        RCLCPP_INFO(ctx_->get_logger(), "    附着物体数量: %zu (期望=1)", attached_objects.size());
        
        if (objects.size() > 0) {
            RCLCPP_WARN(ctx_->get_logger(), "    ! 仍有独立物体（可能是Goal State显示，可忽略）");
        } else {
            RCLCPP_INFO(ctx_->get_logger(), "    ✓ 独立物体已移除");
        }
        
        if (attached_objects.size() == 1) {
            RCLCPP_INFO(ctx_->get_logger(), "    ✓ 附着物体已确认");
        } else {
            RCLCPP_WARN(ctx_->get_logger(), "    ! 附着物体验证异常");
        }
        
        RCLCPP_INFO(ctx_->get_logger(), "  ✓ 附着流程完成");
        RCLCPP_INFO(ctx_->get_logger(), "    原理: 从PlanningScene获取物体真实位姿 + base_link全局坐标系");
        RCLCPP_INFO(ctx_->get_logger(), "    期望: 物体在地面原位附着（Z≈0.02），LIFT时随手臂抬升");
    }

void DualArmController::executeLift()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: LIFT] 抬起物体...");
        
        try {
            // 提升速度执行，weld约束保证铝棒稳固
            ctx_->dual_arm_->setMaxVelocityScalingFactor(0.5);
            ctx_->dual_arm_->setMaxAccelerationScalingFactor(0.5);
            RCLCPP_INFO(ctx_->get_logger(), "  [参数] 速度/加速度缩放: 50%%（weld约束保护）");

            // 关键：强制同步物理真实位置
            // 夹取动作可能导致机械臂被物理反作用力"推"开
            ctx_->dual_arm_->setStartStateToCurrentState();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 等待状态更新
            
            // 获取当前位置
            auto left_current = ctx_->left_arm_->getCurrentPose();
            auto right_current = ctx_->right_arm_->getCurrentPose();
            
            // 保持 x, y 不变，只改变 z
            geometry_msgs::msg::Pose left_lift = left_current.pose;
            left_lift.position.z = ctx_->lift_height_;
            
            geometry_msgs::msg::Pose right_lift = right_current.pose;
            right_lift.position.z = ctx_->lift_height_;
            
            // 使用IK计算双臂目标关节状态
            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
            *robot_state = *ctx_->dual_arm_->getCurrentState();
            
            auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
            bool left_ik_ok = robot_state->setFromIK(left_jmg, left_lift);
            
            auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);
            bool right_ik_ok = robot_state->setFromIK(right_jmg, right_lift);
            
            if (!left_ik_ok || !right_ik_ok) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 抬升IK求解失败");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            ctx_->dual_arm_->setJointValueTarget(*robot_state);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto success = ctx_->dual_arm_->plan(plan);
            
            if (success == moveit::core::MoveItErrorCode::SUCCESS) {
                auto result = ctx_->dual_arm_->execute(plan);
                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ 双臂抬升完成");
                } else {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 双臂执行失败");
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }
            } else {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 双臂抬升规划失败");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ 抬起完成\n");
            ctx_->current_state_ = TaskState::TRANSPORT;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "LIFT 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::executeTransport()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: TRANSPORT] 平移物体...");

        try {
            // 提升速度执行，weld约束保证铝棒稳固
            ctx_->dual_arm_->setMaxVelocityScalingFactor(0.4);
            ctx_->dual_arm_->setMaxAccelerationScalingFactor(0.4);
            RCLCPP_INFO(ctx_->get_logger(), "  [参数] 速度/加速度缩放: 40%%（降低运输阶段动态冲击）");

            // 发生IK/规划失败时，优先尝试在“同一末端目标”下调整冗余关节（肘/腕）来绕开碰撞与奇异构型。
            auto attempt_joint_redundancy_avoidance = [&](const geometry_msgs::msg::Pose& left_target,
                                                          const geometry_msgs::msg::Pose& right_target,
                                                          const std::string& stage_name) -> bool {
                auto robot_model = ctx_->dual_arm_->getRobotModel();
                auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
                auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);

                // [j4_delta, j6_delta, j7_delta]，左右臂采取相反符号，尽量把肘/腕从碰撞区“拧开”。
                const std::vector<std::vector<double>> bias_sets = {
                    {0.00, 0.00, 0.45},
                    {0.20, 0.00, 0.35},
                    {-0.20, 0.00, -0.35},
                    {0.25, 0.15, 0.30},
                    {-0.25, -0.15, -0.30},
                    {0.00, 0.25, 0.55},
                    {0.00, -0.25, -0.55}
                };

                for (size_t i = 0; i < bias_sets.size(); ++i) {
                    ctx_->dual_arm_->setStartStateToCurrentState();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));

                    auto candidate_state = std::make_shared<moveit::core::RobotState>(robot_model);
                    *candidate_state = *ctx_->dual_arm_->getCurrentState();
                    candidate_state->update();

                    std::vector<double> left_seed_vals, right_seed_vals;
                    candidate_state->copyJointGroupPositions(left_jmg, left_seed_vals);
                    candidate_state->copyJointGroupPositions(right_jmg, right_seed_vals);

                    if (left_seed_vals.size() > 3 && right_seed_vals.size() > 3) {
                        left_seed_vals[3] += bias_sets[i][0];
                        right_seed_vals[3] -= bias_sets[i][0];
                    }
                    if (left_seed_vals.size() > 5 && right_seed_vals.size() > 5) {
                        left_seed_vals[5] += bias_sets[i][1];
                        right_seed_vals[5] -= bias_sets[i][1];
                    }
                    if (left_seed_vals.size() > 6 && right_seed_vals.size() > 6) {
                        left_seed_vals[6] += bias_sets[i][2];
                        right_seed_vals[6] -= bias_sets[i][2];
                    }

                    candidate_state->setJointGroupPositions(left_jmg, left_seed_vals);
                    candidate_state->setJointGroupPositions(right_jmg, right_seed_vals);
                    candidate_state->enforceBounds();

                    bool left_ik = candidate_state->setFromIK(left_jmg, left_target, 0.3);
                    if (!left_ik) {
                        left_ik = candidate_state->setFromIK(left_jmg, left_target, 1.0);
                    }
                    bool right_ik = candidate_state->setFromIK(right_jmg, right_target, 0.3);
                    if (!right_ik) {
                        right_ik = candidate_state->setFromIK(right_jmg, right_target, 1.0);
                    }

                    if (!left_ik || !right_ik) {
                        RCLCPP_WARN(ctx_->get_logger(),
                            "  [%s] 关节避碰候选%zu IK失败 (左=%s, 右=%s)",
                            stage_name.c_str(), i + 1, left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                        continue;
                    }

                    candidate_state->update();
                    ctx_->dual_arm_->setJointValueTarget(*candidate_state);

                    moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;
                    auto candidate_plan_result = ctx_->dual_arm_->plan(candidate_plan);
                    if (candidate_plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_WARN(ctx_->get_logger(),
                            "  [%s] 关节避碰候选%zu 规划失败", stage_name.c_str(), i + 1);
                        continue;
                    }

                    auto candidate_exec_result = ctx_->dual_arm_->execute(candidate_plan);
                    if (candidate_exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_WARN(ctx_->get_logger(),
                            "  [%s] 关节避碰候选%zu 执行失败", stage_name.c_str(), i + 1);
                        continue;
                    }

                    RCLCPP_INFO(ctx_->get_logger(),
                        "  [%s] 关节避碰候选%zu 成功（通过肘/腕重构绕开冲突）",
                        stage_name.c_str(), i + 1);
                    return true;
                }

                return false;
            };

            auto perform_transport_shift = [&](double dx, double dy, const std::string& stage_name) -> bool {
                // 关键：强制同步物理真实位置（防止前一步执行后的残余偏移）
                ctx_->dual_arm_->setStartStateToCurrentState();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                auto left_current = ctx_->left_arm_->getCurrentPose();
                auto right_current = ctx_->right_arm_->getCurrentPose();

                geometry_msgs::msg::Pose left_target = left_current.pose;
                geometry_msgs::msg::Pose right_target = right_current.pose;
                left_target.position.x += dx;
                left_target.position.y += dy;
                right_target.position.x += dx;
                right_target.position.y += dy;

                RCLCPP_INFO(ctx_->get_logger(), "  [%s] 当前: Left(%.3f, %.3f, %.3f), Right(%.3f, %.3f, %.3f)",
                    stage_name.c_str(),
                    left_current.pose.position.x, left_current.pose.position.y, left_current.pose.position.z,
                    right_current.pose.position.x, right_current.pose.position.y, right_current.pose.position.z);
                RCLCPP_INFO(ctx_->get_logger(), "  [%s] 目标: Left(%.3f, %.3f, %.3f), Right(%.3f, %.3f, %.3f)",
                    stage_name.c_str(),
                    left_target.position.x, left_target.position.y, left_target.position.z,
                    right_target.position.x, right_target.position.y, right_target.position.z);

                auto robot_model = ctx_->dual_arm_->getRobotModel();
                auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
                auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);

                auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
                *robot_state = *ctx_->dual_arm_->getCurrentState();
                robot_state->update();

                bool left_ik_ok = robot_state->setFromIK(left_jmg, left_target, 0.1);
                if (!left_ik_ok) {
                    left_ik_ok = robot_state->setFromIK(left_jmg, left_target, 5.0);
                }

                bool right_ik_ok = robot_state->setFromIK(right_jmg, right_target, 0.1);
                if (!right_ik_ok) {
                    right_ik_ok = robot_state->setFromIK(right_jmg, right_target, 5.0);
                }

                // IK失败时采用分段小步回退
                if (!left_ik_ok || !right_ik_ok) {
                    RCLCPP_WARN(ctx_->get_logger(), "  [%s] IK失败，先尝试关节避碰重构", stage_name.c_str());
                    if (attempt_joint_redundancy_avoidance(left_target, right_target, stage_name)) {
                        return true;
                    }

                    RCLCPP_WARN(ctx_->get_logger(), "  [%s] IK失败，切换分段小步执行", stage_name.c_str());

                    double max_delta = std::max(std::abs(dx), std::abs(dy));
                    int segment_count = std::max(2, static_cast<int>(std::ceil(max_delta / 0.05)));
                    double seg_dx = dx / segment_count;
                    double seg_dy = dy / segment_count;

                    for (int seg = 0; seg < segment_count; ++seg) {
                        ctx_->dual_arm_->setStartStateToCurrentState();
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));

                        auto seg_left_current = ctx_->left_arm_->getCurrentPose();
                        auto seg_right_current = ctx_->right_arm_->getCurrentPose();

                        geometry_msgs::msg::Pose seg_left_target = seg_left_current.pose;
                        geometry_msgs::msg::Pose seg_right_target = seg_right_current.pose;
                        seg_left_target.position.x += seg_dx;
                        seg_left_target.position.y += seg_dy;
                        seg_right_target.position.x += seg_dx;
                        seg_right_target.position.y += seg_dy;

                        auto seg_state = std::make_shared<moveit::core::RobotState>(robot_model);
                        *seg_state = *ctx_->dual_arm_->getCurrentState();
                        seg_state->update();

                        bool seg_left_ik = seg_state->setFromIK(left_jmg, seg_left_target, 5.0);
                        bool seg_right_ik = seg_state->setFromIK(right_jmg, seg_right_target, 5.0);

                        if (!seg_left_ik || !seg_right_ik) {
                            RCLCPP_ERROR(ctx_->get_logger(), "  [%s] 分段 %d/%d IK失败", stage_name.c_str(), seg + 1, segment_count);
                            return false;
                        }

                        ctx_->dual_arm_->setJointValueTarget(*seg_state);
                        moveit::planning_interface::MoveGroupInterface::Plan seg_plan;
                        auto seg_plan_result = ctx_->dual_arm_->plan(seg_plan);
                        if (seg_plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                            RCLCPP_ERROR(ctx_->get_logger(), "  [%s] 分段 %d/%d 规划失败", stage_name.c_str(), seg + 1, segment_count);
                            return false;
                        }

                        auto seg_exec_result = ctx_->dual_arm_->execute(seg_plan);
                        if (seg_exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
                            RCLCPP_ERROR(ctx_->get_logger(), "  [%s] 分段 %d/%d 执行失败", stage_name.c_str(), seg + 1, segment_count);
                            return false;
                        }

                        RCLCPP_INFO(ctx_->get_logger(), "  [%s] 分段 %d/%d 完成", stage_name.c_str(), seg + 1, segment_count);
                        std::this_thread::sleep_for(std::chrono::milliseconds(150));
                    }

                    return true;
                }

                // IK成功，一步规划执行
                ctx_->dual_arm_->setStartStateToCurrentState();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                ctx_->dual_arm_->setJointValueTarget(*robot_state);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                auto plan_result = ctx_->dual_arm_->plan(plan);

                if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                    // 增加规划时间重试一次
                    ctx_->dual_arm_->setPlanningTime(10.0);
                    auto retry_plan_result = ctx_->dual_arm_->plan(plan);
                    ctx_->dual_arm_->setPlanningTime(5.0);
                    if (retry_plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_WARN(ctx_->get_logger(), "  [%s] 一步规划失败，尝试关节避碰重构", stage_name.c_str());
                        if (attempt_joint_redundancy_avoidance(left_target, right_target, stage_name)) {
                            return true;
                        }
                        RCLCPP_ERROR(ctx_->get_logger(), "  [%s] 一步规划失败", stage_name.c_str());
                        return false;
                    }
                }

                auto exec_result = ctx_->dual_arm_->execute(plan);
                if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(ctx_->get_logger(), "  [%s] 一步执行成功", stage_name.c_str());
                    return true;
                }

                // 执行失败再重试一次
                RCLCPP_WARN(ctx_->get_logger(), "  [%s] 执行失败，尝试重试一次", stage_name.c_str());
                ctx_->dual_arm_->setStartStateToCurrentState();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ctx_->dual_arm_->setJointValueTarget(*robot_state);

                moveit::planning_interface::MoveGroupInterface::Plan retry_plan;
                auto retry_plan_result = ctx_->dual_arm_->plan(retry_plan);
                if (retry_plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  [%s] 重试规划失败", stage_name.c_str());
                    return false;
                }

                auto retry_exec_result = ctx_->dual_arm_->execute(retry_plan);
                if (retry_exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(ctx_->get_logger(), "  [%s] 重试执行失败，尝试关节避碰重构", stage_name.c_str());
                    if (attempt_joint_redundancy_avoidance(left_target, right_target, stage_name)) {
                        return true;
                    }
                    RCLCPP_ERROR(ctx_->get_logger(), "  [%s] 重试执行失败", stage_name.c_str());
                    return false;
                }

                RCLCPP_INFO(ctx_->get_logger(), "  [%s] 重试执行成功", stage_name.c_str());
                return true;
            };

            // 预检当前运输后姿态是否还能完成ROTATE阶段，避免进入“可搬运但不可旋转”的死角构型。
            auto rotate_preview_feasible = [&]() -> bool {
                ctx_->dual_arm_->setStartStateToCurrentState();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                auto left_current = ctx_->left_arm_->getCurrentPose();
                auto right_current = ctx_->right_arm_->getCurrentPose();

                double center_x = (left_current.pose.position.x + right_current.pose.position.x) / 2.0;
                double center_y = (left_current.pose.position.y + right_current.pose.position.y) / 2.0;
                double center_z = (left_current.pose.position.z + right_current.pose.position.z) / 2.0;

                double left_dx0 = left_current.pose.position.x - center_x;
                double left_dy0 = left_current.pose.position.y - center_y;
                double radius = std::sqrt(left_dx0 * left_dx0 + left_dy0 * left_dy0);

                if (radius < 1e-4) {
                    RCLCPP_WARN(ctx_->get_logger(), "  [ROTATE预检] 半径过小(%.6f)，无法构造旋转轨迹", radius);
                    return false;
                }

                double left_init_angle = std::atan2(left_dy0, left_dx0);
                double right_dx0 = right_current.pose.position.x - center_x;
                double right_dy0 = right_current.pose.position.y - center_y;
                double right_init_angle = std::atan2(right_dy0, right_dx0);

                geometry_msgs::msg::Pose left_start_pose = left_current.pose;
                geometry_msgs::msg::Pose right_start_pose = right_current.pose;

                auto robot_model = ctx_->dual_arm_->getRobotModel();
                auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
                auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);

                auto seed_state = std::make_shared<moveit::core::RobotState>(*ctx_->dual_arm_->getCurrentState());

                const double total_angle = M_PI / 2.0;
                const int preview_points = 24;  // 约每3.75°采样一次，快速筛掉不可旋转姿态

                auto make_constraint = [](const std::vector<double>& seed_vals, double max_change)
                    -> moveit::core::GroupStateValidityCallbackFn {
                    return [seed_vals, max_change](
                        moveit::core::RobotState* /*state*/,
                        const moveit::core::JointModelGroup* /*group*/,
                        const double* joint_values) -> bool {
                        for (size_t j = 0; j < seed_vals.size(); ++j) {
                            double diff = std::abs(joint_values[j] - seed_vals[j]);
                            if (diff > M_PI) diff = 2.0 * M_PI - diff;
                            if (diff > max_change) return false;
                        }
                        return true;
                    };
                };

                for (int i = 1; i <= preview_points; ++i) {
                    double angle = total_angle * i / preview_points;

                    geometry_msgs::msg::Pose left_target = left_start_pose;
                    geometry_msgs::msg::Pose right_target = right_start_pose;

                    left_target.position.x = center_x + radius * std::cos(left_init_angle + angle);
                    left_target.position.y = center_y + radius * std::sin(left_init_angle + angle);
                    left_target.position.z = center_z;

                    right_target.position.x = center_x + radius * std::cos(right_init_angle + angle);
                    right_target.position.y = center_y + radius * std::sin(right_init_angle + angle);
                    right_target.position.z = center_z;

                    tf2::Quaternion q_rot;
                    q_rot.setRPY(0, 0, angle);

                    tf2::Quaternion q_left_orig, q_right_orig;
                    tf2::fromMsg(left_start_pose.orientation, q_left_orig);
                    tf2::fromMsg(right_start_pose.orientation, q_right_orig);

                    tf2::Quaternion q_left_new = q_rot * q_left_orig;
                    tf2::Quaternion q_right_new = q_rot * q_right_orig;
                    q_left_new.normalize();
                    q_right_new.normalize();
                    left_target.orientation = tf2::toMsg(q_left_new);
                    right_target.orientation = tf2::toMsg(q_right_new);

                    std::vector<double> left_seed_vals, right_seed_vals;
                    seed_state->copyJointGroupPositions(left_jmg, left_seed_vals);
                    seed_state->copyJointGroupPositions(right_jmg, right_seed_vals);

                    double joint7_bias = total_angle / preview_points;
                    if (left_seed_vals.size() > 6) left_seed_vals[6] += joint7_bias;
                    if (right_seed_vals.size() > 6) right_seed_vals[6] += joint7_bias;

                    auto waypoint_state = std::make_shared<moveit::core::RobotState>(*seed_state);
                    waypoint_state->setJointGroupPositions(left_jmg, left_seed_vals);
                    waypoint_state->setJointGroupPositions(right_jmg, right_seed_vals);

                    bool left_ik = waypoint_state->setFromIK(left_jmg, left_target, 0.1,
                        make_constraint(left_seed_vals, 0.8));
                    if (!left_ik) {
                        waypoint_state->setJointGroupPositions(left_jmg, left_seed_vals);
                        left_ik = waypoint_state->setFromIK(left_jmg, left_target, 1.0);
                    }

                    bool right_ik = waypoint_state->setFromIK(right_jmg, right_target, 0.1,
                        make_constraint(right_seed_vals, 0.8));
                    if (!right_ik) {
                        waypoint_state->setJointGroupPositions(right_jmg, right_seed_vals);
                        right_ik = waypoint_state->setFromIK(right_jmg, right_target, 1.0);
                    }

                    if (!left_ik || !right_ik) {
                        RCLCPP_WARN(ctx_->get_logger(),
                            "  [ROTATE预检] 航点 %d/%d 不可达（左=%s, 右=%s）",
                            i, preview_points, left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                        return false;
                    }

                    waypoint_state->update();
                    *seed_state = *waypoint_state;
                }

                RCLCPP_INFO(ctx_->get_logger(), "  [ROTATE预检] 通过：当前搬运位姿可执行90°旋转");
                return true;
            };

            // 阶段1：先向X轴正方向移动20cm
            RCLCPP_INFO(ctx_->get_logger(), "  [TRANSPORT] 阶段1：X方向预搬运 +0.20m");
            if (!perform_transport_shift(0.20, 0.0, "X+0.20m")) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            // 阶段2：再执行原Y方向搬运
            RCLCPP_INFO(ctx_->get_logger(), "  [TRANSPORT] 阶段2：Y方向搬运 +0.10m");
            if (!perform_transport_shift(0.0, 0.10, "Y+0.10m")) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            if (ctx_->enable_rotate_) {
                // 若运输后旋转不可行，则自动沿X轴小步回退，找到“既搬运又可旋转”的安全走廊。
                if (!rotate_preview_feasible()) {
                    RCLCPP_WARN(ctx_->get_logger(),
                        "  [TRANSPORT] 当前位姿旋转预检失败，开始X轴回退寻优（每步-0.02m）");

                    const int max_backoff_steps = 8;  // 最多回退16cm，保留至少部分X搬运收益
                    double total_backoff = 0.0;
                    bool found_safe_pose = false;

                    for (int step = 1; step <= max_backoff_steps; ++step) {
                        if (!perform_transport_shift(-0.02, 0.0, "X回退寻优")) {
                            RCLCPP_ERROR(ctx_->get_logger(), "  [TRANSPORT] X回退寻优失败（step=%d）", step);
                            ctx_->current_state_ = TaskState::ERROR;
                            return;
                        }

                        total_backoff += 0.02;

                        if (rotate_preview_feasible()) {
                            found_safe_pose = true;
                            RCLCPP_INFO(ctx_->get_logger(),
                                "  [TRANSPORT] 寻优成功：累计回退X=%.2fm，净X搬运=%.2fm",
                                total_backoff, 0.20 - total_backoff);
                            break;
                        }
                    }

                    if (!found_safe_pose) {
                        RCLCPP_ERROR(ctx_->get_logger(),
                            "  [TRANSPORT] 回退后仍无法保证旋转可行，请减小X预搬运量或提高lift_height");
                        ctx_->current_state_ = TaskState::ERROR;
                        return;
                    }
                }

                RCLCPP_INFO(ctx_->get_logger(), "✓ 平移完成，进入旋转阶段\n");
                ctx_->current_state_ = TaskState::ROTATE;
            } else {
                RCLCPP_INFO(ctx_->get_logger(), "✓ 平移完成（旋转已禁用），直接进入下降放置阶段\n");
                ctx_->current_state_ = TaskState::DESCEND;
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "TRANSPORT 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::executeRotate()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: ROTATE] 旋转物体方向（Y轴→X轴）...");
        RCLCPP_INFO(ctx_->get_logger(), "  策略: 分段执行(3×30°) + 密集航点(1°/步) + TOTG（无段间刷新）");
        RCLCPP_INFO(ctx_->get_logger(), "  [原理] 分段执行: 每30°重新获取实际关节状态，补偿跟踪误差");
        RCLCPP_INFO(ctx_->get_logger(), "  [原理] 不刷新夹爪: simGripperGrasp保持at(0)=0(最大夹持力)，段间刷新会触发simGripperMove导致松手");
        RCLCPP_INFO(ctx_->get_logger(), "  [原理] Joint7偏置: 引导手腕关节主动承担Z轴旋转，避免中段/根部关节漂移翻转");
        
        try {
            // ===== Phase 1: 计算固定几何参数（整个旋转过程使用同一参考基准） =====
            ctx_->dual_arm_->setStartStateToCurrentState();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            auto left_current = ctx_->left_arm_->getCurrentPose();
            auto right_current = ctx_->right_arm_->getCurrentPose();
            
            // 杆件中心（两臂末端中点）
            double center_x = (left_current.pose.position.x + right_current.pose.position.x) / 2.0;
            double center_y = (left_current.pose.position.y + right_current.pose.position.y) / 2.0;
            double center_z = (left_current.pose.position.z + right_current.pose.position.z) / 2.0;
            
            // 夹持半径和各臂初始角度
            double left_dx0 = left_current.pose.position.x - center_x;
            double left_dy0 = left_current.pose.position.y - center_y;
            double radius = std::sqrt(left_dx0 * left_dx0 + left_dy0 * left_dy0);
            double left_init_angle = std::atan2(left_dy0, left_dx0);
            
            double right_dx0 = right_current.pose.position.x - center_x;
            double right_dy0 = right_current.pose.position.y - center_y;
            double right_init_angle = std::atan2(right_dy0, right_dx0);
            
            RCLCPP_INFO(ctx_->get_logger(), "  杆件中心: (%.3f, %.3f, %.3f)", center_x, center_y, center_z);
            RCLCPP_INFO(ctx_->get_logger(), "  夹持半径: %.3f m", radius);
            RCLCPP_INFO(ctx_->get_logger(), "  左臂初始角度: %.1f°, 右臂初始角度: %.1f°",
                        left_init_angle * 180.0 / M_PI, right_init_angle * 180.0 / M_PI);
            
            // 保存起始姿态（作为所有航点的姿态旋转基准）
            geometry_msgs::msg::Pose left_start_pose = left_current.pose;
            geometry_msgs::msg::Pose right_start_pose = right_current.pose;
            
            // ===== Phase 2: 分段参数 =====
            const int num_segments = 3;
            const int wp_per_seg = 30;        // 每段30个航点 → 每步1°（加密提升双臂协调性）
            const int total_wp = num_segments * wp_per_seg;
            const double total_angle = M_PI / 2.0;
            const double step_angle_deg = std::abs(total_angle) * 180.0 / M_PI / total_wp;  // 每步角度(度)
            
            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
            auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);
            
            // 获取关节模型（用于极限检测）
            const auto& left_joint_models = left_jmg->getActiveJointModels();
            const auto& right_joint_models = right_jmg->getActiveJointModels();
            
            // IK约束回调工厂: 拒绝关节变化超过阈值的IK解
            auto make_constraint = [](const std::vector<double>& seed_vals, double max_change)
                -> moveit::core::GroupStateValidityCallbackFn {
                return [seed_vals, max_change](
                    moveit::core::RobotState* /*state*/,
                    const moveit::core::JointModelGroup* /*group*/,
                    const double* joint_values) -> bool {
                    for (size_t j = 0; j < seed_vals.size(); ++j) {
                        double diff = std::abs(joint_values[j] - seed_vals[j]);
                        if (diff > M_PI) diff = 2.0 * M_PI - diff;
                        if (diff > max_change) return false;
                    }
                    return true;
                };
            };
            
            // 关节极限检测函数: 当关节逼近极限时发出警告
            const double limit_margin = 0.15;  // 距极限 0.15 rad (8.6°) 时警告
            auto check_joint_limits = [&](const std::string& arm_name,
                const std::vector<double>& vals,
                const std::vector<const moveit::core::JointModel*>& joint_models_vec,
                int waypoint_idx) {
                for (size_t j = 0; j < joint_models_vec.size() && j < vals.size(); ++j) {
                    const auto& bounds = joint_models_vec[j]->getVariableBounds();
                    if (!bounds.empty()) {
                        double lo = bounds[0].min_position_;
                        double hi = bounds[0].max_position_;
                        if (vals[j] < lo + limit_margin || vals[j] > hi - limit_margin) {
                            RCLCPP_WARN(ctx_->get_logger(),
                                "    航点%d %s J%zu=%.3frad(%.1f°) 逼近极限[%.1f°, %.1f°]!",
                                waypoint_idx, arm_name.c_str(), j + 1,
                                vals[j], vals[j] * 180.0 / M_PI,
                                lo * 180.0 / M_PI, hi * 180.0 / M_PI);
                        }
                    }
                }
            };
            
            RCLCPP_INFO(ctx_->get_logger(), "  分段参数: %d段×%d航点, 每步%.1f°, 总计%d航点",
                        num_segments, wp_per_seg, step_angle_deg, total_wp);
            
            // 打印起始joint7值
            {
                auto init_state = ctx_->dual_arm_->getCurrentState();
                std::vector<double> init_left, init_right;
                init_state->copyJointGroupPositions(left_jmg, init_left);
                init_state->copyJointGroupPositions(right_jmg, init_right);
                if (init_left.size() > 6 && init_right.size() > 6) {
                    RCLCPP_INFO(ctx_->get_logger(), "  起始 J7: 左=%.1f°, 右=%.1f°",
                                init_left[6] * 180.0 / M_PI, init_right[6] * 180.0 / M_PI);
                }
            }
            
            // ===== Phase 3: 分段执行 =====
            for (int seg = 0; seg < num_segments; ++seg) {
                int wp_global_start = seg * wp_per_seg;
                int wp_global_end = wp_global_start + wp_per_seg;
                double seg_start_deg = total_angle * wp_global_start / total_wp * 180.0 / M_PI;
                double seg_end_deg = total_angle * wp_global_end / total_wp * 180.0 / M_PI;
                
                RCLCPP_INFO(ctx_->get_logger(), "  === 分段 %d/%d (%.0f° → %.0f°) ===",
                            seg + 1, num_segments, seg_start_deg, seg_end_deg);
                
                // 从实际机器人状态开始（自动补偿前一段的跟踪误差）
                ctx_->dual_arm_->setStartStateToCurrentState();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                
                auto current_state = ctx_->dual_arm_->getCurrentState();
                auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
                traj->addSuffixWayPoint(*current_state, 0.0);
                
                auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
                
                for (int i = wp_global_start + 1; i <= wp_global_end; ++i) {
                    double angle = total_angle * i / total_wp;
                    
                    // ----- 左臂目标位姿（圆弧上的点，基于固定几何参数）-----
                    double left_new_angle = left_init_angle + angle;
                    geometry_msgs::msg::Pose left_target = left_start_pose;
                    left_target.position.x = center_x + radius * std::cos(left_new_angle);
                    left_target.position.y = center_y + radius * std::sin(left_new_angle);
                    left_target.position.z = center_z;
                    
                    // ----- 右臂目标位姿（圆弧上对称点）-----
                    double right_new_angle = right_init_angle + angle;
                    geometry_msgs::msg::Pose right_target = right_start_pose;
                    right_target.position.x = center_x + radius * std::cos(right_new_angle);
                    right_target.position.y = center_y + radius * std::sin(right_new_angle);
                    right_target.position.z = center_z;
                    
                    // ----- 姿态：绕Z轴累积旋转 -----
                    tf2::Quaternion q_rot;
                    q_rot.setRPY(0, 0, angle);
                    
                    tf2::Quaternion q_left_orig, q_right_orig;
                    tf2::fromMsg(left_start_pose.orientation, q_left_orig);
                    tf2::fromMsg(right_start_pose.orientation, q_right_orig);
                    
                    tf2::Quaternion q_left_new = q_rot * q_left_orig;
                    q_left_new.normalize();
                    left_target.orientation = tf2::toMsg(q_left_new);
                    
                    tf2::Quaternion q_right_new = q_rot * q_right_orig;
                    q_right_new.normalize();
                    right_target.orientation = tf2::toMsg(q_right_new);
                    
                    // ----- Joint7 种子偏置（引导手腕承担旋转）-----
                    // TCP Z轴向下(Roll=180)，joint7偏置需跟随旋转方向变号，
                    // 避免反向旋转时仍用同号偏置把IK推向错误分支。
                    std::vector<double> left_seed_vals, right_seed_vals;
                    seed_state->copyJointGroupPositions(left_jmg, left_seed_vals);
                    seed_state->copyJointGroupPositions(right_jmg, right_seed_vals);
                    
                    double tcp_z_delta = total_angle / total_wp;  // 约 ±0.017 rad (1°/步)，符号跟随旋转方向
                    if (left_seed_vals.size() > 6) left_seed_vals[6] += tcp_z_delta;
                    if (right_seed_vals.size() > 6) right_seed_vals[6] += tcp_z_delta;
                    
                    auto waypoint_state = std::make_shared<moveit::core::RobotState>(*seed_state);
                    waypoint_state->setJointGroupPositions(left_jmg, left_seed_vals);
                    waypoint_state->setJointGroupPositions(right_jmg, right_seed_vals);
                    
                    // ----- 三级IK策略（约束从严到宽逐级放松）-----
                    // Tier 1: 严格约束 (0.3 rad ≈ 17.2°/关节/步)
                    bool left_ik = waypoint_state->setFromIK(left_jmg, left_target, 0.1,
                        make_constraint(left_seed_vals, 0.3));
                    
                    // Tier 2: 放松约束 (0.8 rad ≈ 45.8°)
                    if (!left_ik) {
                        waypoint_state->setJointGroupPositions(left_jmg, left_seed_vals);
                        left_ik = waypoint_state->setFromIK(left_jmg, left_target, 1.0,
                            make_constraint(left_seed_vals, 0.8));
                    }
                    
                    // Tier 3: 无约束（最后手段，可能跳变）
                    if (!left_ik) {
                        waypoint_state->setJointGroupPositions(left_jmg, left_seed_vals);
                        left_ik = waypoint_state->setFromIK(left_jmg, left_target, 3.0);
                        if (left_ik) RCLCPP_WARN(ctx_->get_logger(),
                            "    航点%d 左臂IK使用无约束解（可能跳变！）", i);
                    }
                    
                    bool right_ik = waypoint_state->setFromIK(right_jmg, right_target, 0.1,
                        make_constraint(right_seed_vals, 0.3));
                    
                    if (!right_ik) {
                        waypoint_state->setJointGroupPositions(right_jmg, right_seed_vals);
                        right_ik = waypoint_state->setFromIK(right_jmg, right_target, 1.0,
                            make_constraint(right_seed_vals, 0.8));
                    }
                    
                    if (!right_ik) {
                        waypoint_state->setJointGroupPositions(right_jmg, right_seed_vals);
                        right_ik = waypoint_state->setFromIK(right_jmg, right_target, 3.0);
                        if (right_ik) RCLCPP_WARN(ctx_->get_logger(),
                            "    航点%d 右臂IK使用无约束解（可能跳变！）", i);
                    }
                    
                    if (!left_ik || !right_ik) {
                        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 航点 %d/%d IK求解失败: 左=%s, 右=%s",
                                     i, total_wp, left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                        ctx_->current_state_ = TaskState::ERROR;
                        return;
                    }
                    
                    // ----- 关节跳变 + 极限检测 -----
                    std::vector<double> new_left_vals, new_right_vals;
                    waypoint_state->copyJointGroupPositions(left_jmg, new_left_vals);
                    waypoint_state->copyJointGroupPositions(right_jmg, new_right_vals);
                    
                    double max_left_jump = 0, max_right_jump = 0;
                    for (size_t j = 0; j < left_seed_vals.size(); ++j) {
                        double diff = std::abs(new_left_vals[j] - left_seed_vals[j]);
                        max_left_jump = std::max(max_left_jump, diff);
                    }
                    for (size_t j = 0; j < right_seed_vals.size(); ++j) {
                        double diff = std::abs(new_right_vals[j] - right_seed_vals[j]);
                        max_right_jump = std::max(max_right_jump, diff);
                    }
                    
                    if (max_left_jump > 0.2 || max_right_jump > 0.2) {
                        RCLCPP_WARN(ctx_->get_logger(),
                            "    航点%d 跳变: L=%.2f°, R=%.2f°",
                            i, max_left_jump * 180.0 / M_PI, max_right_jump * 180.0 / M_PI);
                    }
                    
                    // 关节极限检测
                    check_joint_limits("左", new_left_vals, left_joint_models, i);
                    check_joint_limits("右", new_right_vals, right_joint_models, i);
                    
                    waypoint_state->update();
                    traj->addSuffixWayPoint(*waypoint_state, 0.0);
                    *seed_state = *waypoint_state;
                    
                    // 每段首末航点 + 每15个航点打印详情
                    if (i == wp_global_start + 1 || i == wp_global_end || i % 15 == 0) {
                        RCLCPP_INFO(ctx_->get_logger(),
                            "    航点 %d/%d (%.1f°): 左(%.3f,%.3f,%.3f) 右(%.3f,%.3f,%.3f) J7:L=%.1f° R=%.1f° 跳变:L=%.1f° R=%.1f°",
                            i, total_wp, angle * 180.0 / M_PI,
                            left_target.position.x, left_target.position.y, left_target.position.z,
                            right_target.position.x, right_target.position.y, right_target.position.z,
                            new_left_vals.size() > 6 ? new_left_vals[6] * 180.0 / M_PI : 0.0,
                            new_right_vals.size() > 6 ? new_right_vals[6] * 180.0 / M_PI : 0.0,
                            max_left_jump * 180.0 / M_PI, max_right_jump * 180.0 / M_PI);
                    }
                }
                
                // ----- TOTG时间参数化 -----
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                double vel_scale = 0.06;
                double acc_scale = 0.06;
                bool time_ok = totg.computeTimeStamps(*traj, vel_scale, acc_scale);
                RCLCPP_INFO(ctx_->get_logger(), "    TOTG缩放: vel=%.4f, acc=%.4f", vel_scale, acc_scale);
                
                if (!time_ok) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 分段 %d TOTG失败", seg + 1);
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }
                
                RCLCPP_INFO(ctx_->get_logger(), "    TOTG: 时长=%.2fs, 轨迹点=%zu",
                            traj->getDuration(), traj->getWayPointCount());
                
                // ----- 构建Plan并执行（不走OMPL） -----
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                traj->getRobotTrajectoryMsg(plan.trajectory_);
                plan.planning_time_ = 0.0;
                
                auto exec_result = ctx_->dual_arm_->execute(plan);
                if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 分段 %d 执行失败 (error: %d)",
                                 seg + 1, exec_result.val);
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }
                
                RCLCPP_INFO(ctx_->get_logger(), "  ✓ 分段 %d/%d 执行完成 (%.0f° → %.0f°)",
                            seg + 1, num_segments, seg_start_deg, seg_end_deg);
                
                // ----- 段间稳定等待（不刷新夹爪）-----
                // 关键发现: simGripperGrasp后at(0)=0(最大夹持力)，kCurrentWidth为负值(-0.014)
                // 若发送controlGripper(ctx_->gripper_close_pos_=0.010)，target_width=0.020 > kCurrentWidth
                // → 触发simGripperMove(OPEN方向!)而非simGripperGrasp，导致at(0)从0跳到63
                // → 夹持力骤降，铝棒脱落。因此删除段间夹爪刷新，保持simGripperGrasp的持续夹持
                if (seg < num_segments - 1) {
                    RCLCPP_INFO(ctx_->get_logger(), "  [段间等待] 等待300ms稳定...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ 段间稳定完成");
                }
            }
            
            // 旋转后稳定等待
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // 打印最终joint7值
            {
                auto final_state = ctx_->dual_arm_->getCurrentState();
                std::vector<double> final_left, final_right;
                final_state->copyJointGroupPositions(left_jmg, final_left);
                final_state->copyJointGroupPositions(right_jmg, final_right);
                if (final_left.size() > 6 && final_right.size() > 6) {
                    RCLCPP_INFO(ctx_->get_logger(), "  最终 J7: 左=%.1f°, 右=%.1f°",
                                final_left[6] * 180.0 / M_PI, final_right[6] * 180.0 / M_PI);
                }
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ 旋转完成（杆件现在沿X轴方向），进入下降阶段\n");
            ctx_->current_state_ = TaskState::DESCEND;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "ROTATE 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::executeDescend()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: DESCEND] 下降准备放置...");
        RCLCPP_INFO(ctx_->get_logger(), "  策略: 密集航点插值 + TOTG时间参数化（绕过OMPL，保证双臂严格同步）");
        
        try {
            ctx_->dual_arm_->setStartStateToCurrentState();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            auto left_current = ctx_->left_arm_->getCurrentPose();
            auto right_current = ctx_->right_arm_->getCurrentPose();
            
            // 旋转模式下：使用较高放置高度+X回避，给旋转后构型留安全间隙。
            // 非旋转模式下：直接使用抓取时的高度，确保铝棒直接贴合地面。
            double place_z = ctx_->enable_rotate_ ? (ctx_->grasp_height_ + 0.02) : ctx_->grasp_height_;
            double descend_shift_x = ctx_->enable_rotate_ ? -0.10 : 0.0;

            double left_delta_z = place_z - left_current.pose.position.z;
            double right_delta_z = place_z - right_current.pose.position.z;
            double left_delta_x = descend_shift_x;
            double right_delta_x = descend_shift_x;
            
            RCLCPP_INFO(ctx_->get_logger(), "  当前Z: 左=%.3f, 右=%.3f → 目标Z: %.3f (下降约%.3fm)",
                        left_current.pose.position.z, right_current.pose.position.z, place_z,
                        std::abs((left_delta_z + right_delta_z) / 2.0));
            RCLCPP_INFO(ctx_->get_logger(), "  DESCEND阶段附加平移: 双臂X方向 +%.3fm", descend_shift_x);
            if (!ctx_->enable_rotate_) {
                RCLCPP_INFO(ctx_->get_logger(), "  [DESCEND] 当前为无旋转模式：保持铝棒原朝向（沿Y轴）放置");
            }
            
            // ===== 密集航点 =====
            // 10个航点覆盖约11cm下降，每步约1.1cm
            int num_waypoints = 10;
            
            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
            auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);
            
            auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
            
            // 添加起始航点
            auto current_state = ctx_->dual_arm_->getCurrentState();
            traj->addSuffixWayPoint(*current_state, 0.0);
            
            auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
            
            geometry_msgs::msg::Pose left_start = left_current.pose;
            geometry_msgs::msg::Pose right_start = right_current.pose;
            
            RCLCPP_INFO(ctx_->get_logger(), "  生成%d个密集航点...", num_waypoints);
            
            for (int i = 1; i <= num_waypoints; ++i) {
                double frac = static_cast<double>(i) / num_waypoints;
                
                // 左臂：保持Y和姿态不变，线性插值X/Z
                geometry_msgs::msg::Pose left_target = left_start;
                left_target.position.x = left_start.position.x + left_delta_x * frac;
                left_target.position.z = left_start.position.z + left_delta_z * frac;
                
                // 右臂：保持Y和姿态不变，线性插值X/Z
                geometry_msgs::msg::Pose right_target = right_start;
                right_target.position.x = right_start.position.x + right_delta_x * frac;
                right_target.position.z = right_start.position.z + right_delta_z * frac;
                
                // IK求解（加一致性约束，防止关节跳变 - 与ROTATE相同策略）
                std::vector<double> left_seed_vals, right_seed_vals;
                seed_state->copyJointGroupPositions(left_jmg, left_seed_vals);
                seed_state->copyJointGroupPositions(right_jmg, right_seed_vals);
                
                auto waypoint_state = std::make_shared<moveit::core::RobotState>(*seed_state);
                
                auto make_constraint = [](const std::vector<double>& seed_vals, double max_change)
                    -> moveit::core::GroupStateValidityCallbackFn {
                    return [seed_vals, max_change](
                        moveit::core::RobotState*, const moveit::core::JointModelGroup*,
                        const double* joint_values) -> bool {
                        for (size_t j = 0; j < seed_vals.size(); ++j) {
                            double diff = std::abs(joint_values[j] - seed_vals[j]);
                            if (diff > M_PI) diff = 2.0 * M_PI - diff;
                            if (diff > max_change) return false;
                        }
                        return true;
                    };
                };
                
                // 左臂：三级约束递减
                bool left_ik = waypoint_state->setFromIK(left_jmg, left_target, 0.1,
                    make_constraint(left_seed_vals, 0.5));
                if (!left_ik) {
                    waypoint_state->setJointGroupPositions(left_jmg, left_seed_vals);
                    left_ik = waypoint_state->setFromIK(left_jmg, left_target, 1.0,
                        make_constraint(left_seed_vals, 1.0));
                }
                if (!left_ik) {
                    waypoint_state->setJointGroupPositions(left_jmg, left_seed_vals);
                    left_ik = waypoint_state->setFromIK(left_jmg, left_target, 3.0);
                    if (left_ik) RCLCPP_WARN(ctx_->get_logger(),
                        "  DESCEND航点%d 左臂无约束解（可能跳变）", i);
                }
                
                // 右臂：三级约束递减
                bool right_ik = waypoint_state->setFromIK(right_jmg, right_target, 0.1,
                    make_constraint(right_seed_vals, 0.5));
                if (!right_ik) {
                    waypoint_state->setJointGroupPositions(right_jmg, right_seed_vals);
                    right_ik = waypoint_state->setFromIK(right_jmg, right_target, 1.0,
                        make_constraint(right_seed_vals, 1.0));
                }
                if (!right_ik) {
                    waypoint_state->setJointGroupPositions(right_jmg, right_seed_vals);
                    right_ik = waypoint_state->setFromIK(right_jmg, right_target, 3.0);
                    if (right_ik) RCLCPP_WARN(ctx_->get_logger(),
                        "  DESCEND航点%d 右臂无约束解（可能跳变）", i);
                }
                
                if (!left_ik || !right_ik) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 航点 %d/%d IK求解失败: 左=%s, 右=%s",
                                 i, num_waypoints, left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }
                
                waypoint_state->update();
                traj->addSuffixWayPoint(*waypoint_state, 0.0);
                *seed_state = *waypoint_state;
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 全部%d个航点IK求解成功", num_waypoints);
            
            // ===== TOTG时间参数化 =====
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            // 修复：由于下降过程中手臂向下弯曲产生惯性，加上MuJoCo中摩擦力有限，
            // 较大的加速度会导致铝棒由于惯性直接在两指之间滑动脱落现象！同时如果手肘与杆件发生轻微物理擦碰也会加剧脱落。
            // weld约束保证铝棒不会滑落，可以适当提升下降速度。
            bool time_ok = totg.computeTimeStamps(*traj, 0.06, 0.06);
            
            if (!time_ok) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ TOTG时间参数化失败");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ TOTG时间参数化完成: 总时长=%.2f秒, 轨迹点数=%zu",
                        traj->getDuration(), traj->getWayPointCount());
            
            // ===== 构建Plan并直接执行 =====
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            traj->getRobotTrajectoryMsg(plan.trajectory_);
            plan.planning_time_ = 0.0;
            
            RCLCPP_INFO(ctx_->get_logger(), "  执行下降轨迹（绕过OMPL，双臂严格同步）...");
            auto exec_result = ctx_->dual_arm_->execute(plan);
            
            if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 下降轨迹执行失败 (error: %d)", exec_result.val);
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ 下降完成，进入放置阶段\n");
            ctx_->current_state_ = TaskState::PLACE;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "DESCEND 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::executePlace()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: PLACE] 放置物体（GRASP逆操作）...");
        
        try {
            // ========== Step A: 去激活MuJoCo weld约束 + 打开夹爪释放物体 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step A: 去激活weld约束并打开夹爪释放物体");
            
            // 先去激活weld，再开夹爪，让物体自然落下
            {
                auto weld_msg = std_msgs::msg::Bool();
                weld_msg.data = false;
                ctx_->weld_pub_->publish(weld_msg);
                RCLCPP_INFO(ctx_->get_logger(), "  [MuJoCo] Weld约束已去激活");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            ctx_->grippers_->controlDualGrippers(0.04);         // 完全同步打开
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 夹爪已同步打开");
            
            // 等待物理引擎稳定（杆件落地）
            RCLCPP_INFO(ctx_->get_logger(), "  [等待] 物理引擎稳定中（1.5秒）...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            
            // ========== Step B: 解除附着并重新添加为世界碰撞物体 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step B: 解除物体附着，重新添加为世界碰撞物体");
            ctx_->arms_->detachAndReplaceObject();
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 物体已放置");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // ========== Step B2: 恢复ACM（禁止夹爪与铝棒碰撞）==========
            // GRASP Step C 中调用了 ctx_->arms_->allow_gripper_collision(true) 允许夹爪碰撞铝棒
            // 放置完成后必须恢复为禁止碰撞，保证后续阶段的碰撞检测正确性
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step B2: 恢复ACM（禁止夹爪-铝棒碰撞）");
            ctx_->arms_->allow_gripper_collision(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // ========== Step C: 水平撤退 + 垂直抬升（合并为TOTG密集航点）==========
            // 旋转后：左臂在+X端，右臂在-X端，杆件沿X方向
            // 撤退方向：左臂向+X，右臂向-X，同时抬升到安全高度
            // 使用TOTG避免OMPL在紧凑构型下的规划失败（之前3次测试2次OMPL失败）
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step C+D: 水平撤退+垂直抬升（TOTG密集航点）");
            
            ctx_->dual_arm_->setStartStateToCurrentState();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            auto left_current = ctx_->left_arm_->getCurrentPose();
            auto right_current = ctx_->right_arm_->getCurrentPose();
            
            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
            auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);
            
            // 目标位置：水平撤退10cm + 垂直抬升到安全高度
            double retreat_offset = 0.10;
            double safe_z = ctx_->lift_height_;  // 0.40m
            
            geometry_msgs::msg::Pose left_final = left_current.pose;
            left_final.position.x += retreat_offset;
            left_final.position.z = safe_z;
            
            geometry_msgs::msg::Pose right_final = right_current.pose;
            right_final.position.x -= retreat_offset;
            right_final.position.z = safe_z;
            
            RCLCPP_INFO(ctx_->get_logger(), "    左臂: (%.3f,%.3f) → (%.3f,%.3f)",
                        left_current.pose.position.x, left_current.pose.position.z,
                        left_final.position.x, left_final.position.z);
            RCLCPP_INFO(ctx_->get_logger(), "    右臂: (%.3f,%.3f) → (%.3f,%.3f)",
                        right_current.pose.position.x, right_current.pose.position.z,
                        right_final.position.x, right_final.position.z);
            
            // 生成密集航点（10步）
            int num_retreat_wp = 10;
            auto retreat_traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
            
            auto retreat_current_state = ctx_->dual_arm_->getCurrentState();
            retreat_traj->addSuffixWayPoint(*retreat_current_state, 0.0);
            
            auto retreat_seed = std::make_shared<moveit::core::RobotState>(*retreat_current_state);
            
            auto make_constraint = [](const std::vector<double>& seed_vals, double max_change)
                -> moveit::core::GroupStateValidityCallbackFn {
                return [seed_vals, max_change](
                    moveit::core::RobotState*, const moveit::core::JointModelGroup*,
                    const double* joint_values) -> bool {
                    for (size_t j = 0; j < seed_vals.size(); ++j) {
                        double diff = std::abs(joint_values[j] - seed_vals[j]);
                        if (diff > M_PI) diff = 2.0 * M_PI - diff;
                        if (diff > max_change) return false;
                    }
                    return true;
                };
            };
            
            bool retreat_ik_ok = true;
            for (int i = 1; i <= num_retreat_wp; ++i) {
                double frac = static_cast<double>(i) / num_retreat_wp;
                
                geometry_msgs::msg::Pose left_wp = left_current.pose;
                left_wp.position.x += retreat_offset * frac;
                left_wp.position.z += (safe_z - left_current.pose.position.z) * frac;
                
                geometry_msgs::msg::Pose right_wp = right_current.pose;
                right_wp.position.x -= retreat_offset * frac;
                right_wp.position.z += (safe_z - right_current.pose.position.z) * frac;
                
                std::vector<double> left_seed_vals, right_seed_vals;
                retreat_seed->copyJointGroupPositions(left_jmg, left_seed_vals);
                retreat_seed->copyJointGroupPositions(right_jmg, right_seed_vals);
                
                auto wp_state = std::make_shared<moveit::core::RobotState>(*retreat_seed);
                
                // 三级约束IK
                bool left_ik = wp_state->setFromIK(left_jmg, left_wp, 0.1,
                    make_constraint(left_seed_vals, 0.5));
                if (!left_ik) {
                    wp_state->setJointGroupPositions(left_jmg, left_seed_vals);
                    left_ik = wp_state->setFromIK(left_jmg, left_wp, 1.0,
                        make_constraint(left_seed_vals, 1.0));
                }
                if (!left_ik) {
                    wp_state->setJointGroupPositions(left_jmg, left_seed_vals);
                    left_ik = wp_state->setFromIK(left_jmg, left_wp, 3.0);
                }
                
                bool right_ik = wp_state->setFromIK(right_jmg, right_wp, 0.1,
                    make_constraint(right_seed_vals, 0.5));
                if (!right_ik) {
                    wp_state->setJointGroupPositions(right_jmg, right_seed_vals);
                    right_ik = wp_state->setFromIK(right_jmg, right_wp, 1.0,
                        make_constraint(right_seed_vals, 1.0));
                }
                if (!right_ik) {
                    wp_state->setJointGroupPositions(right_jmg, right_seed_vals);
                    right_ik = wp_state->setFromIK(right_jmg, right_wp, 3.0);
                }
                
                if (!left_ik || !right_ik) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 撤退航点 %d/%d IK失败: 左=%s, 右=%s",
                                 i, num_retreat_wp, left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                    retreat_ik_ok = false;
                    break;
                }
                
                wp_state->update();
                retreat_traj->addSuffixWayPoint(*wp_state, 0.0);
                *retreat_seed = *wp_state;
            }
            
            if (retreat_ik_ok) {
                trajectory_processing::TimeOptimalTrajectoryGeneration retreat_totg;
                bool time_ok = retreat_totg.computeTimeStamps(*retreat_traj, 0.5, 0.5);
                
                if (time_ok) {
                    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
                    retreat_traj->getRobotTrajectoryMsg(retreat_plan.trajectory_);
                    retreat_plan.planning_time_ = 0.0;
                    
                    RCLCPP_INFO(ctx_->get_logger(), "    TOTG完成: 时长=%.2fs, 轨迹点=%zu",
                                retreat_traj->getDuration(), retreat_traj->getWayPointCount());
                    
                    auto exec_result = ctx_->dual_arm_->execute(retreat_plan);
                    if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_INFO(ctx_->get_logger(), "  ✓ 水平撤退+垂直抬升完成");
                    } else {
                        RCLCPP_WARN(ctx_->get_logger(), "  ! 撤退轨迹执行失败（继续撤退）");
                    }
                } else {
                    RCLCPP_WARN(ctx_->get_logger(), "  ! TOTG参数化失败（继续撤退）");
                }
            } else {
                RCLCPP_WARN(ctx_->get_logger(), "  ! 撤退IK部分失败（跳过撤退，直接进入RETREAT）");
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "✓ 放置完成，进入撤退阶段\n");
            ctx_->current_state_ = TaskState::RETREAT;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "PLACE 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

void DualArmController::detachAndReplaceObject()
    {
        using namespace std::chrono_literals;
        
        auto planning_scene_pub = ctx_->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/planning_scene", rclcpp::QoS(10).transient_local());
        std::this_thread::sleep_for(100ms);
        
        // 获取当前双臂位置，推算杆件中心
        auto left_current = ctx_->left_arm_->getCurrentPose();
        auto right_current = ctx_->right_arm_->getCurrentPose();
        
        double rod_center_x = (left_current.pose.position.x + right_current.pose.position.x) / 2.0;
        double rod_center_y = (left_current.pose.position.y + right_current.pose.position.y) / 2.0;
        double rod_center_z = 0.02;  // 放置到地面（与初始高度相同）
        
        RCLCPP_INFO(ctx_->get_logger(), "    杆件放置位置: (%.3f, %.3f, %.3f)，方向沿%s轴",
                rod_center_x, rod_center_y, rod_center_z, ctx_->enable_rotate_ ? "X" : "Y");
        
        // ========== 步骤1: 解除附着 ==========
        // 修复：之前只是发了REMOVE消息到话题，但由于ROS 2 MoveIt架构中话题处理的延迟或覆盖，
        // 常常无法正确更新 attached_objects，导致MoveIt依然认为物体粘在手上（RETREAT时发生碰撞）。
        // 改用高层API detachObject 会通过 Service 进行可靠同步。
        bool detach_success = ctx_->left_arm_->detachObject("aluminum_rod");
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 执行解除附着: %s", detach_success ? "成功" : "失败");
        std::this_thread::sleep_for(300ms);
        
        // 为了绝对安全，额外从 MoveGroupInterface 中显式调用
        ctx_->dual_arm_->detachObject("aluminum_rod");
        
        // ========== 步骤2: 重新添加为世界碰撞物体（沿X轴方向）==========
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "base_link";
        collision_object.id = "aluminum_rod";
        
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.04, 0.40, 0.04}; 
        
        geometry_msgs::msg::Pose rod_pose;
        rod_pose.position.x = rod_center_x;
        rod_pose.position.y = rod_center_y;
        rod_pose.position.z = rod_center_z;
        
        tf2::Quaternion q_rod;
        q_rod.setRPY(0, 0, ctx_->enable_rotate_ ? M_PI / 2.0 : 0.0);
        rod_pose.orientation = tf2::toMsg(q_rod);
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(rod_pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        
        // 修复：使用 PlanningSceneInterface 的 Apply 方法强制同步更新世界物体
        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(collision_object);
        
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 重新添加铝棒为世界碰撞物体（沿%s轴方向）",
                ctx_->enable_rotate_ ? "X" : "Y");
        std::this_thread::sleep_for(300ms);
        
        // ========== 步骤3: 验证结果 ==========
        auto objects = psi.getObjects();
        auto attached_objects = psi.getAttachedObjects();
        
        RCLCPP_INFO(ctx_->get_logger(), "    世界物体: %zu (期望≥1), 附着物体: %zu (期望=0)",
                    objects.size(), attached_objects.size());
        
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 解除附着并重新放置完成");
    }

void DualArmController::executeRetreat()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: RETREAT] 撤退至初始位置...");
        
        try {
            // ===== 直接使用 TOTG 关节空间插值撤退 =====
            // 彻底放弃 OMPL 规划：RETREAT 起始状态下铝棒作为世界碰撞体
            // 与hand link碰撞，导致 OMPL 起始状态校验必然失败。
            // TOTG 绕过碰撞检测，直接在关节空间线性插值到初始位置。
            
            // 检查是否有保存的初始关节位置
            if (ctx_->initial_left_joints_.empty() || ctx_->initial_right_joints_.empty()) {
                RCLCPP_WARN(ctx_->get_logger(), "  ! 无初始关节角记录，使用Franka默认ready位置");
                ctx_->initial_left_joints_ = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
                ctx_->initial_right_joints_ = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
            }
            
            RCLCPP_INFO(ctx_->get_logger(), "  [TOTG] 关节空间直接插值到初始位置...");
            RCLCPP_INFO(ctx_->get_logger(), "  左臂目标: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        ctx_->initial_left_joints_[0], ctx_->initial_left_joints_[1], ctx_->initial_left_joints_[2],
                        ctx_->initial_left_joints_[3], ctx_->initial_left_joints_[4], ctx_->initial_left_joints_[5],
                        ctx_->initial_left_joints_[6]);
            RCLCPP_INFO(ctx_->get_logger(), "  右臂目标: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        ctx_->initial_right_joints_[0], ctx_->initial_right_joints_[1], ctx_->initial_right_joints_[2],
                        ctx_->initial_right_joints_[3], ctx_->initial_right_joints_[4], ctx_->initial_right_joints_[5],
                        ctx_->initial_right_joints_[6]);
            
            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto left_jmg = robot_model->getJointModelGroup(ctx_->left_arm_group_);
            auto right_jmg = robot_model->getJointModelGroup(ctx_->right_arm_group_);
            
            ctx_->dual_arm_->setStartStateToCurrentState();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            auto current_st = ctx_->dual_arm_->getCurrentState();
            std::vector<double> curr_left, curr_right;
            current_st->copyJointGroupPositions(left_jmg, curr_left);
            current_st->copyJointGroupPositions(right_jmg, curr_right);
            
            int num_wp = 20;  // 20步线性插值
            auto retreat_traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
            retreat_traj->addSuffixWayPoint(*current_st, 0.0);
            
            for (int i = 1; i <= num_wp; ++i) {
                double frac = static_cast<double>(i) / num_wp;
                auto wp = std::make_shared<moveit::core::RobotState>(*current_st);
                
                std::vector<double> interp_left(curr_left.size()), interp_right(curr_right.size());
                for (size_t j = 0; j < curr_left.size(); ++j)
                    interp_left[j] = curr_left[j] + (ctx_->initial_left_joints_[j] - curr_left[j]) * frac;
                for (size_t j = 0; j < curr_right.size(); ++j)
                    interp_right[j] = curr_right[j] + (ctx_->initial_right_joints_[j] - curr_right[j]) * frac;
                
                wp->setJointGroupPositions(left_jmg, interp_left);
                wp->setJointGroupPositions(right_jmg, interp_right);
                wp->update();
                retreat_traj->addSuffixWayPoint(*wp, 0.0);
            }
            
            trajectory_processing::TimeOptimalTrajectoryGeneration retreat_totg;
            bool time_ok = retreat_totg.computeTimeStamps(*retreat_traj, 0.5, 0.5);
            
            if (time_ok) {
                moveit::planning_interface::MoveGroupInterface::Plan totg_plan;
                retreat_traj->getRobotTrajectoryMsg(totg_plan.trajectory_);
                totg_plan.planning_time_ = 0.0;
                
                RCLCPP_INFO(ctx_->get_logger(), "    TOTG完成: 时长=%.2fs, 轨迹点=%zu",
                            retreat_traj->getDuration(), retreat_traj->getWayPointCount());
                
                auto totg_exec = ctx_->dual_arm_->execute(totg_plan);
                if (totg_exec == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(ctx_->get_logger(), "✓ 撤退完成，任务结束\n");
                    ctx_->current_state_ = TaskState::DONE;
                } else {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ TOTG撤退执行失败");
                    ctx_->current_state_ = TaskState::ERROR;
                }
            } else {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ TOTG时间参数化失败");
                ctx_->current_state_ = TaskState::ERROR;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "RETREAT 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

