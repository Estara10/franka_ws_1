#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#if __has_include(<moveit/trajectory_processing/ruckig_traj_smoothing.h>)
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#define DUAL_ARM_HAS_RUCKIG 1
#else
#define DUAL_ARM_HAS_RUCKIG 0
#endif
#include <cctype>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include "dual_arm_carry_task/dual_arm_controller.hpp"
#include "dual_arm_carry_task/gripper_controller.hpp"

namespace {
constexpr double kReleaseHeightTolerance = 0.004;
constexpr double kReleaseCorrectionVelScale = 0.08;
constexpr double kReleaseCorrectionAccScale = 0.05;
constexpr double kMaxManualTrajectoryJointStepRad = 0.75;
constexpr double kMaxManualTrajectoryHandDistanceDrift = 0.02;
// MoveIt's Ruckig convenience overload uses 1000 rad/s^3 when the model does
// not provide jerk limits. Keep that fallback explicit and shared by every
// manually generated phase instead of relying on an undocumented default.
constexpr double kFallbackJerkLimitRadPerSec3 = 500.0;

struct MotionModeProfile
{
    double transport_vel_scale {1.0};
    double transport_acc_scale {1.0};
    double transport_waypoint_scale {1.0};

    double rotate_vel_scale {1.0};
    double rotate_acc_scale {1.0};
    double rotate_waypoint_scale {1.0};

    double descend_vel_scale {1.0};
    double descend_acc_scale {1.0};
    double descend_waypoint_scale {1.0};

    double place_vel_scale {1.0};
    double place_acc_scale {1.0};
    double place_waypoint_scale {1.0};
};

std::string normalizeControlMode(std::string mode)
{
    // Strip "vision_" prefix for Mode B compatibility (vision_<profile> → <profile>)
    if (mode.rfind("vision_", 0) == 0) {
        mode = mode.substr(7);
    }
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return mode;
}

double clampScale(double value, double lo, double hi)
{
    return std::clamp(value, lo, hi);
}

int scaledWaypointCount(int base, double scale, int min_count)
{
    return std::max(min_count, static_cast<int>(std::round(static_cast<double>(base) * scale)));
}

double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    // Yaw from quaternion (rotation around Z)
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

MotionModeProfile getMotionModeProfile(const std::string& raw_mode)
{
    MotionModeProfile p;
    const std::string mode = normalizeControlMode(raw_mode);

    if (mode == "compliant") {
        p.transport_vel_scale = 0.90;
        p.transport_acc_scale = 0.82;
        p.transport_waypoint_scale = 1.10;

        p.rotate_vel_scale = 0.88;
        p.rotate_acc_scale = 0.80;
        p.rotate_waypoint_scale = 1.15;

        p.descend_vel_scale = 0.88;
        p.descend_acc_scale = 0.80;
        p.descend_waypoint_scale = 1.10;

        p.place_vel_scale = 0.90;
        p.place_acc_scale = 0.82;
        p.place_waypoint_scale = 1.10;
        return p;
    }

    if (mode == "chomp_only") {
        p.transport_vel_scale = 0.84;
        p.transport_acc_scale = 0.74;
        p.transport_waypoint_scale = 1.20;

        p.rotate_vel_scale = 0.80;
        p.rotate_acc_scale = 0.72;
        p.rotate_waypoint_scale = 1.25;

        p.descend_vel_scale = 0.82;
        p.descend_acc_scale = 0.72;
        p.descend_waypoint_scale = 1.20;

        p.place_vel_scale = 0.84;
        p.place_acc_scale = 0.74;
        p.place_waypoint_scale = 1.20;
        return p;
    }

    if (mode == "compliant_chomp") {
        // Mode4 以 CHOMP-only 的平滑骨架为主，只保留轻量柔顺余量，
        // 避免“过密航点 + 过软控制”叠加后在旋转/放置阶段引入跟踪抖动。
        p.transport_vel_scale = 0.86;
        p.transport_acc_scale = 0.76;
        p.transport_waypoint_scale = 1.08;

        p.rotate_vel_scale = 0.80;
        p.rotate_acc_scale = 0.70;
        p.rotate_waypoint_scale = 1.10;

        p.descend_vel_scale = 0.80;
        p.descend_acc_scale = 0.70;
        p.descend_waypoint_scale = 1.12;

        p.place_vel_scale = 0.82;
        p.place_acc_scale = 0.72;
        p.place_waypoint_scale = 1.10;
        return p;
    }

    // rigid 或未知模式均回退到基线剖面。
    return p;
}

bool parameterizeAndSmoothTrajectory(robot_trajectory::RobotTrajectory& traj,
                                     double vel_scale,
                                     double acc_scale,
                                     const rclcpp::Logger& logger,
                                     const char* phase)
{
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(traj, vel_scale, acc_scale)) {
        RCLCPP_ERROR(logger, "  ✗ [%s] TOTG 时间参数化失败", phase);
        return false;
    }

#if DUAL_ARM_HAS_RUCKIG
    std::unordered_map<std::string, double> velocity_limits;
    std::unordered_map<std::string, double> acceleration_limits;
    std::unordered_map<std::string, double> jerk_limits;
    std::size_t fallback_jerk_count = 0;

    // Use the same MoveIt velocity/acceleration bounds as the scaling-factor
    // overload, while supplying jerk explicitly for every active variable.
    // This avoids silently falling back to Ruckig's hard-coded 1000 value.
    if (const auto* group = traj.getGroup(); group != nullptr) {
        for (const auto* joint : group->getActiveJointModels()) {
            for (const auto& variable : joint->getVariableNames()) {
                const auto& bounds = joint->getVariableBounds(variable);
                const double velocity = bounds.velocity_bounded_ && bounds.max_velocity_ > 0.0
                                            ? bounds.max_velocity_ * vel_scale
                                            : 1.0;
                const double acceleration = bounds.acceleration_bounded_ && bounds.max_acceleration_ > 0.0
                                                ? bounds.max_acceleration_ * acc_scale
                                                : 1.0;
                const double jerk = bounds.jerk_bounded_ && bounds.max_jerk_ > 0.0
                                        ? bounds.max_jerk_
                                        : kFallbackJerkLimitRadPerSec3;
                velocity_limits.emplace(variable, velocity);
                acceleration_limits.emplace(variable, acceleration);
                jerk_limits.emplace(variable, jerk);
                if (!(bounds.jerk_bounded_ && bounds.max_jerk_ > 0.0)) {
                    ++fallback_jerk_count;
                }
            }
        }
    }

    const bool smoothing_ok = !velocity_limits.empty() &&
        trajectory_processing::RuckigSmoothing::applySmoothing(
            traj, velocity_limits, acceleration_limits, jerk_limits);
    if (smoothing_ok) {
        RCLCPP_INFO(logger, "  ✓ [%s] 已启用 Ruckig jerk 限制平滑", phase);
        if (fallback_jerk_count > 0) {
            RCLCPP_WARN(logger,
                        "  ! [%s] %zu 个变量未配置 jerk limit，使用显式回退 %.1f rad/s^3",
                        phase, fallback_jerk_count, kFallbackJerkLimitRadPerSec3);
        }
    } else {
        RCLCPP_WARN(logger, "  ! [%s] Ruckig 平滑失败，沿用 TOTG 轨迹", phase);
    }
#else
    RCLCPP_INFO(logger, "  ! [%s] 当前环境无 Ruckig，沿用 TOTG 轨迹", phase);
#endif

    return true;
}

double wrappedAbsJointDelta(double a, double b)
{
    double diff = std::abs(a - b);
    if (diff > M_PI) {
        diff = 2.0 * M_PI - diff;
    }
    return diff;
}

bool validateManualTrajectory(const robot_trajectory::RobotTrajectory& traj,
                              const moveit::core::RobotModelConstPtr& robot_model,
                              const std::string& group_name,
                              const rclcpp::Logger& logger,
                              const char* phase,
                              bool enforce_hand_distance = true)
{
    const auto* jmg = robot_model ? robot_model->getJointModelGroup(group_name) : nullptr;
    if (jmg == nullptr) {
        RCLCPP_ERROR(logger, "  ✗ [%s] 手工轨迹校验失败: 找不到规划组 %s",
                     phase, group_name.c_str());
        return false;
    }

    if (traj.getWayPointCount() < 2) {
        RCLCPP_ERROR(logger, "  ✗ [%s] 手工轨迹校验失败: 航点数量不足 (%zu)",
                     phase, traj.getWayPointCount());
        return false;
    }

    if (!robot_model->hasLinkModel("mj_left_hand") ||
        !robot_model->hasLinkModel("mj_right_hand")) {
        RCLCPP_ERROR(logger, "  ✗ [%s] 手工轨迹校验失败: 找不到左右夹爪 link", phase);
        return false;
    }

    planning_scene::PlanningScene validation_scene(robot_model);
    collision_detection::AllowedCollisionMatrix acm =
        validation_scene.getAllowedCollisionMatrix();
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group_name;
    collision_request.contacts = false;

    std::vector<double> previous_positions;
    double reference_hand_distance = 0.0;

    for (std::size_t i = 0; i < traj.getWayPointCount(); ++i) {
        const auto& state = traj.getWayPoint(i);

        if (!state.satisfiesBounds(jmg, 1e-4)) {
            RCLCPP_ERROR(logger, "  ✗ [%s] 手工轨迹校验失败: 航点 %zu 超出关节限位",
                         phase, i);
            return false;
        }

        collision_detection::CollisionResult collision_result;
        validation_scene.checkSelfCollision(collision_request, collision_result, state, acm);
        if (collision_result.collision) {
            RCLCPP_ERROR(logger, "  ✗ [%s] 手工轨迹校验失败: 航点 %zu 存在自碰撞",
                         phase, i);
            return false;
        }

        std::vector<double> positions;
        state.copyJointGroupPositions(jmg, positions);
        for (const double position : positions) {
            if (!std::isfinite(position)) {
                RCLCPP_ERROR(logger, "  ✗ [%s] 手工轨迹校验失败: 航点 %zu 含非有限关节值",
                             phase, i);
                return false;
            }
        }

        if (!previous_positions.empty()) {
            double max_step = 0.0;
            const std::size_t count = std::min(previous_positions.size(), positions.size());
            for (std::size_t j = 0; j < count; ++j) {
                max_step = std::max(max_step, wrappedAbsJointDelta(positions[j], previous_positions[j]));
            }
            if (max_step > kMaxManualTrajectoryJointStepRad) {
                RCLCPP_ERROR(logger,
                             "  ✗ [%s] 手工轨迹校验失败: 航点 %zu 关节跳变 %.1f° 超过阈值 %.1f°",
                             phase, i, max_step * 180.0 / M_PI,
                             kMaxManualTrajectoryJointStepRad * 180.0 / M_PI);
                return false;
            }
        }
        previous_positions = positions;

        if (enforce_hand_distance) {
            const auto& left_tf = state.getGlobalLinkTransform("mj_left_hand");
            const auto& right_tf = state.getGlobalLinkTransform("mj_right_hand");
            const double hand_distance = (left_tf.translation() - right_tf.translation()).norm();
            if (!std::isfinite(hand_distance)) {
                RCLCPP_ERROR(logger, "  ✗ [%s] 手工轨迹校验失败: 航点 %zu 夹爪距离非有限",
                             phase, i);
                return false;
            }
            if (i == 0) {
                reference_hand_distance = hand_distance;
            } else {
                const double drift = std::abs(hand_distance - reference_hand_distance);
                if (drift > kMaxManualTrajectoryHandDistanceDrift) {
                    RCLCPP_ERROR(logger,
                                 "  ✗ [%s] 手工轨迹校验失败: 航点 %zu 左右夹爪距离漂移 %.1f mm 超过阈值 %.1f mm",
                                 phase, i, drift * 1000.0,
                                 kMaxManualTrajectoryHandDistanceDrift * 1000.0);
                    return false;
                }
            }
        }
    }

    if (enforce_hand_distance) {
        RCLCPP_INFO(logger,
                    "  ✓ [%s] 手工轨迹校验通过: %zu航点，关节限位/自碰撞/跳变/夹爪距离已检查",
                    phase, traj.getWayPointCount());
    } else {
        RCLCPP_INFO(logger,
                    "  ✓ [%s] 手工轨迹校验通过: %zu航点，关节限位/自碰撞/跳变已检查",
                    phase, traj.getWayPointCount());
    }
    return true;
}

struct CoordinatedIkResult
{
    bool success {false};
    bool left_success {false};
    bool right_success {false};
};

CoordinatedIkResult solveCoordinatedIk(const moveit::core::RobotState& seed_state,
                                       moveit::core::RobotState& solved_state,
                                       const moveit::core::JointModelGroup* left_jmg,
                                       const moveit::core::JointModelGroup* right_jmg,
                                       const geometry_msgs::msg::Pose& left_target,
                                       const geometry_msgs::msg::Pose& right_target,
                                       double timeout)
{
    CoordinatedIkResult result;
    if (left_jmg == nullptr || right_jmg == nullptr) {
        return result;
    }

    auto attempt = [&](double attempt_timeout) {
        moveit::core::RobotState candidate(seed_state);
        result.left_success = candidate.setFromIK(left_jmg, left_target, attempt_timeout);
        result.right_success = result.left_success &&
            candidate.setFromIK(right_jmg, right_target, attempt_timeout);
        if (!result.left_success || !result.right_success) {
            return false;
        }
        candidate.update();
        solved_state = candidate;
        result.success = true;
        return true;
    };

    if (attempt(timeout)) {
        return result;
    }

    // Retry from the exact same seed. Never let a failed left/right partial
    // solve leak into the next attempt or into the caller's trajectory.
    if (timeout < 0.5) {
        attempt(0.5);
    }
    return result;
}
}

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
            
            ctx_->dual_arm_->setGoalPositionTolerance(0.01);
            ctx_->dual_arm_->setGoalOrientationTolerance(0.01);
            
            
            
            // 同时创建单臂接口用于获取运动学信息
            ctx_->left_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                ctx_->shared_from_this(), ctx_->left_arm_group_);
            ctx_->right_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                ctx_->shared_from_this(), ctx_->right_arm_group_);
            
            // 切换为 Pilz 工业运动规划器
            ctx_->dual_arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
            ctx_->left_arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
            ctx_->right_arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
            RCLCPP_INFO(ctx_->get_logger(), "✓ 规划管道已全局切换至 Pilz Industrial Motion Planner");
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
        
        // 查找物体在 ACM 中的索引；若不存在则补建一列/一行
        auto obj_it = std::find(acm.entry_names.begin(), acm.entry_names.end(), object_id);
        size_t obj_idx = 0;
        if (obj_it == acm.entry_names.end()) {
            RCLCPP_WARN(ctx_->get_logger(), "  ! 物体 %s 不在 ACM 中，正在自动补建条目", object_id.c_str());
            acm.entry_names.push_back(object_id);
            for (auto& entry : acm.entry_values) {
                entry.enabled.push_back(false);
            }
            moveit_msgs::msg::AllowedCollisionEntry new_entry;
            new_entry.enabled.resize(acm.entry_names.size(), false);
            acm.entry_values.push_back(new_entry);
            obj_idx = acm.entry_names.size() - 1;
        } else {
            obj_idx = std::distance(acm.entry_names.begin(), obj_it);
        }
        
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
        
        // 定义铝棒位置 (与MuJoCo中objects.xml一致，或由视觉动态检测)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        geometry_msgs::msg::Pose detected_pose;
        if (ctx_->getDetectedRodPose(detected_pose)) {
            box_pose.position.x = detected_pose.position.x;
            box_pose.position.y = detected_pose.position.y;
        } else {
            box_pose.position.x = 0.50;  // 与MuJoCo中的位置一致
            box_pose.position.y = 0.0;
        }
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

void DualArmController::runGripperSelftest()
    {
        RCLCPP_INFO(ctx_->get_logger(),
                    "\n----- [P3A-2A] 夹爪空载开合测试: 打开 -> 空载闭合 -> 打开"
                    "（机械臂不动，不发布 weld、不 attach）-----");
        const double open_pos = ctx_->gripper_open_position_;
        const double close_pos = ctx_->gripper_close_pos_;
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2A] 命令单位为**单指位移**：打开 %.4f（服务器目标开口 %.4f m，"
                    "即最大开口）/ 闭合 %.4f（目标开口 %.4f m）",
                    open_pos, 2.0 * open_pos, close_pos, 2.0 * close_pos);

        // The finger joints are prismatic with limits [0.0, 0.04] and their
        // position IS the per-finger displacement, so they can be compared
        // directly with command.position. The server's reported `position` is a
        // separate internal width model and is recorded alongside, not instead.
        auto logFingerJoints = [&](const char* label) {
            const moveit::core::RobotStatePtr st = ctx_->dual_arm_->getCurrentState();
            std::string vars;
            for (const std::string& v : st->getVariableNames()) {
                vars += " " + v;
            }
            const bool has_fingers = vars.find("finger_joint1") != std::string::npos;
            if (!has_fingers) {
                RCLCPP_WARN(ctx_->get_logger(),
                            "  [P3A-2A] %s: 机器人模型中不含 finger_joint，无法读取实测指关节",
                            label);
                return;
            }
            st->update();
            const double l = st->getVariablePosition("mj_left_finger_joint1");
            const double r = st->getVariablePosition("mj_right_finger_joint1");
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P3A-2A] %s 实测指关节: 左=%.4f m 右=%.4f m（单指位移，"
                        "关节范围 [0.0000, 0.0400]）⇒ 实测总开口 左=%.4f 右=%.4f m",
                        label, l, r, 2.0 * l, 2.0 * r);
        };

        const char* step = "open";
        bool ok = ctx_->grippers_->controlDualGrippers(open_pos);
        logFingerJoints("open");
        if (ok) {
            step = "empty_close";
            ok = ctx_->grippers_->controlDualGrippers(close_pos);
            logFingerJoints("empty_close");
        }
        if (ok) {
            step = "reopen";
            ok = ctx_->grippers_->controlDualGrippers(open_pos);
            logFingerJoints("reopen");
        }

        RCLCPP_INFO(ctx_->get_logger(),
                    "[GRIPPER_RESULT] state=%s failed_step=%s open_cmd=%.4f close_cmd=%.4f "
                    "arm_motion=none weld=0 attach=0",
                    ok ? "GRIPPER_TEST_DONE" : "FAILED", ok ? "-" : step, open_pos, close_pos);
        if (ok) {
            RCLCPP_INFO(ctx_->get_logger(),
                        "  ✓ [P3A-2A] 空载开合完成。**空载成功不是抓取成功**；"
                        "接触导致的停止语义留待下一阶段结合物体验证");
            ctx_->setCurrentState(TaskState::GRIPPER_TEST_DONE);
        } else {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2A] 空载开合在步骤 '%s' 失败，停止本轮", step);
            ctx_->setCurrentState(TaskState::ERROR);
        }
    }

void DualArmController::executeInit()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: INIT] 初始化...");

        try {
            // A1 P3a-2a: minimal explicit empty-load entry. It short-circuits
            // INIT, so no arm trajectory, weld or attach can be reached.
            if (ctx_->gripper_selftest_) {
                runGripperSelftest();
                return;
            }

            // Mode B: wait for vision detection before proceeding
            if (ctx_->use_vision_) {
                RCLCPP_INFO(ctx_->get_logger(), "  [视觉模式] 等待物体检测...");
                auto start = ctx_->now();
                geometry_msgs::msg::Pose detected_pose;
                while (rclcpp::ok() && !ctx_->getDetectedRodPose(detected_pose)) {
                    if ((ctx_->now() - start).seconds() > 10.0) {
                        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 视觉超时: 10秒内未检测到物体");
                        ctx_->current_state_ = TaskState::ERROR;
                        return;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                double yaw = extractYawFromQuaternion(detected_pose.orientation);
                RCLCPP_INFO(ctx_->get_logger(),
                    "  ✓ 物体已检测: pos=(%.3f, %.3f, %.3f) yaw=%.1f°",
                    detected_pose.position.x,
                    detected_pose.position.y,
                    detected_pose.position.z,
                    yaw * 180.0 / M_PI);
            }

            // A1 P1b: prepare-only path. Frozen truth -> planning scene sync.
            // This returns before any of the static-path scene construction
            // below, and before APPROACH, so no motion can be reached.
            if (ctx_->usesA1Truth()) {
                if (!prepareFromA1Truth()) {
                    return;  // reason already logged; state set to ERROR
                }
                return;  // PREPARED, no motion
            }

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

/**
 * @brief A1 P1b: freeze the truth reference and synchronise the planning scene.
 *
 * Reached from INIT only when rod_pose_source == a1_truth. It never moves the
 * robot: on success the FSM goes to PREPARED and the caller returns immediately,
 * so APPROACH/GRASP are not reachable from here.
 */
bool DualArmController::prepareFromA1Truth()
    {
        using namespace std::chrono_literals;

        // 1. This round A1 is preparation only. Refuse at the entry of the call
        //    chain rather than truncating after motion has started.
        if (!ctx_->prepare_only_) {
            RCLCPP_ERROR(ctx_->get_logger(),
                         "  ✗ [A1] rod_pose_source=a1_truth 需要 prepare_only=true；本轮不开启 A1 运动");
            ctx_->setCurrentState(TaskState::ERROR);
            return false;
        }

        // 2. Coordinate frames. The planning frame is read from MoveIt; the
        //    mapping onto the truth frame is decided by documented source
        //    evidence, not by assuming a TF chain exists.
        const std::string planning_frame = ctx_->dual_arm_->getPlanningFrame();
        RCLCPP_INFO(ctx_->get_logger(), "  [A1] MoveIt 规划坐标系: %s", planning_frame.c_str());

        // 3. Bounded wait (steady clock) for the truth reference to freeze.
        const double timeout_sec = ctx_->a1_truth_timeout_sec_;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeout_sec));
        while (rclcpp::ok() && !ctx_->a1_truth_latch_.frozen()) {
            if (std::chrono::steady_clock::now() >= deadline) {
                RCLCPP_ERROR(ctx_->get_logger(),
                             "  ✗ [A1] E_TRUTH_TIMEOUT: %.1f 秒内未收到有效真值 (topic=%s, "
                             "expected_run_id=%s)",
                             timeout_sec, ctx_->a1_truth_topic_.c_str(),
                             ctx_->expected_run_id_.empty() ? "<UNSET>"
                                                            : ctx_->expected_run_id_.c_str());
                RCLCPP_ERROR(ctx_->get_logger(),
                             "    不回退到硬编码 A0 位姿，也不复用视觉检测缓存；停止准备");
                if (!ctx_->a1_truth_latch_.last_rejection_code().empty()) {
                    RCLCPP_ERROR(ctx_->get_logger(), "    最近一次被拒原因: %s",
                                 ctx_->a1_truth_latch_.last_rejection_code().c_str());
                }
                ctx_->setCurrentState(TaskState::ERROR);
                return false;
            }
            std::this_thread::sleep_for(50ms);
        }
        if (!rclcpp::ok()) {
            ctx_->setCurrentState(TaskState::ERROR);
            return false;
        }

        a1t::TruthPayload truth;
        if (!ctx_->getFrozenRodTruth(truth)) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [A1] 真值 latch 状态异常");
            ctx_->setCurrentState(TaskState::ERROR);
            return false;
        }

        RCLCPP_INFO(ctx_->get_logger(),
                    "  [A1] 冻结真值: run_id=%s frame=%s pos=(%.6f, %.6f, %.6f) size=(%.3f, %.3f, %.3f) mass=%.6f kg",
                    truth.run_id.c_str(), truth.frame_id.c_str(),
                    truth.position.x, truth.position.y, truth.position.z,
                    truth.size.x, truth.size.y, truth.size.z, truth.mass_kg);
        const double truth_yaw = extractYawFromQuaternion(
            tf2::toMsg(tf2::Quaternion(truth.orientation.x, truth.orientation.y,
                                       truth.orientation.z, truth.orientation.w)));
        RCLCPP_INFO(ctx_->get_logger(), "  [A1] 冻结真值 yaw=%.4f° (长轴为杆局部 Y 轴)",
                    truth_yaw * 180.0 / M_PI);

        const a1t::FrameMapping mapping =
            a1t::resolveFrameMapping(truth.frame_id, planning_frame);
        if (mapping == a1t::FrameMapping::UNSUPPORTED) {
            RCLCPP_ERROR(ctx_->get_logger(),
                         "  ✗ [A1] E_TRUTH_FRAME_UNSUPPORTED: 真值 frame '%s' 与规划坐标系 '%s' "
                         "之间没有源码依据的变换",
                         truth.frame_id.c_str(), planning_frame.c_str());
            ctx_->setCurrentState(TaskState::ERROR);
            return false;
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [A1] 坐标处理: %s",
                    mapping == a1t::FrameMapping::SAME_FRAME
                        ? "同名坐标系，位姿直通"
                        : "world 与 base_link 数值等价（SRDF 固定 virtual_joint + URDF/MuJoCo "
                          "基座位置一致），按显式单位变换处理");

        // 4. Synchronise the world object from the frozen truth.
        geometry_msgs::msg::Pose pose;
        pose.position.x = truth.position.x;
        pose.position.y = truth.position.y;
        pose.position.z = truth.position.z;
        pose.orientation.x = truth.orientation.x;
        pose.orientation.y = truth.orientation.y;
        pose.orientation.z = truth.orientation.z;
        pose.orientation.w = truth.orientation.w;
        geometry_msgs::msg::Vector3 size;
        size.x = truth.size.x;
        size.y = truth.size.y;
        size.z = truth.size.z;

        std::string reason;
        if (!syncAluminumRodFromTruth(pose, size, planning_frame, reason)) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [A1] 规划场景同步失败: %s", reason.c_str());
            ctx_->setCurrentState(TaskState::ERROR);
            return false;
        }

        // 5. A1 P2a/P2b: optional static and joint pre-grasp path checks on the
        //    frozen truth. Both run inside the same no-motion boundary.
        //    run inside the same no-motion boundary. A failure is reported
        //    loudly and is NOT turned into a prepare failure -- the truth really
        //    is frozen and the scene really is synced -- but the run record
        //    makes the rejection unmistakable and the sample is never accepted.
        if (ctx_->p2a_check_) {
            a1v::SampleValidation validation;
            const bool static_ok = runP2aChecks(truth, validation);
            if (!static_ok) {
                RCLCPP_ERROR(ctx_->get_logger(),
                             "  ✗ [P2A] 该样本未通过静态有效性检查（几何=%s 说明=%s / IK=%s 说明=%s / "
                             "碰撞=%s 说明=%s）",
                             a1v::statusName(validation.geometry.status),
                             validation.geometry.reason_code.c_str(),
                             a1v::statusName(validation.ik.status),
                             validation.ik.reason_code.c_str(),
                             a1v::statusName(validation.collision.status),
                             validation.collision.reason_code.c_str());
                RCLCPP_ERROR(ctx_->get_logger(),
                             "  ✗ [P2A] 不放宽碰撞规则或容差；该样本按拒绝处理，"
                             "不得作为可搬运样本使用");
            } else {
                RCLCPP_INFO(ctx_->get_logger(),
                            "  ✓ [P2A] 静态检查通过（几何/IK/碰撞）");
                // P2b runs only on a sample whose static checks passed.
                a1v::SampleValidation full = validation;
                if (ctx_->p2b_check_) {
                    const bool path_ok = runP2bPathCheck(full);
                    const char* p2_accept = (static_ok && path_ok) ? "true" : "false";
                    RCLCPP_INFO(ctx_->get_logger(),
                                "[P2B_RESULT] geometry=%s ik=%s collision=%s path_validation=%s "
                                "p2_acceptance=%s",
                                a1v::statusName(full.geometry.status),
                                a1v::statusName(full.ik.status),
                                a1v::statusName(full.collision.status),
                                a1v::statusName(full.path_validation.status), p2_accept);
                    if (path_ok && ctx_->pregrasp_execute_) {
                        // P3a-1: only a real p2_acceptance=true reaches here, and
                        // the SAME plan object P2b just verified is executed.
                        RCLCPP_INFO(ctx_->get_logger(),
                                    "  [P2] p2_acceptance=true ⇒ 允许执行本轮校验过的同一 Plan");
                        std::string exec_detail;
                        const bool arrived = executePregrasp(truth, exec_detail);
                        RCLCPP_INFO(ctx_->get_logger(),
                                    "[P3A_RESULT] state=%s detail=%s",
                                    arrived ? "PREGRASP_REACHED" : "FAILED", exec_detail.c_str());
                        if (arrived) {
                            ctx_->setCurrentState(TaskState::PREGRASP_REACHED);
                        } else {
                            ctx_->setCurrentState(TaskState::ERROR);
                        }
                        return arrived;
                    }
                    if (path_ok) {
                        RCLCPP_INFO(ctx_->get_logger(),
                                    "  [P2] p2_acceptance=true 的语义 = 通过本次静态与预抓取路径检查；"
                                    "**不是**搬运 DONE。pregrasp_execute 未启用，本轮不下发运动");
                        RCLCPP_INFO(ctx_->get_logger(),
                                    "[P3A_RESULT] state=NOT_RUN detail=pregrasp_execute is disabled");
                    } else {
                        RCLCPP_ERROR(ctx_->get_logger(),
                                     "  ✗ [P2B] 联合预抓取路径未通过：%s（%s）",
                                     full.path_validation.reason_code.c_str(),
                                     full.path_validation.detail.c_str());
                        RCLCPP_ERROR(ctx_->get_logger(),
                                     "  ✗ [P2] 不放宽碰撞规则或换起点；本样本按拒绝处理");
                    }
                } else {
                    RCLCPP_INFO(ctx_->get_logger(),
                                "[P2B_RESULT] geometry=%s ik=%s collision=%s "
                                "path_validation=NOT_RUN p2_acceptance=null",
                                a1v::statusName(full.geometry.status),
                                a1v::statusName(full.ik.status),
                                a1v::statusName(full.collision.status));
                    RCLCPP_INFO(ctx_->get_logger(),
                                "  [P2B] 未启用 p2b_check，跳过联合预抓取路径检查");
                }
            }
        } else {
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P2A] 未启用 p2a_check，跳过静态有效性检查（几何/IK/碰撞均为 NOT_RUN）");
        }

        RCLCPP_INFO(ctx_->get_logger(),
                    "✓ [A1] 准备完成(PREPARED): 真值已冻结且规划场景已同步确认，未执行任何搬运运动\n");
        ctx_->setCurrentState(TaskState::PREPARED);
        return true;
    }

/**
 * @brief Writes the rod world object from the frozen truth and verifies it.
 *
 * Uses the synchronous, bool-returning apply so a failure cannot be reported as
 * success, then reads the object back and compares position, orientation
 * (q/-q equivalent) and size. The ACM/SRDF are deliberately not touched.
 */
bool DualArmController::syncAluminumRodFromTruth(const geometry_msgs::msg::Pose& pose,
                                                const geometry_msgs::msg::Vector3& size,
                                                const std::string& planning_frame,
                                                std::string& reason)
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = planning_frame;
        collision_object.id = "aluminum_rod";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {size.x, size.y, size.z};

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        moveit::planning_interface::PlanningSceneInterface psi;
        if (!psi.applyCollisionObject(collision_object)) {
            reason = "E_SCENE_APPLY_FAILED: /apply_planning_scene 返回失败";
            return false;
        }

        // Read back and verify. A topic-only publish would not prove anything.
        const auto objects = psi.getObjects(std::vector<std::string>{"aluminum_rod"});
        const auto it = objects.find("aluminum_rod");
        if (it == objects.end()) {
            reason = "E_SCENE_READBACK_MISSING: 读回时未找到 aluminum_rod";
            return false;
        }
        if (it->second.primitive_poses.empty() || it->second.primitives.empty()) {
            reason = "E_SCENE_READBACK_MISSING: aluminum_rod 缺少 primitive 或 pose";
            return false;
        }
        if (it->second.primitives[0].dimensions.size() < 3) {
            reason = "E_SCENE_READBACK_MISSING: aluminum_rod 尺寸字段不完整";
            return false;
        }

        // Compose the object frame pose with the primitive pose. MoveIt may
        // express the placement in either: it returns the primitive pose
        // relative to the object pose, so reading only `primitive_poses[0]`
        // yields identity when the placement was stored on the object. The
        // composition below is correct for both conventions.
        tf2::Transform object_tf;
        tf2::Transform primitive_tf;
        tf2::fromMsg(it->second.pose, object_tf);
        tf2::fromMsg(it->second.primitive_poses[0], primitive_tf);
        geometry_msgs::msg::Pose actual_pose;
        tf2::toMsg(object_tf * primitive_tf, actual_pose);
        const auto& actual_dims = it->second.primitives[0].dimensions;

        // Raw read-back, logged so a mismatch can be diagnosed from the record
        // rather than guessed at.
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [A1] 读回原始值: object_frame='%s' primitives=%zu pose=(%.6f, %.6f, %.6f) "
                    "quat=(%.6f, %.6f, %.6f, %.6f) dims=(%.4f, %.4f, %.4f)",
                    it->second.header.frame_id.c_str(), it->second.primitives.size(),
                    actual_pose.position.x, actual_pose.position.y, actual_pose.position.z,
                    actual_pose.orientation.x, actual_pose.orientation.y,
                    actual_pose.orientation.z, actual_pose.orientation.w,
                    actual_dims[0], actual_dims[1], actual_dims[2]);
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [A1] 写入请求值: object_frame='%s' pose=(%.6f, %.6f, %.6f) "
                    "quat=(%.6f, %.6f, %.6f, %.6f)",
                    collision_object.header.frame_id.c_str(), pose.position.x, pose.position.y,
                    pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
                    pose.orientation.w);

        a1t::TruthPayload expected;
        expected.position = {pose.position.x, pose.position.y, pose.position.z};
        expected.orientation = {pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                pose.orientation.w};
        expected.size = {size.x, size.y, size.z};

        const a1t::Vec3 actual_position = {actual_pose.position.x, actual_pose.position.y,
                                           actual_pose.position.z};
        const a1t::Quat actual_orientation = {actual_pose.orientation.x, actual_pose.orientation.y,
                                              actual_pose.orientation.z, actual_pose.orientation.w};
        const a1t::Vec3 actual_size = {actual_dims[0], actual_dims[1], actual_dims[2]};

        const double pos_tol_m = 0.001;                        // 1 mm
        const double angle_tol_rad = 0.5 * M_PI / 180.0;       // 0.5 deg
        const double size_tol_m = 1e-6;
        const a1t::SceneCheck check = a1t::verifySceneReadback(
            expected, actual_position, actual_orientation, actual_size, pos_tol_m, angle_tol_rad,
            size_tol_m);

        RCLCPP_INFO(ctx_->get_logger(),
                    "  [A1] 场景读回核对: Δpos=%.4f mm, Δangle=%.4f°, Δsize=%.3e m (容差 %.1f mm / %.1f° / %.0e m)",
                    check.pos_err_m * 1000.0, check.angle_err_rad * 180.0 / M_PI,
                    check.size_err_m, pos_tol_m * 1000.0, angle_tol_rad * 180.0 / M_PI, size_tol_m);

        if (!check.ok) {
            reason = check.reason_code + ": " + check.detail;
            return false;
        }
        reason = "ok";
        return true;
    }

// ===========================================================================
// A1 P2a: static sample validity (geometry, combined dual-arm IK, collision).
// Everything here runs inside the PREPARED boundary: no arm, gripper, weld or
// attach command is ever sent.
// ===========================================================================
namespace {

Eigen::Isometry3d toEigenPose(const a1v::ArmTarget& t)
{
    Eigen::Isometry3d m = Eigen::Isometry3d::Identity();
    m.translation() = Eigen::Vector3d(t.position.x, t.position.y, t.position.z);
    m.linear() = Eigen::Quaterniond(t.orientation.w, t.orientation.x, t.orientation.y,
                                    t.orientation.z)
                     .normalized()
                     .toRotationMatrix();
    return m;
}

a1v::ArmTarget fromEigenPose(const Eigen::Isometry3d& m)
{
    const Eigen::Quaterniond q(m.rotation());
    a1v::ArmTarget t;
    t.position = {m.translation().x(), m.translation().y(), m.translation().z()};
    t.orientation = {q.x(), q.y(), q.z(), q.w()};
    return t;
}

// Contact tally shared by the P2a static check and the P2b path check, so both
// classify contacts identically.
struct ContactTally {
    std::size_t rod {0};
    std::size_t support {0};
    std::size_t dual_arm {0};
    std::size_t self {0};
    std::size_t world {0};
    std::vector<std::string> examples;
    std::size_t total() const { return rod + support + dual_arm + self + world; }
};

ContactTally tallyContactsIn(const planning_scene::PlanningSceneConstPtr& sc,
                             const moveit::core::RobotModelConstPtr& mdl,
                             const moveit::core::RobotState& st,
                             const collision_detection::AllowedCollisionMatrix& acm,
                             const std::string& rod_id,
                             const std::string& support_id)
{
    ContactTally tally;
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 200;
    collision_detection::CollisionResult res;
    sc->checkCollision(req, res, st, acm);
    for (const auto& contact : res.contacts) {
        const std::string& a = contact.first.first;
        const std::string& b = contact.first.second;
        if (a1v::isRodPair(a, b, rod_id)) {
            ++tally.rod;
        } else if (a1v::isSupportPair(a, b, support_id)) {
            ++tally.support;
        } else if (a1v::isCrossArmPair(a, b)) {
            ++tally.dual_arm;
        } else if (mdl->hasLinkModel(a) && mdl->hasLinkModel(b)) {
            ++tally.self;
        } else {
            ++tally.world;
        }
        if (tally.examples.size() < 10) {
            tally.examples.push_back(a + "<->" + b);
        }
    }
    return tally;
}

std::string describeTally(const ContactTally& t)
{
    std::string s = "杆接触=" + std::to_string(t.rod) + " 地面/支撑面=" + std::to_string(t.support) +
                    " 双臂互碰=" + std::to_string(t.dual_arm) + " 自碰撞=" + std::to_string(t.self) +
                    " 其他世界物体=" + std::to_string(t.world);
    if (!t.examples.empty()) {
        s += " 例:";
        for (const std::string& e : t.examples) {
            s += " " + e;
        }
    }
    return s;
}

// Collects the arm joints of one group in a stable order for limit / jump
// checks.
void collectArmJoints(const moveit::core::RobotState& st,
                      const moveit::core::JointModelGroup* jmg,
                      std::vector<double>& q,
                      std::vector<double>& lower,
                      std::vector<double>& upper,
                      std::vector<std::string>& names)
{
    for (const moveit::core::JointModel* jm : jmg->getActiveJointModels()) {
        for (const std::string& var : jm->getVariableNames()) {
            const moveit::core::VariableBounds& b = jm->getVariableBounds(var);
            q.push_back(st.getVariablePosition(var));
            lower.push_back(b.position_bounded_ ? b.min_position_
                                                : -std::numeric_limits<double>::infinity());
            upper.push_back(b.position_bounded_ ? b.max_position_
                                                : std::numeric_limits<double>::infinity());
            names.push_back(var);
        }
    }
}

}  // namespace

/**
 * A1 P2a: build an INDEPENDENT robot model in which only the two hand links have
 * their collision geometry replaced by the conservative AABB of the MuJoCo
 * hand_c mesh.
 *
 * The URDF and SRDF are re-parsed from the node's own parameters, so the result
 * is a private object: the shared RobotModel and the live PlanningScene are
 * never touched. If anything cannot be loaded the caller is told, so an empty
 * model can never silently stand in for a real one.
 */
std::shared_ptr<moveit::core::RobotModel> DualArmController::buildMujocoHandModel(
    const a1v::Aabb& hand_aabb,
    const std::vector<std::string>& hand_links,
    std::string& reason)
{
    if (!hand_aabb.valid) {
        reason = "E_HAND_AABB_INVALID: the hand mesh bounding box was not computed";
        return nullptr;
    }
    if (!ctx_->has_parameter("robot_description") ||
        !ctx_->has_parameter("robot_description_semantic")) {
        reason = "E_MODEL_PARAMS_MISSING: robot_description / robot_description_semantic "
                 "are not available on this node";
        return nullptr;
    }
    const std::string urdf_xml = ctx_->get_parameter("robot_description").as_string();
    const std::string srdf_xml = ctx_->get_parameter("robot_description_semantic").as_string();
    if (urdf_xml.empty() || srdf_xml.empty()) {
        reason = "E_MODEL_PARAMS_EMPTY: robot_description or robot_description_semantic is empty";
        return nullptr;
    }

    // Fresh parse: this object is ours alone.
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml);
    if (!urdf_model) {
        reason = "E_URDF_PARSE_FAILED: the robot description could not be parsed";
        return nullptr;
    }

    const a1v::Vec3 center = hand_aabb.center();
    const a1v::Vec3 size = hand_aabb.size();
    if (!(size.x > 0.0) || !(size.y > 0.0) || !(size.z > 0.0)) {
        reason = "E_HAND_AABB_DEGENERATE: the hand mesh bounding box has a non-positive extent";
        return nullptr;
    }

    for (const std::string& name : hand_links) {
        // The parsed model is private, so adjusting its links is safe here.
        urdf::LinkSharedPtr link = std::const_pointer_cast<urdf::Link>(urdf_model->getLink(name));
        if (!link) {
            reason = "E_HAND_LINK_MISSING: '" + name + "' is not in the robot description";
            return nullptr;
        }
        auto box = std::make_shared<urdf::Box>();
        box->dim = urdf::Vector3(size.x, size.y, size.z);

        auto collision = std::make_shared<urdf::Collision>();
        collision->origin.clear();  // identity rotation, zero position ...
        // ... then offset to the AABB centre, which is NOT the hand origin.
        collision->origin.position = urdf::Vector3(center.x, center.y, center.z);
        collision->geometry = box;

        link->collision_array.clear();
        link->collision_array.push_back(collision);
        link->collision = collision;
    }

    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, srdf_xml)) {
        reason = "E_SRDF_PARSE_FAILED: the semantic description could not be parsed";
        return nullptr;
    }

    try {
        auto model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
        return model;
    } catch (const std::exception& e) {
        reason = std::string("E_MODEL_BUILD_FAILED: ") + e.what();
        return nullptr;
    }
}

bool DualArmController::runP2aChecks(const a1t::TruthPayload& truth, a1v::SampleValidation& out)
{
    using namespace std::chrono_literals;

    // P2a judges a static target only. Saying so explicitly keeps a static pass
    // from being read as "transportable".
    out.path_validation.status = a1v::CheckStatus::NOT_RUN;
    out.path_validation.reason_code = "E_P2B_NOT_RUN";
    out.path_validation.detail =
        "joint pre-grasp path validity belongs to P2b and is not evaluated here";

    // Every exit path emits the machine-readable line, so a rejected sample is
    // recorded as precisely as an accepted one. Absence of the line is never a
    // pass: the run script treats a missing line as "did not pass".
    auto emit = [&](bool ok) {
        RCLCPP_INFO(ctx_->get_logger(),
                    "[P2A_RESULT] geometry=%s ik=%s collision=%s path_validation=NOT_RUN "
                    "p2_acceptance=null static_passed=%s",
                    a1v::statusName(out.geometry.status), a1v::statusName(out.ik.status),
                    a1v::statusName(out.collision.status), ok ? "true" : "false");
        return ok;
    };

    RCLCPP_INFO(ctx_->get_logger(), "\n----- [P2A] 样本静态有效性检查（不运动）-----");

    // -----------------------------------------------------------------------
    // 1. Geometry
    // -----------------------------------------------------------------------
    a1v::WorkspaceBounds bounds;
    bounds.x_min = ctx_->p2a_workspace_x_min_;
    bounds.x_max = ctx_->p2a_workspace_x_max_;
    bounds.y_min = ctx_->p2a_workspace_y_min_;
    bounds.y_max = ctx_->p2a_workspace_y_max_;
    bounds.source = ctx_->p2a_workspace_source_;

    a1v::SupportCriteria support;
    support.support_z = ctx_->p2a_support_z_;
    support.resting_z_tol = ctx_->p2a_resting_z_tol_;
    support.max_tilt_deg = ctx_->p2a_max_tilt_deg_;

    const a1v::GeometryResult geo = a1v::checkGeometry(truth, bounds, support);
    out.geometry = geo.item;

    RCLCPP_INFO(ctx_->get_logger(), "  [P2A][几何] %s (%s)", a1v::statusName(out.geometry.status),
                out.geometry.reason_code.c_str());
    RCLCPP_INFO(ctx_->get_logger(),
                "    杆 yaw=%.4f° 倾角=%.4f° 中心 z=%.6f | footprint x=[%.4f, %.4f] y=[%.4f, %.4f]",
                geo.yaw_rad * 180.0 / M_PI, geo.tilt_deg, geo.center_z, geo.footprint.x_min,
                geo.footprint.x_max, geo.footprint.y_min, geo.footprint.y_max);
    RCLCPP_INFO(ctx_->get_logger(),
                "    配置工作区 x=[%.3f, %.3f] y=[%.3f, %.3f]，最小余量 %.3f mm。来源: %s",
                bounds.x_min, bounds.x_max, bounds.y_min, bounds.y_max,
                geo.footprint.min_clearance_m * 1000.0,
                bounds.source.empty() ? "<未注明>" : bounds.source.c_str());
    RCLCPP_INFO(ctx_->get_logger(),
                "    说明: MuJoCo 支撑面为无限 plane（dual_scene.xml 的 floor geom，无 pos），"
                "上界是配置的研究包络，不是物理桌面边缘");
    if (out.geometry.status != a1v::CheckStatus::PASSED) {
        RCLCPP_ERROR(ctx_->get_logger(), "    ✗ %s", out.geometry.detail.c_str());
    }

    // -----------------------------------------------------------------------
    // 2. Shared targets
    // -----------------------------------------------------------------------
    a1v::GraspGeometry gg;
    gg.grasp_offset_m = ctx_->p2a_grasp_offset_m_;
    gg.eef_base_yaw_rad = ctx_->p2a_eef_base_yaw_deg_ * M_PI / 180.0;
    gg.pre_grasp_height_m = ctx_->approach_height_;
    gg.grasp_height_m = ctx_->grasp_height_;

    double rod_yaw = 0.0;
    const a1v::DualTarget pre = a1v::buildPreGraspTarget(truth, gg, &rod_yaw);
    const a1v::DualTarget grasp = a1v::buildGraspTarget(truth, gg, &rod_yaw);

    if (!std::isfinite(rod_yaw)) {
        out.ik.status = a1v::CheckStatus::FAILED;
        out.ik.reason_code = "E_TARGET_NO_YAW";
        out.ik.detail = "rod yaw could not be extracted, so no target can be built";
        out.collision.status = a1v::CheckStatus::NOT_RUN;
        return emit(false);
    }

    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2A][目标] 抓取偏移 %.3f m（沿杆长轴，杆半长 %.3f m），末端基准 yaw %.1f°",
                gg.grasp_offset_m, truth.size.y * 0.5, ctx_->p2a_eef_base_yaw_deg_);
    RCLCPP_INFO(ctx_->get_logger(), "    预抓取 z=%.3f 抓取 z=%.3f（均为腕部 link8 原点高度，非指尖/TCP）",
                gg.pre_grasp_height_m, gg.grasp_height_m);
    RCLCPP_INFO(ctx_->get_logger(), "    左(杆 +长轴端) 预抓取=(%.4f, %.4f, %.4f) 抓取=(%.4f, %.4f, %.4f)",
                pre.left.position.x, pre.left.position.y, pre.left.position.z,
                grasp.left.position.x, grasp.left.position.y, grasp.left.position.z);
    RCLCPP_INFO(ctx_->get_logger(), "    右(杆 -长轴端) 预抓取=(%.4f, %.4f, %.4f) 抓取=(%.4f, %.4f, %.4f)",
                pre.right.position.x, pre.right.position.y, pre.right.position.z,
                grasp.right.position.x, grasp.right.position.y, grasp.right.position.z);

    // Cross-check the ROS-free orientation builder against tf2, the API the A0
    // path uses, so "yaw=0 reproduces A0" is demonstrated and not asserted.
    {
        tf2::Quaternion reference;
        reference.setRPY(M_PI, 0.0, rod_yaw + gg.eef_base_yaw_rad);
        const double diff = a1t::rotationDistanceRad(
            a1t::Quat{pre.left.orientation.x, pre.left.orientation.y, pre.left.orientation.z,
                      pre.left.orientation.w},
            a1t::Quat{reference.x(), reference.y(), reference.z(), reference.w()});
        RCLCPP_INFO(ctx_->get_logger(),
                    "    姿态约定核对: 本构造器 q=(%.9f, %.9f, %.9f, %.9f) 与 tf2 setRPY(π,0,%.4f°) "
                    "角度差 %.3e rad",
                    pre.left.orientation.x, pre.left.orientation.y, pre.left.orientation.z,
                    pre.left.orientation.w, (rod_yaw + gg.eef_base_yaw_rad) * 180.0 / M_PI, diff);
    }

    // -----------------------------------------------------------------------
    // 3. Combined dual-arm IK
    // -----------------------------------------------------------------------
    auto robot_model = ctx_->dual_arm_->getRobotModel();
    const moveit::core::JointModelGroup* left_jmg =
        robot_model->getJointModelGroup(ctx_->left_arm_group_);
    const moveit::core::JointModelGroup* right_jmg =
        robot_model->getJointModelGroup(ctx_->right_arm_group_);
    if (left_jmg == nullptr || right_jmg == nullptr) {
        out.ik.status = a1v::CheckStatus::FAILED;
        out.ik.reason_code = "E_IK_NO_GROUP";
        out.ik.detail = "an arm joint model group is not present in the robot model";
        out.collision.status = a1v::CheckStatus::NOT_RUN;
        return emit(false);
    }

    // Verified at runtime rather than assumed: which link the arm group's
    // targets actually refer to.
    const std::string left_eef = ctx_->left_arm_->getEndEffectorLink();
    const std::string right_eef = ctx_->right_arm_->getEndEffectorLink();
    RCLCPP_INFO(ctx_->get_logger(), "  [P2A][IK] 末端 link: 左=%s 右=%s（目标位姿即该 link 的原点位姿）",
                left_eef.c_str(), right_eef.c_str());

    const moveit::core::RobotStatePtr start = ctx_->dual_arm_->getCurrentState();
    const a1v::DualTarget targets[2] = {pre, grasp};
    const char* eefs[2] = {left_eef.c_str(), right_eef.c_str()};
    const double pos_tol = ctx_->p2a_ik_pos_tol_m_;
    const double ang_tol = ctx_->p2a_ik_angle_tol_deg_ * M_PI / 180.0;

    // Bounded search over several IK attempts.
    //
    // KDL restarts randomly inside its timeout, so "take the first success"
    // makes the outcome depend on luck: repeated runs of the same sample
    // produced descent changes of 1.86 rad and 2.26 rad for the same arm.
    // Every attempt that solves BOTH targets and passes the limit and residual
    // checks is therefore scored, and the pair with the smallest pre-grasp ->
    // grasp change is kept. The check then answers "does a well-behaved
    // combined dual-arm configuration exist for this sample?", which is the
    // question that matters, instead of "did the first random seed land well?".
    moveit::core::RobotState best_pre(*start);
    moveit::core::RobotState best_grasp(*start);
    bool have_candidate = false;
    double best_descent_jump = std::numeric_limits<double>::infinity();
    double best_margin = 0.0;
    std::string last_failure_code;
    std::string last_failure_detail;
    int solved_pairs = 0;
    int skipped_limits = 0;
    int skipped_residual = 0;

    for (int attempt = 0; attempt < ctx_->p2a_ik_attempts_; ++attempt) {
        moveit::core::RobotState trial_pre(*start);
        moveit::core::RobotState trial_grasp(*start);

        bool solved = trial_pre.setFromIK(left_jmg, toEigenPose(pre.left), ctx_->p2a_ik_timeout_sec_) &&
                      trial_pre.setFromIK(right_jmg, toEigenPose(pre.right), ctx_->p2a_ik_timeout_sec_) &&
                      trial_grasp.setFromIK(left_jmg, toEigenPose(grasp.left), ctx_->p2a_ik_timeout_sec_) &&
                      trial_grasp.setFromIK(right_jmg, toEigenPose(grasp.right), ctx_->p2a_ik_timeout_sec_);
        if (!solved) {
            last_failure_code = "E_IK_NO_SOLUTION";
            last_failure_detail = "attempt " + std::to_string(attempt + 1) +
                                  ": no combined dual-arm solution for both targets";
            continue;
        }
        ++solved_pairs;
        trial_pre.update();
        trial_grasp.update();

        // (a) joint limits of both arms, (b) FK residual against the target,
        //     for both states.
        bool candidate_ok = true;
        double candidate_margin = std::numeric_limits<double>::infinity();
        const moveit::core::RobotState* trial_states[2] = {&trial_pre, &trial_grasp};
        for (int si = 0; si < 2 && candidate_ok; ++si) {
            const moveit::core::RobotState& tst = *trial_states[si];
            std::vector<double> q, lo, hi;
            std::vector<std::string> names;
            collectArmJoints(tst, left_jmg, q, lo, hi, names);
            collectArmJoints(tst, right_jmg, q, lo, hi, names);
            const a1v::JointLimitCheck lim = a1v::checkJointLimits(q, lo, hi, names);
            if (!lim.ok) {
                candidate_ok = false;
                ++skipped_limits;
                last_failure_code = "E_IK_JOINT_LIMITS";
                last_failure_detail = std::string(si == 0 ? "pre-grasp" : "grasp") + ": joint " +
                                      lim.offending_name + " = " + std::to_string(lim.value) +
                                      " outside [" + std::to_string(lim.lower) + ", " +
                                      std::to_string(lim.upper) + "]";
                break;
            }
            candidate_margin = std::min(candidate_margin, lim.worst_margin_rad);

            const a1v::ArmTarget actual_left = fromEigenPose(tst.getGlobalLinkTransform(eefs[0]));
            const a1v::ArmTarget actual_right = fromEigenPose(tst.getGlobalLinkTransform(eefs[1]));
            const a1v::PoseError el = a1v::poseError(targets[si].left, actual_left);
            const a1v::PoseError er = a1v::poseError(targets[si].right, actual_right);
            if (el.pos_m > pos_tol || er.pos_m > pos_tol || el.angle_rad > ang_tol ||
                er.angle_rad > ang_tol) {
                candidate_ok = false;
                ++skipped_residual;
                last_failure_code = "E_IK_RESIDUAL";
                last_failure_detail = std::string(si == 0 ? "pre-grasp" : "grasp") +
                                      ": FK residual exceeds tolerance (tolerances " +
                                      std::to_string(pos_tol * 1000.0) + " mm / " +
                                      std::to_string(ang_tol * 180.0 / M_PI) + " deg)";
                break;
            }
        }
        if (!candidate_ok) {
            continue;
        }

        // (c) score by the descent change -- the quantity the gate below judges.
        double candidate_jump = 0.0;
        for (int ai = 0; ai < 2; ++ai) {
            const moveit::core::JointModelGroup* jmg = ai == 0 ? left_jmg : right_jmg;
            std::vector<double> qp, qg, l3, u3;
            std::vector<std::string> n3;
            collectArmJoints(trial_pre, jmg, qp, l3, u3, n3);
            collectArmJoints(trial_grasp, jmg, qg, l3, u3, n3);
            candidate_jump = std::max(candidate_jump, a1v::maxAbsJointDelta(qp, qg));
        }

        if (!have_candidate || candidate_jump < best_descent_jump) {
            have_candidate = true;
            best_descent_jump = candidate_jump;
            best_margin = candidate_margin;
            best_pre = trial_pre;
            best_grasp = trial_grasp;
        }
    }

    moveit::core::RobotState& pre_state = best_pre;
    moveit::core::RobotState& grasp_state = best_grasp;

    if (!have_candidate) {
        out.ik.status = a1v::CheckStatus::FAILED;
        out.ik.reason_code = last_failure_code.empty() ? "E_IK_NO_SOLUTION" : last_failure_code;
        out.ik.detail = (last_failure_detail.empty() ? std::string("no solution in any attempt")
                                                     : last_failure_detail) +
                        " (" + std::to_string(ctx_->p2a_ik_attempts_) + " attempts)";
        RCLCPP_ERROR(ctx_->get_logger(), "    ✗ %s", out.ik.detail.c_str());
        out.collision.status = a1v::CheckStatus::NOT_RUN;
        out.collision.reason_code = "E_IK_NOT_SOLVED";
        out.collision.detail = "collision is judged on a solved combined state";
        return emit(false);
    }

    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2A][IK] %d/%d 次尝试同时解出双臂（关节限位剔除 %d，残差剔除 %d）；"
                "已选下降变化最小者",
                solved_pairs, ctx_->p2a_ik_attempts_, skipped_limits, skipped_residual);

    bool ik_ok = true;
    std::string ik_reason = "E_OK";
    std::string ik_detail = "both combined states solved, within limits, FK residual and jump checks";

    // Report the chosen pair.
    {
        const moveit::core::RobotState* final_states[2] = {&pre_state, &grasp_state};
        const char* labels[2] = {"pre-grasp", "grasp"};
        for (int si = 0; si < 2; ++si) {
            const moveit::core::RobotState& tst = *final_states[si];
            std::vector<double> q, lo, hi;
            std::vector<std::string> names;
            collectArmJoints(tst, left_jmg, q, lo, hi, names);
            collectArmJoints(tst, right_jmg, q, lo, hi, names);
            const a1v::JointLimitCheck lim = a1v::checkJointLimits(q, lo, hi, names);
            const a1v::ArmTarget actual_left = fromEigenPose(tst.getGlobalLinkTransform(eefs[0]));
            const a1v::ArmTarget actual_right = fromEigenPose(tst.getGlobalLinkTransform(eefs[1]));
            const a1v::PoseError el = a1v::poseError(targets[si].left, actual_left);
            const a1v::PoseError er = a1v::poseError(targets[si].right, actual_right);

            std::vector<double> q_start, l2, u2;
            std::vector<std::string> n2;
            collectArmJoints(*start, left_jmg, q_start, l2, u2, n2);
            collectArmJoints(*start, right_jmg, q_start, l2, u2, n2);

            RCLCPP_INFO(ctx_->get_logger(),
                        "    [%s] 关节限位余量 %.4f rad | FK 残差 左=%.4f mm/%.4f° 右=%.4f mm/%.4f°",
                        labels[si], lim.worst_margin_rad, el.pos_m * 1000.0,
                        el.angle_rad * 180.0 / M_PI, er.pos_m * 1000.0, er.angle_rad * 180.0 / M_PI);
            RCLCPP_INFO(ctx_->get_logger(),
                        "    [%s] 相对机器人起始位形的最大单关节差 %.4f rad（仅记录，不作判据："
                        "起始位形不是抓取起点，且抓取姿态按设计相差 180° 腕部翻转）",
                        labels[si], a1v::maxAbsJointDelta(q, q_start));
        }
    }

    // (d) The gate: the descent must not re-configure an arm relative to the
    //     pre-grasp solution. An elbow flip between the two IK solutions would
    //     make the descent impossible even though each pose is valid alone.
    for (int ai = 0; ai < 2 && ik_ok; ++ai) {
        const moveit::core::JointModelGroup* jmg = ai == 0 ? left_jmg : right_jmg;
        std::vector<double> qp, qg, l3, u3;
        std::vector<std::string> n3;
        collectArmJoints(pre_state, jmg, qp, l3, u3, n3);
        collectArmJoints(grasp_state, jmg, qg, l3, u3, n3);
        const double descent_jump = a1v::maxAbsJointDelta(qp, qg);
        const char* arm = ai == 0 ? "左臂" : "右臂";
        RCLCPP_INFO(ctx_->get_logger(), "    [预抓取→抓取] %s 最大单关节变化 %.4f rad（上限 %.4f）",
                    arm, descent_jump, ctx_->p2a_max_joint_jump_rad_);
        if (descent_jump > ctx_->p2a_max_joint_jump_rad_) {
            ik_ok = false;
            ik_reason = "E_IK_DESCENT_JUMP";
            ik_detail = std::string(arm) + ": pre-grasp -> grasp changes one joint by " +
                        std::to_string(descent_jump) + " rad, above " +
                        std::to_string(ctx_->p2a_max_joint_jump_rad_) + " rad (best of " +
                        std::to_string(solved_pairs) + " solved attempts)";
        }
    }
    (void)best_margin;
    if (!ik_ok) {
        out.ik.status = a1v::CheckStatus::FAILED;
        out.ik.reason_code = ik_reason;
        out.ik.detail = ik_detail;
        RCLCPP_ERROR(ctx_->get_logger(), "    ✗ %s", ik_detail.c_str());
        out.collision.status = a1v::CheckStatus::NOT_RUN;
        out.collision.reason_code = "E_IK_NOT_VALID";
        out.collision.detail = "collision is judged on states that passed the IK checks";
        return emit(false);
    }
    out.ik.status = a1v::CheckStatus::PASSED;
    out.ik.reason_code = ik_reason;
    out.ik.detail = ik_detail;

    // Log the joint solutions so the record carries them.
    {
        std::vector<double> q, lo, hi;
        std::vector<std::string> names;
        collectArmJoints(grasp_state, left_jmg, q, lo, hi, names);
        std::string line;
        for (std::size_t i = 0; i < q.size(); ++i) {
            line += names[i] + "=" + std::to_string(q[i]) + " ";
        }
        RCLCPP_INFO(ctx_->get_logger(), "    [grasp] 左臂关节解: %s", line.c_str());
        q.clear(); lo.clear(); hi.clear(); names.clear();
        collectArmJoints(grasp_state, right_jmg, q, lo, hi, names);
        line.clear();
        for (std::size_t i = 0; i < q.size(); ++i) {
            line += names[i] + "=" + std::to_string(q[i]) + " ";
        }
        RCLCPP_INFO(ctx_->get_logger(), "    [grasp] 右臂关节解: %s", line.c_str());
    }

    // -----------------------------------------------------------------------
    // 4. Collision on the live planning scene
    // -----------------------------------------------------------------------
    const std::string rod_id = "aluminum_rod";

    auto scene_client =
        ctx_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!scene_client->wait_for_service(5s)) {
        out.collision.status = a1v::CheckStatus::FAILED;
        out.collision.reason_code = "E_P2A_SCENE_UNAVAILABLE";
        out.collision.detail = "/get_planning_scene is unavailable, so the environment cannot be checked";
        RCLCPP_ERROR(ctx_->get_logger(), "    ✗ %s", out.collision.detail.c_str());
        return emit(false);
    }
    auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    scene_request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto scene_future = scene_client->async_send_request(scene_request);
    if (scene_future.wait_for(5s) != std::future_status::ready) {
        out.collision.status = a1v::CheckStatus::FAILED;
        out.collision.reason_code = "E_P2A_SCENE_TIMEOUT";
        out.collision.detail = "/get_planning_scene did not answer within 5 s";
        RCLCPP_ERROR(ctx_->get_logger(), "    ✗ %s", out.collision.detail.c_str());
        return emit(false);
    }
    const auto scene_response = scene_future.get();

    // Refuse to claim an environment check if the rod is not actually in the
    // scene we are about to test against.
    bool rod_in_scene = false;
    for (const auto& obj : scene_response->scene.world.collision_objects) {
        if (obj.id == rod_id) {
            rod_in_scene = true;
            break;
        }
    }
    RCLCPP_INFO(ctx_->get_logger(), "  [P2A][碰撞] 场景来源 /get_planning_scene：世界物体 %zu 个，"
                                    "aluminum_rod 在场景中: %s",
                scene_response->scene.world.collision_objects.size(),
                rod_in_scene ? "是" : "否");
    if (!rod_in_scene) {
        out.collision.status = a1v::CheckStatus::FAILED;
        out.collision.reason_code = "E_P2A_ROD_NOT_IN_SCENE";
        out.collision.detail = "aluminum_rod is absent from the live scene; no environment claim is made";
        RCLCPP_ERROR(ctx_->get_logger(), "    ✗ %s", out.collision.detail.c_str());
        return emit(false);
    }

    // Build the verification scene from what the live planner actually has --
    // never a bare robot_model-only scene.
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    scene->setPlanningSceneMsg(scene_response->scene);

    // Verification copy of the ACM. Any entry that would hide a dual-arm mutual
    // collision, or any contact between the rod and a non-finger link, is
    // turned OFF in this copy only. The global SRDF/ACM is not modified.
    collision_detection::AllowedCollisionMatrix base_acm = scene->getAllowedCollisionMatrix();

    // The support surface is NOT in the MoveIt planning scene: the MuJoCo floor
    // is a `type="plane"` geom in dual_scene.xml that never enters MoveIt. A rod
    // pose/height check is not a robot-vs-ground check, so the plane is added to
    // this VERIFICATION scene only, via processCollisionObjectMsg, which touches
    // the local PlanningScene and never publishes.
    const std::string support_id = ctx_->p2a_support_object_id_;
    bool support_already_present = false;
    {
        const std::vector<std::string> world_ids = scene->getWorld()->getObjectIds();
        for (const std::string& id : world_ids) {
            if (id == support_id) {
                support_already_present = true;
            }
        }
        std::string joined;
        for (const std::string& id : world_ids) {
            joined += " " + id;
        }
        RCLCPP_INFO(ctx_->get_logger(), "  [P2A][碰撞] 验证场景世界物体:%s",
                    joined.empty() ? " <none>" : joined.c_str());
    }

    if (ctx_->p2a_support_plane_) {
        if (support_already_present) {
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P2A][支撑面] 场景中已存在 '%s'，不重复添加", support_id.c_str());
        } else {
            // Source: franka_description/mujoco/franka/dual_scene.xml
            //   <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
            // i.e. an unbounded plane at z = support_z (the geom has no `pos`).
            //
            // It is represented as a slab whose TOP FACE lies exactly on that
            // plane: shape_msgs/SolidPrimitive in this ROS distro has no PLANE
            // constant (only BOX/SPHERE/CYLINDER/CONE/PRISM), so a plane cannot
            // be handed to MoveIt through a CollisionObject. A robot point below
            // z = support_z is inside the slab, so this is faithful for the
            // collision question; the slab is finite, which is recorded as a
            // limitation rather than hidden.
            const double slab_half = 3.0;
            // A DEEP box, not a thin slab: the top face stays on the support
            // plane, and it now covers the whole depth a path could reach.
            const double slab_thickness = ctx_->p2a_support_depth_m_;
            moveit_msgs::msg::CollisionObject floor_obj;
            floor_obj.header.frame_id = scene->getPlanningFrame();
            floor_obj.id = support_id;
            shape_msgs::msg::SolidPrimitive slab;
            slab.type = shape_msgs::msg::SolidPrimitive::BOX;
            slab.dimensions = {2.0 * slab_half, 2.0 * slab_half, slab_thickness};
            floor_obj.primitives.push_back(slab);
            geometry_msgs::msg::Pose slab_pose;
            slab_pose.position.z = ctx_->p2a_support_z_ - slab_thickness * 0.5;
            slab_pose.orientation.w = 1.0;
            floor_obj.primitive_poses.push_back(slab_pose);
            floor_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
            // Local verification scene only; nothing is published.
            scene->processCollisionObjectMsg(floor_obj);
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P2A][支撑面] 已在**验证场景**中补齐 '%s': BOX %.1fx%.1fx%.3f，顶面位于 "
                        "z=%.3f 向下覆盖 %.1f m (frame=%s)，来源 dual_scene.xml 的 floor geom"
                        "（无限 plane，此处用有限深板近似；超出 ±%.1f m 或深于 %.1f m 处不覆盖）",
                        support_id.c_str(), 2.0 * slab_half, 2.0 * slab_half, slab_thickness,
                        ctx_->p2a_support_z_, slab_thickness, scene->getPlanningFrame().c_str(),
                        slab_half, slab_thickness);
        }
    } else {
        RCLCPP_WARN(ctx_->get_logger(),
                    "  [P2A][支撑面] 已禁用，本轮**不包含**机器人与地面的碰撞检查");
    }

    std::vector<std::string> acm_names;
    base_acm.getAllEntryNames(acm_names);
    std::size_t cross_arm_allowed = 0;
    std::size_t rod_allowed = 0;
    std::vector<std::string> stripped_examples;
    for (std::size_t i = 0; i < acm_names.size(); ++i) {
        for (std::size_t j = i + 1; j < acm_names.size(); ++j) {
            collision_detection::AllowedCollision::Type type;
            if (!base_acm.getEntry(acm_names[i], acm_names[j], type) ||
                type != collision_detection::AllowedCollision::ALWAYS) {
                continue;
            }
            if (a1v::isCrossArmPair(acm_names[i], acm_names[j])) {
                base_acm.setEntry(acm_names[i], acm_names[j], false);
                ++cross_arm_allowed;
                if (stripped_examples.size() < 10) {
                    stripped_examples.push_back(acm_names[i] + "<->" + acm_names[j]);
                }
            } else if (a1v::isRodPair(acm_names[i], acm_names[j], rod_id)) {
                base_acm.setEntry(acm_names[i], acm_names[j], false);
                ++rod_allowed;
                if (stripped_examples.size() < 10) {
                    stripped_examples.push_back(acm_names[i] + "<->" + acm_names[j]);
                }
            }
        }
    }

    std::string examples;
    for (const std::string& e : stripped_examples) {
        examples += " " + e;
    }
    RCLCPP_INFO(ctx_->get_logger(),
                "    ACM 条目核查: 共 %zu 个条目；会隐藏双臂互碰的允许项 %zu 个，"
                "会隐藏杆接触的允许项 %zu 个（已在验证副本中关闭，未改动全局 ACM/SRDF）%s",
                acm_names.size(), cross_arm_allowed, rod_allowed,
                examples.empty() ? "" : ("；示例:" + examples).c_str());

    // The grasp pose is the only one where finger-rod contact is expected, and
    // only for the finger links -- the hand and the rest of the arm stay
    // checked.
    collision_detection::AllowedCollisionMatrix grasp_acm = base_acm;
    const char* fingers[4] = {"mj_left_leftfinger", "mj_left_rightfinger", "mj_right_leftfinger",
                              "mj_right_rightfinger"};
    for (const char* f : fingers) {
        grasp_acm.setEntry(f, rod_id, true);
    }

    // The base is bolted to the world: base_link and both link0 bodies cross the
    // support plane by construction. That is the fixed mounting, not a free-link
    // collision, so exactly these three pairs are allowed -- no other link is
    // exempted from the ground, and no arm link is exempted at all.
    if (ctx_->p2a_support_plane_) {
        // Fixed base assembly only. Justification is geometric, not blanket:
        // each of these has collision geometry straddling z = support_z whose
        // z-extent is independent of the arm configuration (see
        // a1v::isMountingLink). The hand and fingers are NOT included.
        const char* mounting[5] = {"base_link", "mj_left_link0", "mj_right_link0", "mj_left_link1",
                                   "mj_right_link1"};
        for (const char* m : mounting) {
            base_acm.setEntry(m, support_id, true);
            grasp_acm.setEntry(m, support_id, true);
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "    [P2A][支撑面] 仅允许固定安装配对 base_link/link0/link1（两臂）与 '%s' "
                    "接触（这些碰撞体跨越安装平面且不随臂位形变化）；手部与手指**不在**豁免内，"
                    "其余连杆与地面的接触一律检查",
                    support_id.c_str());
    }

    ContactTally pre_tally;
    ContactTally grasp_tally;
    pre_tally = tallyContactsIn(scene, robot_model, pre_state, base_acm, rod_id, support_id);
    grasp_tally = tallyContactsIn(scene, robot_model, grasp_state, grasp_acm, rod_id, support_id);

    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2A][对照 model=urdf_reference] 原始 URDF 六体手掌近似，仅作对照，非本轮判据");
    RCLCPP_INFO(ctx_->get_logger(), "    [预抓取 z=%.3f] 不允许任何杆接触: %s", gg.pre_grasp_height_m,
                describeTally(pre_tally).c_str());
    RCLCPP_INFO(ctx_->get_logger(),
                "    [抓取 z=%.3f] 仅允许手指-杆接触（hand/其余连杆仍检查）: %s", gg.grasp_height_m,
                describeTally(grasp_tally).c_str());

    // =======================================================================
    // Chosen model: mj_left_hand / mj_right_hand collision replaced by the
    // conservative AABB of the simulation's actual hand_c mesh. This model is
    // built independently; the shared RobotModel and the live scene are
    // untouched.
    // =======================================================================
    const std::vector<std::string> hand_links = {"mj_left_hand", "mj_right_hand"};
    bool hand_aabb_ok = false;
    a1v::Aabb hand_aabb;
    std::size_t hand_mesh_vertices = 0;
    std::string mesh_path;

    if (ctx_->p2a_hand_aabb_substitution_) {
        try {
            mesh_path = ament_index_cpp::get_package_share_directory("franka_description") +
                        "/mujoco/franka/assets/hand.stl";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2A][hand] E_HAND_MESH_PKG_NOT_FOUND: %s", e.what());
            out.collision.status = a1v::CheckStatus::FAILED;
            out.collision.reason_code = "E_HAND_MESH_PKG_NOT_FOUND";
            out.collision.detail = e.what();
            return emit(false);
        }

        std::vector<a1v::Vec3> verts;
        std::string mesh_error;
        if (!a1v::readBinaryStlVertices(mesh_path, verts, mesh_error)) {
            // An unreadable mesh stops the check: an empty model must never be
            // used in place of the real one.
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2A][hand] E_HAND_MESH_LOAD_FAILED: %s",
                         mesh_error.c_str());
            out.collision.status = a1v::CheckStatus::FAILED;
            out.collision.reason_code = "E_HAND_MESH_LOAD_FAILED";
            out.collision.detail = mesh_error;
            return emit(false);
        }
        hand_mesh_vertices = verts.size();

        const a1v::Aabb box = a1v::aabbOfPoints(verts);
        // A loaded-but-conservative check: the box must contain every vertex,
        // otherwise it is not a bounding proxy at all.
        if (!a1v::aabbContainsAll(box, verts)) {
            RCLCPP_ERROR(ctx_->get_logger(),
                         "  ✗ [P2A][hand] E_HAND_AABB_NOT_CONSERVATIVE: 包围盒未覆盖全部顶点");
            out.collision.status = a1v::CheckStatus::FAILED;
            out.collision.reason_code = "E_HAND_AABB_NOT_CONSERVATIVE";
            out.collision.detail = "the computed box does not contain all mesh vertices";
            return emit(false);
        }
        hand_aabb = box;
        hand_aabb_ok = true;

        const a1v::Vec3 c = hand_aabb.center();
        const a1v::Vec3 sz = hand_aabb.size();
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2A][hand] 模型来源: %s（%zu 顶点，二进制 STL，未施加额外缩放）",
                    mesh_path.c_str(), hand_mesh_vertices);
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2A][hand] hand_c 局部 AABB: x[%.6f, %.6f] y[%.6f, %.6f] z[%.6f, %.6f]",
                    hand_aabb.min.x, hand_aabb.max.x, hand_aabb.min.y, hand_aabb.max.y,
                    hand_aabb.min.z, hand_aabb.max.z);
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2A][hand] 替换为 BOX 中心=(%.6f, %.6f, %.6f) 尺寸=(%.6f, %.6f, %.6f)"
                    "（中心为 (min+max)/2，非 hand 原点；已断言包含全部顶点）",
                    c.x, c.y, c.z, sz.x, sz.y, sz.z);
    }

    std::shared_ptr<moveit::core::RobotModel> mujoco_model;
    planning_scene::PlanningScenePtr mujoco_scene;
    moveit::core::RobotState mj_pre_state(*start);
    moveit::core::RobotState mj_grasp_state(*start);
    ContactTally mj_pre_tally;
    ContactTally mj_grasp_tally;

    if (hand_aabb_ok) {
        std::string build_reason;
        mujoco_model = buildMujocoHandModel(hand_aabb, hand_links, build_reason);
        if (!mujoco_model) {
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2A][hand] %s", build_reason.c_str());
            out.collision.status = a1v::CheckStatus::FAILED;
            out.collision.reason_code = build_reason.substr(0, build_reason.find(':'));
            out.collision.detail = build_reason;
            return emit(false);
        }

        // Same world as the reference scene: the very same message plus the
        // support surface, so the two models are compared on identical worlds.
        mujoco_scene = std::make_shared<planning_scene::PlanningScene>(mujoco_model);
        mujoco_scene->setPlanningSceneMsg(scene_response->scene);
        if (ctx_->p2a_support_plane_) {
            moveit_msgs::msg::CollisionObject floor_obj;
            floor_obj.header.frame_id = mujoco_scene->getPlanningFrame();
            floor_obj.id = support_id;
            shape_msgs::msg::SolidPrimitive slab;
            slab.type = shape_msgs::msg::SolidPrimitive::BOX;
            slab.dimensions = {6.0, 6.0, ctx_->p2a_support_depth_m_};
            floor_obj.primitives.push_back(slab);
            geometry_msgs::msg::Pose slab_pose;
            slab_pose.position.z = ctx_->p2a_support_z_ - ctx_->p2a_support_depth_m_ * 0.5;
            slab_pose.orientation.w = 1.0;
            floor_obj.primitive_poses.push_back(slab_pose);
            floor_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
            mujoco_scene->processCollisionObjectMsg(floor_obj);
        }

        // Map the solved states across by joint name.
        auto mapState = [&](const moveit::core::RobotState& src) {
            moveit::core::RobotState dst(mujoco_model);
            std::vector<std::string> names;
            std::vector<double> values;
            std::set<std::string> dst_vars;
            for (const std::string& v : dst.getVariableNames()) {
                dst_vars.insert(v);
            }
            for (const std::string& v : src.getVariableNames()) {
                if (dst_vars.count(v) != 0) {
                    names.push_back(v);
                    values.push_back(src.getVariablePosition(v));
                }
            }
            dst.setVariablePositions(names, values);
            dst.update();
            return dst;
        };
        mj_pre_state = mapState(pre_state);
        mj_grasp_state = mapState(grasp_state);

        // The mapping is only trustworthy if it reproduces the same FK; verify
        // it rather than assuming the two models agree.
        const a1v::ArmTarget mj_left_actual =
            fromEigenPose(mj_grasp_state.getGlobalLinkTransform(left_eef));
        const a1v::PoseError map_err = a1v::poseError(grasp.left, mj_left_actual);
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2A][hand] 状态按同名关节映射后 FK 残差 %.6f mm / %.6f°（左臂抓取目标）",
                    map_err.pos_m * 1000.0, map_err.angle_rad * 180.0 / M_PI);

        // The ACM is name-based, so the same matrix applies to both models.
        mj_pre_tally = tallyContactsIn(mujoco_scene, mujoco_model, mj_pre_state, base_acm, rod_id, support_id);
        mj_grasp_tally = tallyContactsIn(mujoco_scene, mujoco_model, mj_grasp_state, grasp_acm, rod_id, support_id);

        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2A][判据 model=mujoco_hand_aabb] 手掌替换为 hand_c AABB，其余连杆/手指/"
                    "场景/ACM 语义不变");
        RCLCPP_INFO(ctx_->get_logger(), "    [预抓取 z=%.3f] %s", gg.pre_grasp_height_m,
                    describeTally(mj_pre_tally).c_str());
        RCLCPP_INFO(ctx_->get_logger(), "    [抓取 z=%.3f] %s", gg.grasp_height_m,
                    describeTally(mj_grasp_tally).c_str());

        // Hand the criterion over to P2b: same model, same world, same ACM,
        // same recorded start and pre-grasp solution.
        a1_verify_.valid = true;
        a1_verify_.model = mujoco_model;
        a1_verify_.scene = mujoco_scene;
        a1_verify_.base_acm = base_acm;
        a1_verify_.grasp_acm = grasp_acm;
        a1_verify_.rod_id = rod_id;
        a1_verify_.support_id = support_id;
        a1_verify_.mesh_path = hand_aabb_ok ? mesh_path : std::string();
        a1_verify_.hand_aabb = hand_aabb;
        a1_verify_.pregrasp_target = pre;
        a1_verify_.left_eef = left_eef;
        a1_verify_.right_eef = right_eef;
        a1_verify_.start = std::make_shared<moveit::core::RobotState>(mapState(*start));
        a1_verify_.pregrasp = std::make_shared<moveit::core::RobotState>(mj_pre_state);
        a1_verify_.grasp = std::make_shared<moveit::core::RobotState>(mj_grasp_state);
        for (const std::string& v : pre_state.getVariableNames()) {
            a1_verify_.pregrasp_joints[v] = pre_state.getVariablePosition(v);
        }
    } else {
        RCLCPP_WARN(ctx_->get_logger(),
                    "  [P2A][判据] hand AABB 替换已禁用，回退到 model=urdf_reference 作为判据");
    }

    const bool use_mujoco = hand_aabb_ok;
    const ContactTally& chosen_pre = use_mujoco ? mj_pre_tally : pre_tally;
    const ContactTally& chosen_grasp = use_mujoco ? mj_grasp_tally : grasp_tally;
    const char* chosen_name = use_mujoco ? "mujoco_hand_aabb" : "urdf_reference";

    // Negative example for the substituted hand itself: an obstacle placed to
    // coincide with the substituted hand box must be detected. This proves the
    // replacement box still participates in collision checking instead of
    // quietly disabling the hand.
    if (use_mujoco && ctx_->p2a_hand_negative_) {
        const std::string obstacle_id = "a1_hand_obstacle_injected";
        const a1v::Vec3 sz = hand_aabb.size();
        moveit_msgs::msg::CollisionObject obstacle;
        obstacle.header.frame_id = mujoco_scene->getPlanningFrame();
        obstacle.id = obstacle_id;
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {sz.x, sz.y, sz.z};
        obstacle.primitives.push_back(box);
        // Placed at the substituted box's own centre (the AABB centre, which is
        // NOT the hand origin) and with the hand's orientation, so the obstacle
        // truly coincides with the replacement geometry.
        const Eigen::Isometry3d hand_world =
            mj_grasp_state.getGlobalLinkTransform("mj_left_hand");
        const a1v::Vec3 box_centre = hand_aabb.center();
        Eigen::Isometry3d obstacle_world = hand_world;
        obstacle_world.translation() +=
            hand_world.rotation() * Eigen::Vector3d(box_centre.x, box_centre.y, box_centre.z);
        geometry_msgs::msg::Pose obstacle_pose;
        obstacle_pose.position.x = obstacle_world.translation().x();
        obstacle_pose.position.y = obstacle_world.translation().y();
        obstacle_pose.position.z = obstacle_world.translation().z();
        const Eigen::Quaterniond hand_q(obstacle_world.rotation());
        obstacle_pose.orientation.x = hand_q.x();
        obstacle_pose.orientation.y = hand_q.y();
        obstacle_pose.orientation.z = hand_q.z();
        obstacle_pose.orientation.w = hand_q.w();
        obstacle.primitive_poses.push_back(obstacle_pose);
        obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
        mujoco_scene->processCollisionObjectMsg(obstacle);

        ContactTally neg_tally = tallyContactsIn(mujoco_scene, mujoco_model, mj_grasp_state,
                                                grasp_acm, rod_id, support_id);

        obstacle.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        mujoco_scene->processCollisionObjectMsg(obstacle);

        bool obstacle_seen = false;
        for (const std::string& e : neg_tally.examples) {
            if (e.find(obstacle_id) != std::string::npos) {
                obstacle_seen = true;
            }
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "    [手掌负例-注入] 在与替换后手掌 AABB 同中心同姿态处注入障碍 '%s'"
                    "（验证注入，非实际机器人运动）: %s",
                    obstacle_id.c_str(), describeTally(neg_tally).c_str());
        if (neg_tally.total() == 0 || !obstacle_seen) {
            out.collision.status = a1v::CheckStatus::FAILED;
            out.collision.reason_code = "E_P2A_HAND_NEGATIVE_MISSED";
            out.collision.detail =
                "an obstacle coincident with the substituted hand box was not reported, so the "
                "replacement geometry is not actually being checked";
            RCLCPP_ERROR(ctx_->get_logger(),
                         "    ✗ 替换后的手掌未检出与该障碍的碰撞 —— 替换几何未参与检查，按失败处理");
            return emit(false);
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "    ✓ [手掌负例-注入] 替换后的手掌仍然检出碰撞，替换几何确实参与检查");
    }

    // Ground-collision negative, injected on purpose. It proves the completed
    // scene can actually reject a free arm link passing through the support
    // surface. This is a VERIFICATION INJECTION, not a sampled pose: the target
    // is deliberately placed below the floor.
    if (ctx_->p2a_support_plane_ && ctx_->p2a_ground_negative_) {
        const double injected_z = ctx_->p2a_support_z_ - 0.05;
        a1v::ArmTarget injected_left = pre.left;
        a1v::ArmTarget injected_right = pre.right;
        injected_left.position.z = injected_z;
        injected_right.position.z = injected_z;

        bool injected_solved = false;
        moveit::core::RobotState injected(*start);
        for (int attempt = 0; attempt < ctx_->p2a_ik_attempts_ && !injected_solved; ++attempt) {
            moveit::core::RobotState trial(*start);
            injected_solved =
                trial.setFromIK(left_jmg, toEigenPose(injected_left), ctx_->p2a_ik_timeout_sec_) &&
                trial.setFromIK(right_jmg, toEigenPose(injected_right), ctx_->p2a_ik_timeout_sec_);
            if (injected_solved) {
                injected = trial;
            }
        }

        if (!injected_solved) {
            RCLCPP_WARN(ctx_->get_logger(),
                        "    [地面负例-注入] 未能为 z=%.3f 构造 IK 解；本次注入未执行"
                        "（不视为检查通过）",
                        injected_z);
        } else {
            injected.update();
            ContactTally injected_tally;
            if (use_mujoco) {
                auto mapInjected = [&](const moveit::core::RobotState& src) {
                    moveit::core::RobotState dst(mujoco_model);
                    std::vector<std::string> names;
                    std::vector<double> values;
                    std::set<std::string> dst_vars;
                    for (const std::string& v : dst.getVariableNames()) {
                        dst_vars.insert(v);
                    }
                    for (const std::string& v : src.getVariableNames()) {
                        if (dst_vars.count(v) != 0) {
                            names.push_back(v);
                            values.push_back(src.getVariablePosition(v));
                        }
                    }
                    dst.setVariablePositions(names, values);
                    dst.update();
                    return dst;
                };
                moveit::core::RobotState mj_injected = mapInjected(injected);
                injected_tally = tallyContactsIn(mujoco_scene, mujoco_model, mj_injected, base_acm,
                                                 rod_id, support_id);
            } else {
                injected_tally =
                    tallyContactsIn(scene, robot_model, injected, base_acm, rod_id, support_id);
            }
            RCLCPP_INFO(ctx_->get_logger(),
                        "    [地面负例-注入] 目标 z=%.3f（低于支撑面 %.3f，非采样位姿）: %s", injected_z,
                        ctx_->p2a_support_z_, describeTally(injected_tally).c_str());
            if (injected_tally.total() == 0) {
                out.collision.status = a1v::CheckStatus::FAILED;
                out.collision.reason_code = "E_P2A_GROUND_NEGATIVE_MISSED";
                out.collision.detail =
                    "the injected below-floor state produced no contact, so the support-surface "
                    "check is not proven to work";
                RCLCPP_ERROR(ctx_->get_logger(),
                             "    ✗ 注入的穿地状态未被拒绝 —— 支撑面检查不可信，按失败处理");
                return emit(false);
            }
            RCLCPP_INFO(ctx_->get_logger(),
                        "    ✓ [地面负例-注入] 穿地状态被正确拒绝（地面/支撑面接触 %zu 处）",
                        injected_tally.support);
        }
    }

    if (chosen_pre.total() > 0 || chosen_grasp.total() > 0) {
        out.collision.status = a1v::CheckStatus::FAILED;
        out.collision.reason_code = chosen_pre.total() > 0 ? "E_P2A_COLLISION_PREGRASP"
                                                           : "E_P2A_COLLISION_GRASP";
        out.collision.detail = std::string("model=") + chosen_name + " pre-grasp{" +
                               describeTally(chosen_pre) + "} grasp{" + describeTally(chosen_grasp) + "}";
        RCLCPP_ERROR(ctx_->get_logger(), "    ✗ [model=%s] 组合状态存在碰撞", chosen_name);
        return emit(false);
    }

    out.collision.status = a1v::CheckStatus::PASSED;
    out.collision.reason_code = "E_OK";
    out.collision.detail = std::string("model=") + chosen_name +
                           ": no environment, self or dual-arm collision in either combined state";

    RCLCPP_INFO(ctx_->get_logger(), "  [P2A] 静态检查汇总: %s", out.summary().c_str());
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2A] 注意: 静态通过仅表示该组合目标本身有效；"
                "p2_acceptance=null, path_validation=NOT_RUN，可搬运性由 P2b 判定");
    // Single machine-readable line for the run record.
    RCLCPP_INFO(ctx_->get_logger(),
                "[P2A_RESULT] geometry=%s ik=%s collision=%s path_validation=NOT_RUN "
                "p2_acceptance=null",
                a1v::statusName(out.geometry.status), a1v::statusName(out.ik.status),
                a1v::statusName(out.collision.status));
    return emit(out.allStaticPassed());
}

bool DualArmController::runP2bPathCheck(a1v::SampleValidation& out)
{
    out.path_validation.status = a1v::CheckStatus::NOT_RUN;
    out.path_validation.reason_code = "E_P2B_NOT_RUN";
    out.path_validation.detail = "not evaluated";

    if (!a1_verify_.valid || !a1_verify_.model || !a1_verify_.scene) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_NO_VERIFY_CONTEXT";
        out.path_validation.detail =
            "the P2a criterion model/scene is unavailable; refusing to plan against anything else";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
        return false;
    }
    const auto model = a1_verify_.model;
    const auto scene = a1_verify_.scene;
    const std::string& rod_id = a1_verify_.rod_id;
    const std::string& support_id = a1_verify_.support_id;

    RCLCPP_INFO(ctx_->get_logger(),
                "\n----- [P2B] 联合预抓取路径规划与校验（仅规划，不执行运动）-----");
    RCLCPP_INFO(ctx_->get_logger(),
                "  判据模型 model=mujoco_hand_aabb（手掌为 hand_c AABB），场景为 P2a 独立验证场景");

    // -----------------------------------------------------------------------
    // 1. The recorded start state must be present and valid. The robot is never
    //    teleported and the start is never swapped for a default/ready pose.
    // -----------------------------------------------------------------------
    a1_verify_.start->update();
    const moveit::core::JointModelGroup* left_jmg = model->getJointModelGroup(ctx_->left_arm_group_);
    const moveit::core::JointModelGroup* right_jmg =
        model->getJointModelGroup(ctx_->right_arm_group_);
    if (left_jmg == nullptr || right_jmg == nullptr) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_NO_GROUP";
        out.path_validation.detail = "an arm group is missing from the criterion model";
        return false;
    }

    auto collectAll = [&](const moveit::core::RobotState& st, std::vector<double>& q,
                          std::vector<double>& lo, std::vector<double>& hi,
                          std::vector<std::string>& names) {
        q.clear(); lo.clear(); hi.clear(); names.clear();
        collectArmJoints(st, left_jmg, q, lo, hi, names);
        collectArmJoints(st, right_jmg, q, lo, hi, names);
    };

    std::vector<double> q0, lo0, hi0;
    std::vector<std::string> n0;
    collectAll(*a1_verify_.start, q0, lo0, hi0, n0);
    const a1v::JointLimitCheck start_limits = a1v::checkJointLimits(q0, lo0, hi0, n0);
    if (!start_limits.ok) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_START_INVALID";
        out.path_validation.detail = "the recorded start state is invalid: joint " +
                                     start_limits.offending_name + " = " +
                                     std::to_string(start_limits.value);
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
        return false;
    }

    // The live robot must still be where the static check saw it.
    {
        const moveit::core::RobotStatePtr live = ctx_->dual_arm_->getCurrentState();
        std::set<std::string> live_vars;
        for (const std::string& v : live->getVariableNames()) {
            live_vars.insert(v);
        }
        double worst = 0.0;
        for (const std::string& name : n0) {
            if (live_vars.count(name) != 0) {
                worst = std::max(worst, std::fabs(live->getVariablePosition(name) -
                                                  a1_verify_.start->getVariablePosition(name)));
            }
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2B] 起始状态: 记录值与当前 live 状态最大关节差 %.6f rad（起点不被替换）",
                    worst);
        if (worst > 1e-3) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_START_MOVED";
            out.path_validation.detail = "the robot is no longer at the recorded start (max joint "
                                         "delta " + std::to_string(worst) + " rad)";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
            return false;
        }
    }

    // -----------------------------------------------------------------------
    // 2. Plan with the existing MoveIt interface. This is a real planner call;
    //    a hand-built interpolation would not be reported as a plan.
    // -----------------------------------------------------------------------
    auto dual = ctx_->dual_arm_;
    // Conservative motion scaling for the plan that may be executed.
    dual->setMaxVelocityScalingFactor(ctx_->pregrasp_vel_scale_);
    dual->setMaxAccelerationScalingFactor(ctx_->pregrasp_acc_scale_);
    dual->setPlanningTime(ctx_->p2b_planning_time_sec_);
    dual->setNumPlanningAttempts(ctx_->p2b_planning_attempts_);
    dual->setPlannerId(ctx_->p2b_planner_id_);
    dual->setStartStateToCurrentState();

    moveit::core::RobotState goal(*dual->getCurrentState());
    goal.setVariablePositions(a1_verify_.pregrasp_joints);
    goal.update();
    dual->setJointValueTarget(goal);

    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2B] 调用规划器: planner=%s, time=%.1fs, attempts=%d, 起点=当前实际状态, "
                "终点=本次选定预抓取解, 速度缩放=%.3f, 加速度缩放=%.3f",
                ctx_->p2b_planner_id_.c_str(), ctx_->p2b_planning_time_sec_,
                ctx_->p2b_planning_attempts_, ctx_->pregrasp_vel_scale_,
                ctx_->pregrasp_acc_scale_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_code = dual->plan(plan);
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_PLAN_FAILED";
        out.path_validation.detail = "the planner returned error code " +
                                     std::to_string(plan_code.val) +
                                     "; planning failure is never reported as acceptance";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
        return false;
    }

    const auto& traj = plan.trajectory_.joint_trajectory;
    const auto& traj_names = traj.joint_names;
    if (traj.points.empty() || traj_names.empty()) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_EMPTY_TRAJECTORY";
        out.path_validation.detail = "the planner returned an empty trajectory";
        return false;
    }

    // -----------------------------------------------------------------------
    // 3. Structural validation of the returned path.
    // -----------------------------------------------------------------------
    for (std::size_t pi = 0; pi < traj.points.size(); ++pi) {
        const auto& pos = traj.points[pi].positions;
        if (pos.size() != traj_names.size()) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_DIM_MISMATCH";
            out.path_validation.detail = "point " + std::to_string(pi) + " has " +
                                         std::to_string(pos.size()) + " values for " +
                                         std::to_string(traj_names.size()) + " joint names";
            return false;
        }
        for (std::size_t ji = 0; ji < pos.size(); ++ji) {
            if (!std::isfinite(pos[ji])) {
                out.path_validation.status = a1v::CheckStatus::FAILED;
                out.path_validation.reason_code = "E_P2B_NOT_FINITE";
                out.path_validation.detail = "non-finite value at point " + std::to_string(pi) +
                                             ", joint " + traj_names[ji];
                return false;
            }
        }
    }

    // Both arms must be named exactly once each: a missing joint would leave part
    // of the robot uncommanded and a duplicate or unknown name means the plan is
    // not for this robot.
    {
        const moveit::core::JointModelGroup* dual_jmg =
            model->getJointModelGroup(ctx_->dual_arm_group_);
        if (dual_jmg == nullptr) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_NO_DUAL_GROUP";
            out.path_validation.detail = "the dual-arm group is missing from the criterion model";
            return false;
        }
        const a1v::JointNameCheck name_check =
            a1v::validateJointNames(traj_names, dual_jmg->getVariableNames());
        if (!name_check.ok) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = name_check.reason_code;
            out.path_validation.detail = name_check.detail;
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", name_check.detail.c_str());
            return false;
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2B] 关节名称核对: %zu 个，双臂唯一且完整（期望 %zu），无未知/重复",
                    traj_names.size(), dual_jmg->getVariableNames().size());
    }

    // Timing must be legal even before execution is considered.
    {
        std::vector<double> times;
        times.reserve(traj.points.size());
        for (const auto& pt : traj.points) {
            times.push_back(rclcpp::Duration(pt.time_from_start).seconds());
        }
        const a1v::TimeCheck tc = a1v::validateTimeFromStart(times);
        if (!tc.ok) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = tc.reason_code;
            out.path_validation.detail = tc.detail;
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", tc.detail.c_str());
            return false;
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2B] 时间戳核对: 首 %.6f s 末 %.6f s（严格递增、有限、非负）", tc.first_sec,
                    tc.last_sec);
    }

    // Joint-limit check on every waypoint, in the criterion model.
    for (std::size_t pi = 0; pi < traj.points.size(); ++pi) {
        moveit::core::RobotState st(*a1_verify_.start);
        st.setVariablePositions(traj_names, traj.points[pi].positions);
        st.update();
        std::vector<double> q, lo, hi;
        std::vector<std::string> nm;
        collectAll(st, q, lo, hi, nm);
        const a1v::JointLimitCheck lim = a1v::checkJointLimits(q, lo, hi, nm);
        if (!lim.ok) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_LIMIT_VIOLATION";
            out.path_validation.detail = "waypoint " + std::to_string(pi) + ": joint " +
                                         lim.offending_name + " = " + std::to_string(lim.value) +
                                         " outside limits";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
            return false;
        }
    }

    // Continuity: the maximum single-joint change between adjacent waypoints.
    double max_waypoint_delta = 0.0;
    for (std::size_t pi = 1; pi < traj.points.size(); ++pi) {
        max_waypoint_delta = std::max(
            max_waypoint_delta,
            a1v::maxAbsJointDelta(traj.points[pi - 1].positions, traj.points[pi].positions));
    }

    // Start consistency: the path must begin where the robot actually is.
    {
        std::set<std::string> start_vars;
        for (const std::string& v : a1_verify_.start->getVariableNames()) {
            start_vars.insert(v);
        }
        double worst = 0.0;
        for (std::size_t ji = 0; ji < traj_names.size(); ++ji) {
            if (start_vars.count(traj_names[ji]) == 0) {
                continue;
            }
            worst = std::max(worst, std::fabs(traj.points.front().positions[ji] -
                                              a1_verify_.start->getVariablePosition(traj_names[ji])));
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2B] 路径: %zu 航点, %zu 关节 | 起点与记录起始状态最大差 %.6f rad | "
                    "相邻航点最大单关节变化 %.4f rad",
                    traj.points.size(), traj_names.size(), worst, max_waypoint_delta);
        if (worst > 1e-3) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_PATH_START_MISMATCH";
            out.path_validation.detail = "the path does not start at the recorded start state "
                                         "(max delta " + std::to_string(worst) + " rad)";
            return false;
        }
    }

    // End FK residual against the SAME chosen pre-grasp target.
    auto stateOf = [&](const std::vector<double>& pos) {
        moveit::core::RobotState st(*a1_verify_.start);
        st.setVariablePositions(traj_names, pos);
        st.update();
        return st;
    };
    const moveit::core::RobotState end_state = stateOf(traj.points.back().positions);
    const a1v::ArmTarget end_left =
        fromEigenPose(end_state.getGlobalLinkTransform(a1_verify_.left_eef));
    const a1v::ArmTarget end_right =
        fromEigenPose(end_state.getGlobalLinkTransform(a1_verify_.right_eef));
    const a1v::PoseError end_err_l = a1v::poseError(a1_verify_.pregrasp_target.left, end_left);
    const a1v::PoseError end_err_r = a1v::poseError(a1_verify_.pregrasp_target.right, end_right);
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2B] 终点 FK 残差（对本次选定预抓取目标）: 左=%.4f mm/%.4f° 右=%.4f mm/%.4f°",
                end_err_l.pos_m * 1000.0, end_err_l.angle_rad * 180.0 / M_PI,
                end_err_r.pos_m * 1000.0, end_err_r.angle_rad * 180.0 / M_PI);
    if (end_err_l.pos_m > ctx_->p2a_ik_pos_tol_m_ || end_err_r.pos_m > ctx_->p2a_ik_pos_tol_m_ ||
        end_err_l.angle_rad > ctx_->p2a_ik_angle_tol_deg_ * M_PI / 180.0 ||
        end_err_r.angle_rad > ctx_->p2a_ik_angle_tol_deg_ * M_PI / 180.0) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_ENDPOINT_MISMATCH";
        out.path_validation.detail =
            "the planned end does not reproduce the pre-grasp solution this round selected";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
        return false;
    }

    // Record the joint path itself, so the result is reproducible from the run
    // log rather than only summarised. The run script extracts these lines.
    {
        std::string name_line;
        for (const std::string& n : traj_names) {
            name_line += (name_line.empty() ? "" : ",") + n;
        }
        RCLCPP_INFO(ctx_->get_logger(), "[P2B_PATH_NAMES] %s", name_line.c_str());
        for (std::size_t pi = 0; pi < traj.points.size(); ++pi) {
            std::string row = std::to_string(pi);
            for (double v : traj.points[pi].positions) {
                row += "," + std::to_string(v);
            }
            RCLCPP_INFO(ctx_->get_logger(), "[P2B_PATH] %s", row.c_str());
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "[P2B_START_END] model=mujoco_hand_aabb start_source=live_current_state "
                    "goal=this_round_selected_pregrasp planner=%s waypoints=%zu joints=%zu",
                    ctx_->p2b_planner_id_.c_str(), traj.points.size(), traj_names.size());
    }

    // -----------------------------------------------------------------------
    // 4. Collision over every waypoint and every interpolated state, on the
    //    criterion model, with one fixed ACM set.
    // -----------------------------------------------------------------------
    std::size_t checked_states = 0;
    std::size_t subdivisions = 0;
    ContactTally path_tally_total;
    std::string worst_detail;

    // Every waypoint is checked explicitly, including the LAST one. The segment
    // interpolation below samples t in [0,1) of each segment and therefore never
    // lands on the final trajectory point, so this pass is what covers it -- and
    // it does so unconditionally, not via the negative-example switch.
    ContactTally waypoint_tally;
    for (std::size_t pi = 0; pi < traj.points.size(); ++pi) {
        const moveit::core::RobotState st = stateOf(traj.points[pi].positions);
        const ContactTally t = tallyContactsIn(scene, model, st, a1_verify_.base_acm, rod_id,
                                               support_id);
        ++checked_states;
        if (t.total() > 0) {
            waypoint_tally.rod += t.rod;
            waypoint_tally.support += t.support;
            waypoint_tally.dual_arm += t.dual_arm;
            waypoint_tally.self += t.self;
            waypoint_tally.world += t.world;
            if (worst_detail.empty()) {
                worst_detail = "waypoint " + std::to_string(pi) + " " + describeTally(t);
            }
        }
    }
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2B] 逐航点碰撞检查（含最后航点 %zu/%zu）: %zu 个航点，碰撞 %zu 处",
                traj.points.size(), traj.points.size(), traj.points.size(),
                waypoint_tally.total());

    const double step = ctx_->p2b_max_joint_step_rad_;
    if (!(step > 0.0)) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_BAD_CONFIG";
        out.path_validation.detail = "p2b_max_joint_step_rad must be positive";
        return false;
    }
    constexpr std::size_t kMaxSubdivisions = 2000;

    for (std::size_t pi = 0; pi + 1 < traj.points.size(); ++pi) {
        const auto& a = traj.points[pi].positions;
        const auto& b = traj.points[pi + 1].positions;
        const double seg_delta = a1v::maxAbsJointDelta(a, b);
        const std::size_t n = a1v::subdivisionsFor(seg_delta, step);
        if (n > kMaxSubdivisions) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_SEGMENT_TOO_LONG";
            out.path_validation.detail =
                "segment " + std::to_string(pi) + " needs " + std::to_string(n) +
                " subdivisions at the configured step; refusing to skip any of it";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
            return false;
        }
        subdivisions += n;
        for (std::size_t k = 0; k < n; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(n);
            const std::vector<double> pos = a1v::interpolateJoints(a, b, t);
            moveit::core::RobotState st = stateOf(pos);
            // The pre-grasp path must not put the hand or fingers through the
            // rod, so the strict ACM (fingers not exempted) is used throughout.
            const ContactTally tly = tallyContactsIn(scene, model, st, a1_verify_.base_acm, rod_id,
                                                     support_id);
            ++checked_states;
            if (tly.total() > 0) {
                path_tally_total.rod += tly.rod;
                path_tally_total.support += tly.support;
                path_tally_total.dual_arm += tly.dual_arm;
                path_tally_total.self += tly.self;
                path_tally_total.world += tly.world;
                if (worst_detail.empty()) {
                    worst_detail = "segment " + std::to_string(pi) + " t=" + std::to_string(t) +
                                   " " + describeTally(tly);
                }
            }
        }
    }

    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2B] 插值分辨率: 最大单关节步长 %.4f rad ⇒ 共细分 %zu 段, 检查 %zu 个状态"
                "（航点 %zu）。这是离散采样检查，**不是**连续碰撞的数学保证",
                step, subdivisions, checked_states, traj.points.size());
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P2B] 自碰撞口径: 沿用既有 SRDF/ACM 豁免，含同臂 hand↔link6(Never)/link7/link8"
                " 等继承项；跨臂允许项已在验证副本中关闭。结果是在该规则下的结果");

    path_tally_total.rod += waypoint_tally.rod;
    path_tally_total.support += waypoint_tally.support;
    path_tally_total.dual_arm += waypoint_tally.dual_arm;
    path_tally_total.self += waypoint_tally.self;
    path_tally_total.world += waypoint_tally.world;

    if (path_tally_total.total() > 0) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_PATH_COLLISION";
        out.path_validation.detail = "path collision: " + worst_detail;
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] 路径存在碰撞: %s",
                     describeTally(path_tally_total).c_str());
        return false;
    }

    // -----------------------------------------------------------------------
    // 5. Mid-path negative: an obstacle that blocks the MIDDLE of the path but
    //    not its endpoints. This proves the interior is really checked.
    // -----------------------------------------------------------------------
    if (ctx_->p2b_midpath_negative_ && traj.points.size() >= 3) {
        const std::string obstacle_id = "a1_path_obstacle_injected";
        auto placeObstacle = [&](const Eigen::Isometry3d& hand_world) {
            moveit_msgs::msg::CollisionObject obstacle;
            obstacle.header.frame_id = scene->getPlanningFrame();
            obstacle.id = obstacle_id;
            shape_msgs::msg::SolidPrimitive box;
            box.type = shape_msgs::msg::SolidPrimitive::BOX;
            box.dimensions = {0.05, 0.05, 0.05};
            obstacle.primitives.push_back(box);
            geometry_msgs::msg::Pose op;
            op.position.x = hand_world.translation().x();
            op.position.y = hand_world.translation().y();
            op.position.z = hand_world.translation().z();
            op.orientation.w = 1.0;
            obstacle.primitive_poses.push_back(op);
            obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
            scene->processCollisionObjectMsg(obstacle);
        };
        auto removeObstacle = [&]() {
            moveit_msgs::msg::CollisionObject obstacle;
            obstacle.header.frame_id = scene->getPlanningFrame();
            obstacle.id = obstacle_id;
            obstacle.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            scene->processCollisionObjectMsg(obstacle);
        };

        // The injection must be strictly INTERIOR: it has to block part of the
        // path while leaving BOTH endpoints clean. Placing it blindly at the
        // middle waypoint is not enough -- if the hand barely translates while
        // the wrist re-orients, the middle and the end overlap and the test
        // would not demonstrate interior coverage. Candidate positions are
        // therefore searched and validated before the real run.
        // A link that barely translates is a poor carrier for the obstacle: its
        // own collision volume then reaches both endpoints. Several links and
        // positions are therefore searched, and the obstacle is kept small.
        const char* carrier_links[4] = {"mj_left_link5", "mj_left_link6", "mj_left_link7",
                                        "mj_left_link8"};
        std::size_t chosen = 0;
        const char* chosen_link = nullptr;
        bool found = false;
        ContactTally chosen_mid;
        const std::size_t lo = traj.points.size() / 5;
        const std::size_t hi = (traj.points.size() * 4) / 5;
        std::string scan_log;
        for (const char* link : carrier_links) {
            for (std::size_t idx = lo; idx <= hi && !found; ++idx) {
                const moveit::core::RobotState cand = stateOf(traj.points[idx].positions);
                placeObstacle(cand.getGlobalLinkTransform(link));
                const ContactTally sc = tallyContactsIn(scene, model, *a1_verify_.start,
                                                        a1_verify_.base_acm, rod_id, support_id);
                const ContactTally ec = tallyContactsIn(scene, model, end_state,
                                                        a1_verify_.base_acm, rod_id, support_id);
                const ContactTally mc = tallyContactsIn(scene, model, cand, a1_verify_.base_acm,
                                                        rod_id, support_id);
                removeObstacle();
                if (sc.total() == 0 && ec.total() == 0 && mc.total() > 0) {
                    found = true;
                    chosen = idx;
                    chosen_link = link;
                    chosen_mid = mc;
                    scan_log += " [" + std::string(link) + "@" + std::to_string(idx) + ":起0/终0/中" +
                                std::to_string(mc.total()) + "]";
                }
            }
            if (found) {
                break;
            }
            scan_log += " [" + std::string(link) + ": 无严格内部候选]";
        }

        if (!found) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_MIDPATH_NEGATIVE_NOT_CONSTRUCTIBLE";
            out.path_validation.detail =
                "no strictly interior obstacle position was found on this path (candidates" +
                scan_log + "); interior coverage is NOT demonstrated, so this is reported as a "
                "failure rather than quietly dropped";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
            return false;
        }

        const moveit::core::RobotState mid_state = stateOf(traj.points[chosen].positions);
        placeObstacle(mid_state.getGlobalLinkTransform(chosen_link));

        // Re-run the full interpolation with the obstacle present.
        std::size_t blocked_states = 0;
        for (std::size_t pi = 0; pi + 1 < traj.points.size(); ++pi) {
            const auto& a = traj.points[pi].positions;
            const auto& b = traj.points[pi + 1].positions;
            const std::size_t n = a1v::subdivisionsFor(a1v::maxAbsJointDelta(a, b), step);
            for (std::size_t k = 0; k < n; ++k) {
                const double t = static_cast<double>(k) / static_cast<double>(n);
                moveit::core::RobotState st = stateOf(a1v::interpolateJoints(a, b, t));
                if (tallyContactsIn(scene, model, st, a1_verify_.base_acm, rod_id, support_id)
                        .total() > 0) {
                    ++blocked_states;
                }
            }
        }
        removeObstacle();

        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2B][中段负例-注入] 候选扫描: %s", scan_log.c_str());
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P2B][中段负例-注入] 选用航点 %zu/%zu 的 %s 位置注入 0.05 m 障碍 '%s'"
                    "（验证注入，非规划器结果、非实际运动；起终点经确认无碰撞）",
                    chosen + 1, traj.points.size(), chosen_link, obstacle_id.c_str());
        RCLCPP_INFO(ctx_->get_logger(), "     中间航点碰撞=%zu", chosen_mid.total());
        RCLCPP_INFO(ctx_->get_logger(), "     注入后整条路径上有 %zu 个采样状态被检出碰撞（原为 0）",
                    blocked_states);
        if (blocked_states == 0) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_MIDPATH_NEGATIVE_MISSED";
            out.path_validation.detail = "the interior obstacle produced no collision along the path";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P2B] %s", out.path_validation.detail.c_str());
            return false;
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  ✓ [P2B][中段负例-注入] 端点无碰撞、中段被检出 ⇒ 路径内部确实被检查");
    }

    a1_verify_.plan = plan;
    a1_verify_.plan_valid = true;
    out.path_validation.status = a1v::CheckStatus::PASSED;
    out.path_validation.reason_code = "E_OK";
    out.path_validation.detail =
        "planned path verified over " + std::to_string(checked_states) + " states";
    RCLCPP_INFO(ctx_->get_logger(),
                "  ✓ [P2B] 联合预抓取路径通过（仅规划，未执行）：%s", out.path_validation.detail.c_str());
    return true;
}

/**
 * A1 P3a-1: are the controllers in the state this round requires?
 *
 * dual_panda_arm_controller must be active (that is what receives the joint
 * trajectory) and sim_multi_mode_controller must NOT be, since the two must
 * never be active at once. No controller is switched here.
 */
bool DualArmController::checkControllerStatesEnabled(std::string& detail)
{
    using namespace std::chrono_literals;
    auto client = ctx_->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
    if (!client->wait_for_service(5s)) {
        detail = "E_P3A_CM_UNAVAILABLE: /controller_manager/list_controllers is unavailable";
        return false;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {
        detail = "E_P3A_CM_TIMEOUT: the controller list did not arrive within 5 s";
        return false;
    }
    const auto response = future.get();

    std::string summary;
    std::string arm_state;
    std::string multi_state;
    for (const auto& c : response->controller) {
        summary += " [" + c.name + "=" + c.state + "]";
        if (c.name == "dual_panda_arm_controller") {
            arm_state = c.state;
        }
        if (c.name == "sim_multi_mode_controller") {
            multi_state = c.state;
        }
    }
    RCLCPP_INFO(ctx_->get_logger(), "  [P3A-1] 控制器状态:%s", summary.c_str());

    if (arm_state != "active") {
        detail = "E_P3A_ARM_CONTROLLER_NOT_ACTIVE: dual_panda_arm_controller state is '" +
                 (arm_state.empty() ? std::string("<absent>") : arm_state) + "'";
        return false;
    }
    if (multi_state == "active") {
        detail = "E_P3A_CONTROLLER_CONFLICT: sim_multi_mode_controller is active; the two "
                 "controllers must not be active together";
        return false;
    }
    return true;
}

bool DualArmController::executePregrasp(const a1t::TruthPayload& truth, std::string& detail)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(ctx_->get_logger(),
                "\n----- [P3A-1] 执行本轮校验过的联合预抓取轨迹（到位即停，不下降/不抓取）-----");

    // ---- 0. Only a plan that P2b just verified may be executed. -------------
    if (!a1_verify_.valid || !a1_verify_.plan_valid) {
        detail = "E_P3A_NO_VERIFIED_PLAN: no plan from this round's P2b check is available";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    const auto& traj = a1_verify_.plan.trajectory_.joint_trajectory;
    if (traj.points.empty() || traj.joint_names.empty()) {
        detail = "E_P3A_EMPTY_PLAN: the verified plan carries no points";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-1] 执行对象 = 本轮 P2b 刚校验的同一 Plan（%zu 航点，%zu 关节）；"
                "不重新规划、不读取旧 CSV、不经 executeApproach",
                traj.points.size(), traj.joint_names.size());

    // ---- 1. Controllers ----------------------------------------------------
    if (!checkControllerStatesEnabled(detail)) {
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-1] dual_panda_arm_controller active 且 sim_multi_mode_controller 非 active；"
                "本轮不做控制器切换");

    // ---- 2. The robot must still be where the plan starts. -----------------
    {
        const moveit::core::RobotStatePtr cur = ctx_->dual_arm_->getCurrentState();
        double worst = 0.0;
        for (std::size_t i = 0; i < traj.joint_names.size(); ++i) {
            worst = std::max(worst, std::fabs(cur->getVariablePosition(traj.joint_names[i]) -
                                              traj.points.front().positions[i]));
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-1] 当前关节状态与计划首点最大差 %.6f rad（阈值 0.020）", worst);
        if (worst > 0.02) {
            detail = "E_P3A_START_MISMATCH: the robot is no longer at the plan's first point "
                     "(max delta " + std::to_string(worst) + " rad)";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
            return false;
        }
    }

    // ---- 3. The frozen object/scene must still apply. ----------------------
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        const auto objects = psi.getObjects(std::vector<std::string>{"aluminum_rod"});
        const auto it = objects.find("aluminum_rod");
        if (it == objects.end()) {
            detail = "E_P3A_ROD_MISSING: aluminum_rod is no longer in the planning scene";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
            return false;
        }
        tf2::Transform object_tf;
        tf2::Transform primitive_tf;
        tf2::fromMsg(it->second.pose, object_tf);
        tf2::fromMsg(it->second.primitive_poses[0], primitive_tf);
        geometry_msgs::msg::Pose actual;
        tf2::toMsg(object_tf * primitive_tf, actual);
        const a1t::Vec3 apos{actual.position.x, actual.position.y, actual.position.z};
        const a1t::Quat aquat{actual.orientation.x, actual.orientation.y, actual.orientation.z,
                              actual.orientation.w};
        const double dpos = std::sqrt(std::pow(apos.x - truth.position.x, 2) +
                                      std::pow(apos.y - truth.position.y, 2) +
                                      std::pow(apos.z - truth.position.z, 2));
        const double dang = a1t::rotationDistanceRad(
            a1t::Quat{truth.orientation.x, truth.orientation.y, truth.orientation.z,
                      truth.orientation.w}, aquat);
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-1] 冻结对象核对: 场景中的杆与本轮冻结真值差 %.4f mm / %.4f°",
                    dpos * 1000.0, dang * 180.0 / M_PI);
        if (dpos > 0.005 || dang > 5.0 * M_PI / 180.0) {
            detail = "E_P3A_SCENE_CHANGED: the rod in the planning scene no longer matches the "
                     "frozen truth, so the verified plan no longer applies";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
            return false;
        }
    }

    // ---- 4a. Validate the derivative fields of the trajectory we are about to
    //          dispatch. The 0.1 planning scaling is a configuration and does not
    //          by itself prove the dispatched fields are valid.
    {
        const moveit::core::JointModelGroup* dual_jmg =
            a1_verify_.model->getJointModelGroup(ctx_->dual_arm_group_);
        std::vector<double> vel_limits;
        std::vector<bool> vel_bounded;
        std::vector<double> acc_limits;
        std::vector<bool> acc_bounded;
        for (const std::string& j : traj.joint_names) {
            double vmax = 0.0;
            double amax = 0.0;
            bool vb = false;
            bool ab = false;
            // Limits are read from the LIVE model, which is the one that loaded
            // robot_description_planning (joint_limits.yaml) and therefore
            // carries the configured acceleration limits. The independent
            // criterion model is built from URDF+SRDF only and has no
            // acceleration bounds -- using it here would silently reduce this
            // check to nothing. Joint limits are identical in both models; only
            // the hand collision geometry differs. Nothing is invented: a joint
            // with no declared limit is counted as unbounded, never as zero.
            const moveit::core::JointModel* jm = ctx_->dual_arm_->getRobotModel()->getJointModel(j);
            if (jm != nullptr) {
                const moveit::core::VariableBounds& b = jm->getVariableBounds(j);
                vmax = std::max(std::fabs(b.min_velocity_), std::fabs(b.max_velocity_));
                vb = b.velocity_bounded_;
                amax = std::max(std::fabs(b.min_acceleration_), std::fabs(b.max_acceleration_));
                ab = b.acceleration_bounded_;
            }
            vel_limits.push_back(vmax);
            vel_bounded.push_back(vb);
            acc_limits.push_back(amax);
            acc_bounded.push_back(ab);
        }
        (void)dual_jmg;

        std::size_t vel_points = 0;
        std::size_t acc_points = 0;
        std::size_t vel_unbounded = 0;
        std::size_t acc_unbounded = 0;
        double vel_worst = 0.0;
        double acc_worst = 0.0;
        std::string vel_worst_joint;
        std::string acc_worst_joint;
        for (std::size_t pi = 0; pi < traj.points.size(); ++pi) {
            const auto& pt = traj.points[pi];
            if (!pt.velocities.empty()) {
                ++vel_points;
                const a1v::DerivativeCheck c =
                    a1v::checkDerivativeField(pt.velocities, vel_limits, vel_bounded,
                                              traj.joint_names);
                if (!c.ok) {
                    detail = "E_TRAJ_VELOCITY_INVALID at point " + std::to_string(pi) + ": " +
                             c.reason_code + " (" + c.detail + ")";
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-1] %s", detail.c_str());
                    return false;
                }
                vel_unbounded += c.unbounded;
                if (c.worst_ratio > vel_worst) {
                    vel_worst = c.worst_ratio;
                    vel_worst_joint = c.worst_joint;
                }
            }
            if (!pt.accelerations.empty()) {
                ++acc_points;
                const a1v::DerivativeCheck c =
                    a1v::checkDerivativeField(pt.accelerations, acc_limits, acc_bounded,
                                              traj.joint_names);
                if (!c.ok) {
                    detail = "E_TRAJ_ACCELERATION_INVALID at point " + std::to_string(pi) + ": " +
                             c.reason_code + " (" + c.detail + ")";
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-1] %s", detail.c_str());
                    return false;
                }
                acc_unbounded += c.unbounded;
                if (c.worst_ratio > acc_worst) {
                    acc_worst = c.worst_ratio;
                    acc_worst_joint = c.worst_joint;
                }
            }
        }

        const std::size_t npts = traj.points.size();
        if (vel_points != 0 && vel_points != npts) {
            detail = "E_TRAJ_VELOCITY_INCONSISTENT: " + std::to_string(vel_points) + " of " +
                     std::to_string(npts) + " points carry velocities";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-1] %s", detail.c_str());
            return false;
        }
        if (acc_points != 0 && acc_points != npts) {
            detail = "E_TRAJ_ACCELERATION_INCONSISTENT: " + std::to_string(acc_points) + " of " +
                     std::to_string(npts) + " points carry accelerations";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-1] %s", detail.c_str());
            return false;
        }

        if (vel_points == 0) {
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [P3A-1] 下发轨迹**不含速度字段**，因此速度限制**未被校验**；"
                        "不据此声称速度达标（不把缺失当零）");
        } else {
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P3A-1] 速度字段校验（限制来源 live 模型）: %zu/%zu 点，最大占用比 %.4f"
                        "（关节 %s），未声明限制的关节计数 %zu",
                        vel_points, npts, vel_worst,
                        vel_worst_joint.empty() ? "-" : vel_worst_joint.c_str(), vel_unbounded);
        }
        if (acc_points == 0) {
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [P3A-1] 下发轨迹**不含加速度字段**，因此加速度限制**未被校验**；"
                        "不据此声称加速度达标（不把缺失当零）");
        } else {
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P3A-1] 加速度字段校验（限制来源 live 模型）: %zu/%zu 点，最大占用比 %.4f"
                        "（关节 %s），未声明限制的关节计数 %zu",
                        acc_points, npts, acc_worst,
                        acc_worst_joint.empty() ? "-" : acc_worst_joint.c_str(), acc_unbounded);
        }
    }

    // ---- 4b. Dispatch the verified plan with a bounded wait.
    //
    // The helper owns the worker and its result slot as shared state, so nothing
    // on this stack is referenced and a worker that outlives the wait cannot
    // touch freed memory. The MoveGroupInterface is likewise kept alive by a
    // shared_ptr for the duration of the call. A MoveIt async action would be
    // preferable, but MoveGroupInterface exposes no goal handle for the
    // trajectory execution it starts internally.
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-1] 下发轨迹: 速度缩放 %.3f / 加速度缩放 %.3f；本轮不发送任何夹爪动作、"
                "不开 weld、不 attach",
                ctx_->pregrasp_vel_scale_, ctx_->pregrasp_acc_scale_);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> iface = ctx_->dual_arm_;
    moveit::planning_interface::MoveGroupInterface::Plan dispatched = a1_verify_.plan;
    const double planned_sec =
        traj.points.empty() ? 0.0 : rclcpp::Duration(traj.points.back().time_from_start).seconds();

    const a1v::BoundedExecOutcome exec = a1v::runBoundedExec(
        [iface, dispatched]() mutable { return iface->execute(dispatched).val; },
        [iface]() { iface->stop(); }, ctx_->pregrasp_exec_timeout_sec_, 4.0, 0.05);

    if (exec.status == a1v::BoundedExecOutcome::Status::TIMEOUT) {
        // This round terminates as a FAILURE. A late result is never read back
        // into the verdict, so a late success cannot become PREGRASP_REACHED,
        // and nothing is retried or continued.
        RCLCPP_ERROR(ctx_->get_logger(),
                     "  ✗ [P3A-1] E_P3A_EXEC_TIMEOUT: 执行在 %.1f s 内未结束（计划时长 %.3f s）",
                     exec.elapsed_sec, planned_sec);
        // A late result that had already ended needs no stop request; only a
        // still-running worker gets one. Either way the status stays TIMEOUT and
        // the late value is never read into the verdict.
        RCLCPP_WARN(ctx_->get_logger(),
                    "  [P3A-1] worker 实际完成时刻 %.4f s，预算 %.4f s ⇒ 判定为迟到；%s",
                    exec.completed_at_sec >= 0.0 ? exec.completed_at_sec : -1.0, exec.timeout_sec,
                    exec.stop_requested ? "已发出停止请求（**不声称**已保证物理停止）"
                                        : "worker 已自行结束，未发出停止请求");
        if (exec.worker_finished) {
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [P3A-1] 迟到结果被丢弃，不参与判定；本轮以失败终止，不重试、不继续");
        } else {
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [P3A-1] worker 在停止宽限期内仍未结束（已 detach；其捕获均为共享状态，"
                        "不会访问已销毁对象）");
        }
        detail = "E_P3A_EXEC_TIMEOUT: execution did not finish within " +
                 std::to_string(ctx_->pregrasp_exec_timeout_sec_) + " s";
        return false;
    }

    if (exec.status == a1v::BoundedExecOutcome::Status::EXCEPTION) {
        detail = "E_P3A_EXEC_EXCEPTION: the execution call raised an exception";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-1] %s", detail.c_str());
        return false;
    }

    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-1] 执行返回码 %d（MoveIt SUCCESS 枚举，非进程退出码），"
                "用时 %.3f s（计划时长 %.3f s）",
                exec.value, exec.elapsed_sec, planned_sec);
    if (exec.value != moveit::core::MoveItErrorCode::SUCCESS) {
        detail = "E_P3A_EXEC_FAILED: the controller reported error code " +
                 std::to_string(exec.value);
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    if (exec.elapsed_sec > std::max(3.0 * planned_sec, 5.0)) {
        detail = "E_P3A_EXEC_OVERRUN: execution took " + std::to_string(exec.elapsed_sec) +
                 " s for a " + std::to_string(planned_sec) + " s plan";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }

    // ---- 5. Arrival: fresh, settled state versus this round's target. ------
    {
        const auto w0 = std::chrono::steady_clock::now();
        double settled_delta = 0.0;
        std::size_t samples = 0;
        moveit::core::RobotStatePtr prev = ctx_->dual_arm_->getCurrentState();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count() <
               ctx_->pregrasp_stable_window_sec_) {
            std::this_thread::sleep_for(50ms);
            const moveit::core::RobotStatePtr now = ctx_->dual_arm_->getCurrentState();
            double worst = 0.0;
            for (const std::string& j : traj.joint_names) {
                worst = std::max(worst, std::fabs(now->getVariablePosition(j) -
                                                  prev->getVariablePosition(j)));
            }
            settled_delta = std::max(settled_delta, worst);
            prev = now;
            ++samples;
        }
        // The state is read live over a window; a settled reading cannot come
        // from a stale cache.
        const moveit::core::RobotStatePtr arrived = ctx_->dual_arm_->getCurrentState();
        const a1v::ArmTarget got_left =
            fromEigenPose(arrived->getGlobalLinkTransform(a1_verify_.left_eef));
        const a1v::ArmTarget got_right =
            fromEigenPose(arrived->getGlobalLinkTransform(a1_verify_.right_eef));
        const a1v::PoseError el = a1v::poseError(a1_verify_.pregrasp_target.left, got_left);
        const a1v::PoseError er = a1v::poseError(a1_verify_.pregrasp_target.right, got_right);

        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-1] 到位实测（%zu 次采样，窗口 %.2f s，窗口内最大关节变化 %.6f rad）:",
                    samples, ctx_->pregrasp_stable_window_sec_, settled_delta);
        RCLCPP_INFO(ctx_->get_logger(),
                    "     左臂 位置误差 %.4f mm，姿态误差 %.4f°  |  右臂 位置误差 %.4f mm，"
                    "姿态误差 %.4f°",
                    el.pos_m * 1000.0, el.angle_rad * 180.0 / M_PI, er.pos_m * 1000.0,
                    er.angle_rad * 180.0 / M_PI);
        RCLCPP_INFO(ctx_->get_logger(), "     到位容差: 位置 %.1f mm / 姿态 %.2f°",
                    ctx_->pregrasp_arrival_pos_tol_m_ * 1000.0,
                    ctx_->pregrasp_arrival_angle_tol_deg_);

        if (settled_delta > ctx_->pregrasp_stable_tol_rad_) {
            detail = "E_P3A_NOT_SETTLED: the joints were still moving at the end of the window "
                     "(max change " + std::to_string(settled_delta) + " rad)";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
            return false;
        }
        const double ang_tol = ctx_->pregrasp_arrival_angle_tol_deg_ * M_PI / 180.0;
        if (el.pos_m > ctx_->pregrasp_arrival_pos_tol_m_ ||
            er.pos_m > ctx_->pregrasp_arrival_pos_tol_m_ || el.angle_rad > ang_tol ||
            er.angle_rad > ang_tol) {
            detail = "E_P3A_NOT_AT_TARGET: left " + std::to_string(el.pos_m * 1000.0) +
                     " mm/" + std::to_string(el.angle_rad * 180.0 / M_PI) + " deg, right " +
                     std::to_string(er.pos_m * 1000.0) + " mm/" +
                     std::to_string(er.angle_rad * 180.0 / M_PI) + " deg";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
            return false;
        }

        // Observation only: the rod must not have been disturbed.
        moveit::planning_interface::PlanningSceneInterface psi;
        const auto objects = psi.getObjects(std::vector<std::string>{"aluminum_rod"});
        const auto it = objects.find("aluminum_rod");
        if (it != objects.end() && !it->second.primitive_poses.empty()) {
            tf2::Transform object_tf;
            tf2::Transform primitive_tf;
            tf2::fromMsg(it->second.pose, object_tf);
            tf2::fromMsg(it->second.primitive_poses[0], primitive_tf);
            geometry_msgs::msg::Pose actual;
            tf2::toMsg(object_tf * primitive_tf, actual);
            const double d = std::sqrt(std::pow(actual.position.x - truth.position.x, 2) +
                                       std::pow(actual.position.y - truth.position.y, 2) +
                                       std::pow(actual.position.z - truth.position.z, 2));
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P3A-1] 观察（非判据）: 执行后**规划场景中**的杆对象相对冻结真值偏移 "
                        "%.4f mm —— 来源是 /get_planning_scene + PlanningSceneInterface，"
                        "**不是** MuJoCo 物理真值；物理扰动本轮未测。预抓取到位不代表抓取成功",
                        d * 1000.0);
        }
    }

    detail = "PREGRASP_REACHED";
    return true;
}

void DualArmController::executeApproach()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: APPROACH] 接近物体...");
        
        // ========== 物体与机械臂参数定义 ==========
        // 物体位置 (从视觉检测获取，或回退到 objects.xml 硬编码)
        geometry_msgs::msg::Pose detected_pose;
        const bool has_detected_pose = ctx_->getDetectedRodPose(detected_pose);
        double obj_x = has_detected_pose ? detected_pose.position.x : 0.50;
        double obj_y = has_detected_pose ? detected_pose.position.y : 0.0;
        
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
            if (!ctx_->grippers_->controlDualGrippers(0.04)) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 预打开夹爪失败，停止任务");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
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
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            
            // 使用双臂组规划到IK解
            ctx_->dual_arm_->setJointValueTarget(*target_state);
            ctx_->dual_arm_->setPlannerId("PTP");
            
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
            
            // 双末端组在 LIN 下稳定性较差：改用 IK 求目标关节 + Pilz PTP 执行同步下降
            const moveit::core::JointModelGroup* left_jmg = current_state->getJointModelGroup(ctx_->left_arm_group_);
            const moveit::core::JointModelGroup* right_jmg = current_state->getJointModelGroup(ctx_->right_arm_group_);

            bool left_ik = current_state->setFromIK(left_jmg, left_target, 0.05);
            bool right_ik = current_state->setFromIK(right_jmg, right_target, 0.05);
            if (!left_ik || !right_ik) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 下降阶段IK无解! Left=%s, Right=%s", left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            ctx_->dual_arm_->clearPoseTargets();
            // Ensure the planned trajectory starts from the latest joint state,
            // otherwise execution may abort due to start-state tolerance checks.
            ctx_->dual_arm_->setStartStateToCurrentState();
            ctx_->dual_arm_->setJointValueTarget(*current_state);
            ctx_->dual_arm_->setPlannerId("PTP");
            
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

            const auto grounded_left_pose = ctx_->left_arm_->getCurrentPose();
            const auto grounded_right_pose = ctx_->right_arm_->getCurrentPose();
            ctx_->place_release_hand_z_ref_ =
                0.5 * (grounded_left_pose.pose.position.z + grounded_right_pose.pose.position.z);
            ctx_->place_release_hand_z_ref_valid_ = true;
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [GRASP] 已记录放置接地参考高度: hand_z=%.3f",
                        ctx_->place_release_hand_z_ref_);
            
            // ========== Step 2: 碰撞免除同步 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [GRASP] Step 2: 更新世界ACM允许夹取碰撞");
            ctx_->arms_->allow_gripper_collision(true);
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ ACM 更新完毕");
            
            // ========== Step 3: 闭合夹爪并附着物体 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [GRASP] Step 3: Closing grippers and attaching object");
            
            // 闭合夹爪
            if (!ctx_->grippers_->controlDualGrippers(ctx_->gripper_close_pos_)) {   // 同步闭合
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 夹爪闭合失败，禁止激活 weld 和附着物体");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ Grippers closed");
            
            // 关键：非阻塞式等待MuJoCo物理引擎完全稳定
            
            // 激活MuJoCo weld约束 —— 将铝棒刚性绑定到左手
            if (ctx_->weld_pub_) {
                auto weld_msg = std_msgs::msg::Bool();
                weld_msg.data = true;
                ctx_->weld_pub_->publish(weld_msg);
                RCLCPP_INFO(ctx_->get_logger(), "  [MuJoCo] Weld约束已激活（铝棒刚性绑定到双手）");
            }
            
            // 附着物体到机器人。附着失败时必须回滚 weld，避免仿真已抓住但
            // PlanningScene 未附着，后续规划基于错误物体状态继续执行。
            if (!ctx_->arms_->attachObject()) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ Object attach failed, rolling back weld and aborting");
                if (ctx_->weld_pub_) {
                    auto weld_msg = std_msgs::msg::Bool();
                    weld_msg.data = false;
                    ctx_->weld_pub_->publish(weld_msg);
                }
                ctx_->setCurrentState(TaskState::ERROR);
                return;
            }
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ Object attached to robot");
            
            RCLCPP_INFO(ctx_->get_logger(), "  [等待] 异步等待物理引擎稳定中（1.5秒）...");
            ctx_->current_state_ = TaskState::WAITING_PHYSICS;
            
            // 创建一次性定时器，不阻塞当前执行器
            ctx_->wait_timer_ = ctx_->create_wall_timer(
                std::chrono::milliseconds(1500),
                [this]() {
                    if (ctx_->getCurrentState() != TaskState::WAITING_PHYSICS) {
                        if (ctx_->wait_timer_) {
                            ctx_->wait_timer_->cancel();
                        }
                        return;
                    }
                    RCLCPP_INFO(ctx_->get_logger(), "  [Timer Triggered] ✓ Physics stabilized, entering LIFT");
                    ctx_->setCurrentState(TaskState::LIFT);
                    if (ctx_->wait_timer_) {
                        ctx_->wait_timer_->cancel();
                    }
                });
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "GRASP 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

bool DualArmController::attachObject()
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
            // 回退：使用视觉检测位姿或已知初始位置
            RCLCPP_WARN(ctx_->get_logger(), "    ! 无法从PlanningScene获取物体，使用回退位姿");
            geometry_msgs::msg::Pose detected_pose;
            if (ctx_->getDetectedRodPose(detected_pose)) {
                object_world_pose = detected_pose;
            } else {
                object_world_pose.position.x = 0.50;
                object_world_pose.position.y = 0.0;
                object_world_pose.position.z = 0.02;
                object_world_pose.orientation.w = 1.0;
                object_world_pose.orientation.x = 0.0;
                object_world_pose.orientation.y = 0.0;
                object_world_pose.orientation.z = 0.0;
            }
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
        
        const bool world_has_rod = objects.count("aluminum_rod") > 0;
        const bool attached_has_rod = attached_objects.count("aluminum_rod") > 0;

        if (world_has_rod) {
            RCLCPP_WARN(ctx_->get_logger(), "    ! aluminum_rod 仍作为独立物体存在");
        } else {
            RCLCPP_INFO(ctx_->get_logger(), "    ✓ 独立物体已移除");
        }
        
        if (attached_has_rod) {
            RCLCPP_INFO(ctx_->get_logger(), "    ✓ 附着物体已确认");
        } else {
            RCLCPP_ERROR(ctx_->get_logger(), "    ✗ 附着物体验证失败：未找到 aluminum_rod");
            return false;
        }

        if (world_has_rod) {
            RCLCPP_ERROR(ctx_->get_logger(), "    ✗ 附着状态不一致：aluminum_rod 同时存在于世界物体中");
            return false;
        }
        
        RCLCPP_INFO(ctx_->get_logger(), "  ✓ 附着流程完成");
        RCLCPP_INFO(ctx_->get_logger(), "    原理: 从PlanningScene获取物体真实位姿 + base_link全局坐标系");
        RCLCPP_INFO(ctx_->get_logger(), "    期望: 物体在地面原位附着（Z≈0.02），LIFT时随手臂抬升");
        return true;
    }

void DualArmController::executeLift()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: LIFT] 抬起物体...");
        try {
            ctx_->dual_arm_->setMaxVelocityScalingFactor(0.5);
            ctx_->dual_arm_->setMaxAccelerationScalingFactor(0.5);

            ctx_->dual_arm_->setStartStateToCurrentState();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            auto left_current = ctx_->left_arm_->getCurrentPose();
            auto right_current = ctx_->right_arm_->getCurrentPose();
            
            geometry_msgs::msg::Pose left_lift = left_current.pose;
            left_lift.position.z = ctx_->lift_height_;
            
            geometry_msgs::msg::Pose right_lift = right_current.pose;
            right_lift.position.z = ctx_->lift_height_;

            auto target_state = ctx_->dual_arm_->getCurrentState();
            const moveit::core::JointModelGroup* left_jmg = target_state->getJointModelGroup(ctx_->left_arm_group_);
            const moveit::core::JointModelGroup* right_jmg = target_state->getJointModelGroup(ctx_->right_arm_group_);
            bool left_ik = target_state->setFromIK(left_jmg, left_lift, 0.05);
            bool right_ik = target_state->setFromIK(right_jmg, right_lift, 0.05);
            if (!left_ik || !right_ik) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [LIFT] IK 失败 Left=%s Right=%s", left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            ctx_->dual_arm_->clearPoseTargets();
            ctx_->dual_arm_->setJointValueTarget(*target_state);
            ctx_->dual_arm_->setPlannerId("PTP");
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (ctx_->dual_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                if (ctx_->dual_arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ [LIFT] Pilz PTP 抬升成功");
                    ctx_->current_state_ = TaskState::TRANSPORT;
                    return;
                }
            }
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [LIFT] Pilz 规划或执行失败");
            ctx_->current_state_ = TaskState::ERROR;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "LIFT 异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
}

void DualArmController::executeTransport()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: TRANSPORT] 平移物体...");
        try {
            const auto mode_profile = getMotionModeProfile(ctx_->control_mode_);

            auto left_start_pose = ctx_->left_arm_->getCurrentPose().pose;
            auto right_start_pose = ctx_->right_arm_->getCurrentPose().pose;
            const double transport_z = 0.5 * (left_start_pose.position.z + right_start_pose.position.z);
            const bool near_ground_transport = (transport_z <= ctx_->grasp_height_ + 0.10);

            // 低高度时增加航点密度并降低速度，加速度，避免“一卡一卡”的段间停顿。
            const int base_waypoints = near_ground_transport ? 28 : 16;
            const int num_waypoints = scaledWaypointCount(
                base_waypoints, mode_profile.transport_waypoint_scale, 12);
            const double vel_scale = clampScale(
                (near_ground_transport ? 0.20 : 0.35) * mode_profile.transport_vel_scale,
                0.05, 0.40);
            const double acc_scale = clampScale(
                (near_ground_transport ? 0.14 : 0.28) * mode_profile.transport_acc_scale,
                0.04, 0.35);
            const double ik_timeout = near_ground_transport ? 0.15 : 0.10;

            // 旋转场景下若平移过远，右臂在ROTATE中段（约20°附近）更容易出现IK无解。
            // 这里对启用旋转的工况使用更保守的平移终点，优先保证旋转可达性与稳定性。
            double total_dx, total_dy;
            geometry_msgs::msg::Pose detected_pose;
            if (ctx_->getDetectedRodPose(detected_pose)) {
                // Mode B: compute deltas from detected pose to target pose
                total_dx = ctx_->rod_target_x_ - detected_pose.position.x;
                total_dy = ctx_->rod_target_y_ - detected_pose.position.y;
                total_dx = std::clamp(total_dx, -0.30, 0.30);
                total_dy = std::clamp(total_dy, -0.25, 0.25);
                RCLCPP_INFO(ctx_->get_logger(),
                            "  [视觉模式] 动态目标偏移: dx=%.3f dy=%.3f (目标→检测)", total_dx, total_dy);
            } else {
                total_dx = ctx_->enable_rotate_ ? 0.12 : 0.20;
                total_dy = ctx_->enable_rotate_ ? 0.06 : 0.10;
            }

            RCLCPP_INFO(ctx_->get_logger(),
                        "  [TRANSPORT] 模式=%s, z=%.3f, near_ground=%s, waypoints=%d, vel=%.2f, acc=%.2f, dx=%.3f, dy=%.3f",
                        ctx_->control_mode_.c_str(),
                        transport_z, near_ground_transport ? "true" : "false", num_waypoints,
                        vel_scale, acc_scale, total_dx, total_dy);

            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto current_state = ctx_->dual_arm_->getCurrentState();
            auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
            traj->addSuffixWayPoint(*current_state, 0.0);

            const moveit::core::JointModelGroup* left_jmg = current_state->getJointModelGroup(ctx_->left_arm_group_);
            const moveit::core::JointModelGroup* right_jmg = current_state->getJointModelGroup(ctx_->right_arm_group_);
            auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);

            auto smoothstep5 = [](double s) {
                return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
            };

            for (int i = 1; i <= num_waypoints; ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(num_waypoints);
                const double s = smoothstep5(t);

                geometry_msgs::msg::Pose left_target = left_start_pose;
                geometry_msgs::msg::Pose right_target = right_start_pose;
                left_target.position.x += total_dx * s;
                left_target.position.y += total_dy * s;
                right_target.position.x += total_dx * s;
                right_target.position.y += total_dy * s;

                auto waypoint_state = std::make_shared<moveit::core::RobotState>(*seed_state);
                const auto ik_result = solveCoordinatedIk(
                    *seed_state, *waypoint_state, left_jmg, right_jmg,
                    left_target, right_target, ik_timeout);

                if (!ik_result.success) {
                    RCLCPP_ERROR(ctx_->get_logger(),
                                 "  ✗ [TRANSPORT] 连续轨迹航点 %d/%d IK失败 Left=%s Right=%s",
                                 i, num_waypoints,
                                 ik_result.left_success ? "OK" : "FAIL",
                                 ik_result.right_success ? "OK" : "FAIL");
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }

                waypoint_state->update();
                traj->addSuffixWayPoint(*waypoint_state, 0.0);
                *seed_state = *waypoint_state;
            }

            if (!parameterizeAndSmoothTrajectory(*traj, vel_scale, acc_scale,
                                                 ctx_->get_logger(), "TRANSPORT")) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            if (!validateManualTrajectory(*traj, robot_model, ctx_->dual_arm_group_,
                                          ctx_->get_logger(), "TRANSPORT")) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            traj->getRobotTrajectoryMsg(plan.trajectory_);
            plan.planning_time_ = 0.0;

            if (ctx_->dual_arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [TRANSPORT] 连续轨迹执行失败");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            RCLCPP_INFO(ctx_->get_logger(), "  ✓ [TRANSPORT] 连续平移执行成功");
            ctx_->current_state_ = ctx_->enable_rotate_ ? TaskState::ROTATE : TaskState::DESCEND;
            return;
        } catch (const std::exception& e) {
            ctx_->current_state_ = TaskState::ERROR;
        }
}

void DualArmController::executeRotate()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: ROTATE] 旋转物体方向（Y轴→45°）...");
        RCLCPP_INFO(ctx_->get_logger(), "  策略: 单段连续轨迹 + 密集航点 + TOTG（消除段间停顿）");
        RCLCPP_INFO(ctx_->get_logger(), "  [原理] 单段轨迹: TOTG生成全局平滑速度剖面，无中间归零停顿");
        RCLCPP_INFO(ctx_->get_logger(), "  [原理] 不刷新夹爪: simGripperGrasp保持at(0)=0(最大夹持力)，段间刷新会触发simGripperMove导致松手");
        RCLCPP_INFO(ctx_->get_logger(), "  [原理] Joint7偏置: 引导手腕关节主动承担Z轴旋转，避免中段/根部关节漂移翻转");
        
        try {
            const auto mode_profile = getMotionModeProfile(ctx_->control_mode_);

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
            
            // ===== Phase 2: 单段连续航点参数（消除段间停顿） =====
            const int num_segments = 1;
            const int wp_per_seg = scaledWaypointCount(45, mode_profile.rotate_waypoint_scale, 36);
            const int total_wp = num_segments * wp_per_seg;
            double total_angle;
            geometry_msgs::msg::Pose detected_pose;
            if (ctx_->getDetectedRodPose(detected_pose)) {
                double init_yaw = extractYawFromQuaternion(detected_pose.orientation);
                total_angle = ctx_->rod_target_yaw_rad_ - init_yaw;
                // Normalize to [-PI, PI]
                while (total_angle > M_PI) total_angle -= 2.0 * M_PI;
                while (total_angle < -M_PI) total_angle += 2.0 * M_PI;
                RCLCPP_INFO(ctx_->get_logger(),
                            "  [视觉模式] 动态旋转: init_yaw=%.1f° target_yaw=%.1f° angle=%.1f°",
                            init_yaw * 180.0 / M_PI, ctx_->rod_target_yaw_rad_ * 180.0 / M_PI,
                            total_angle * 180.0 / M_PI);
            } else {
                total_angle = M_PI / 4.0;
            }
            const double step_angle_deg = std::abs(total_angle) * 180.0 / M_PI / total_wp;  // 每步角度(度)

            RCLCPP_INFO(ctx_->get_logger(),
                        "  [ROTATE] 模式=%s, 航点密度缩放=%.2f, 速度缩放=%.2f, 加速度缩放=%.2f",
                        ctx_->control_mode_.c_str(),
                        mode_profile.rotate_waypoint_scale,
                        mode_profile.rotate_vel_scale,
                        mode_profile.rotate_acc_scale);
            
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
                double vel_scale = clampScale(0.06 * mode_profile.rotate_vel_scale, 0.025, 0.10);
                double acc_scale = clampScale(0.06 * mode_profile.rotate_acc_scale, 0.025, 0.10);
                bool time_ok = totg.computeTimeStamps(*traj, vel_scale, acc_scale);
                RCLCPP_INFO(ctx_->get_logger(), "    TOTG缩放: vel=%.4f, acc=%.4f", vel_scale, acc_scale);
                
                if (!time_ok) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 分段 %d TOTG失败", seg + 1);
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }
                
                RCLCPP_INFO(ctx_->get_logger(), "    TOTG: 时长=%.2fs, 轨迹点=%zu",
                            traj->getDuration(), traj->getWayPointCount());

                if (!validateManualTrajectory(*traj, robot_model, ctx_->dual_arm_group_,
                                              ctx_->get_logger(), "ROTATE")) {
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }
                
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
        RCLCPP_INFO(ctx_->get_logger(), "[状态: DESCEND] 下降准 备放置...");
        try {
            const auto mode_profile = getMotionModeProfile(ctx_->control_mode_);

            ctx_->dual_arm_->setStartStateToCurrentState();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            auto left_current = ctx_->left_arm_->getCurrentPose();
            auto right_current = ctx_->right_arm_->getCurrentPose();

            const double place_z = ctx_->place_release_hand_z_ref_valid_
                                       ? ctx_->place_release_hand_z_ref_
                                       : ctx_->grasp_height_;
            double descend_shift_x = ctx_->enable_rotate_ ? -0.10 : 0.0;

            geometry_msgs::msg::Pose left_start = left_current.pose;
            geometry_msgs::msg::Pose right_start = right_current.pose;
            geometry_msgs::msg::Pose left_goal = left_start;
            geometry_msgs::msg::Pose right_goal = right_start;
            left_goal.position.x += descend_shift_x;
            right_goal.position.x += descend_shift_x;
            left_goal.position.z = place_z;
            right_goal.position.z = place_z;

            const double descend_dist = std::max(std::abs(left_start.position.z - place_z),
                                                 std::abs(right_start.position.z - place_z));
            const bool near_ground = (place_z <= ctx_->grasp_height_ + 0.02);
            const int base_waypoints = near_ground ? 30 : 22;
            const int num_waypoints = scaledWaypointCount(
                base_waypoints, mode_profile.descend_waypoint_scale, 14);
            const double vel_scale = clampScale(
                (near_ground ? 0.10 : 0.16) * mode_profile.descend_vel_scale,
                0.04, 0.22);
            const double acc_scale = clampScale(
                (near_ground ? 0.06 : 0.10) * mode_profile.descend_acc_scale,
                0.03, 0.16);
            const double ik_timeout = near_ground ? 0.16 : 0.12;

            RCLCPP_INFO(ctx_->get_logger(),
                        "  [DESCEND] 模式=%s, dz=%.3f, place_z=%.3f, near_ground=%s, waypoints=%d, vel=%.2f, acc=%.2f",
                        ctx_->control_mode_.c_str(),
                        descend_dist, place_z, near_ground ? "true" : "false", num_waypoints, vel_scale, acc_scale);

            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto current_state = ctx_->dual_arm_->getCurrentState();
            auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
            traj->addSuffixWayPoint(*current_state, 0.0);

            const moveit::core::JointModelGroup* left_jmg = current_state->getJointModelGroup(ctx_->left_arm_group_);
            const moveit::core::JointModelGroup* right_jmg = current_state->getJointModelGroup(ctx_->right_arm_group_);
            auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);

            auto smoothstep5 = [](double s) {
                return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
            };

            for (int i = 1; i <= num_waypoints; ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(num_waypoints);
                const double s = smoothstep5(t);

                geometry_msgs::msg::Pose left_target = left_start;
                geometry_msgs::msg::Pose right_target = right_start;
                left_target.position.x = left_start.position.x + (left_goal.position.x - left_start.position.x) * s;
                right_target.position.x = right_start.position.x + (right_goal.position.x - right_start.position.x) * s;
                left_target.position.z = left_start.position.z + (left_goal.position.z - left_start.position.z) * s;
                right_target.position.z = right_start.position.z + (right_goal.position.z - right_start.position.z) * s;

                auto waypoint_state = std::make_shared<moveit::core::RobotState>(*seed_state);
                const auto ik_result = solveCoordinatedIk(
                    *seed_state, *waypoint_state, left_jmg, right_jmg,
                    left_target, right_target, ik_timeout);

                if (!ik_result.success) {
                    RCLCPP_ERROR(ctx_->get_logger(),
                                 "  ✗ [DESCEND] 连续轨迹航点 %d/%d IK失败 Left=%s Right=%s",
                                 i, num_waypoints,
                                 ik_result.left_success ? "OK" : "FAIL",
                                 ik_result.right_success ? "OK" : "FAIL");
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }

                waypoint_state->update();
                traj->addSuffixWayPoint(*waypoint_state, 0.0);
                *seed_state = *waypoint_state;
            }

            if (!parameterizeAndSmoothTrajectory(*traj, vel_scale, acc_scale,
                                                 ctx_->get_logger(), "DESCEND")) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            if (!validateManualTrajectory(*traj, robot_model, ctx_->dual_arm_group_,
                                          ctx_->get_logger(), "DESCEND")) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            traj->getRobotTrajectoryMsg(plan.trajectory_);
            plan.planning_time_ = 0.0;

            if (ctx_->dual_arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(ctx_->get_logger(), "  ✓ [DESCEND] 连续减速下降成功");
                ctx_->current_state_ = TaskState::PLACE;
                return;
            }

            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [DESCEND] 连续下降执行失败");
            ctx_->current_state_ = TaskState::ERROR;
        } catch (const std::exception& e) {
            ctx_->current_state_ = TaskState::ERROR;
        }
}

void DualArmController::executePlace()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: PLACE] 放置物体（GRASP逆操作）...");
        
        try {
            const auto mode_profile = getMotionModeProfile(ctx_->control_mode_);
            const double target_release_hand_z = ctx_->place_release_hand_z_ref_valid_
                                                     ? ctx_->place_release_hand_z_ref_
                                                     : ctx_->grasp_height_;

            auto settle_release_height = [&]() -> bool {
                const auto left_pose = ctx_->left_arm_->getCurrentPose().pose;
                const auto right_pose = ctx_->right_arm_->getCurrentPose().pose;
                const double current_release_hand_z =
                    0.5 * (left_pose.position.z + right_pose.position.z);
                const double release_height_error = current_release_hand_z - target_release_hand_z;

                if (release_height_error <= kReleaseHeightTolerance) {
                    RCLCPP_INFO(ctx_->get_logger(),
                                "  [PLACE] 当前放置高度满足松爪条件: current_z=%.3f, target_z=%.3f",
                                current_release_hand_z, target_release_hand_z);
                    return true;
                }

                RCLCPP_WARN(ctx_->get_logger(),
                            "  [PLACE] 松爪前检测到离地偏高 %.3f m，执行补偿下降",
                            release_height_error);

                auto robot_model = ctx_->dual_arm_->getRobotModel();
                auto current_state = ctx_->dual_arm_->getCurrentState();
                auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
                traj->addSuffixWayPoint(*current_state, 0.0);

                const moveit::core::JointModelGroup* left_jmg =
                    current_state->getJointModelGroup(ctx_->left_arm_group_);
                const moveit::core::JointModelGroup* right_jmg =
                    current_state->getJointModelGroup(ctx_->right_arm_group_);
                auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);

                constexpr int kCorrectionWaypoints = 12;
                auto smoothstep5 = [](double s) {
                    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
                };

                for (int i = 1; i <= kCorrectionWaypoints; ++i) {
                    const double t = static_cast<double>(i) / static_cast<double>(kCorrectionWaypoints);
                    const double s = smoothstep5(t);

                    geometry_msgs::msg::Pose left_target = left_pose;
                    geometry_msgs::msg::Pose right_target = right_pose;
                    left_target.position.z = left_pose.position.z - release_height_error * s;
                    right_target.position.z = right_pose.position.z - release_height_error * s;

                    auto waypoint_state = std::make_shared<moveit::core::RobotState>(*seed_state);
                    bool left_ik = waypoint_state->setFromIK(left_jmg, left_target, 0.12);
                    bool right_ik = waypoint_state->setFromIK(right_jmg, right_target, 0.12);

                    if (!left_ik || !right_ik) {
                        left_ik = waypoint_state->setFromIK(left_jmg, left_target, 0.5);
                        right_ik = waypoint_state->setFromIK(right_jmg, right_target, 0.5);
                    }

                    if (!left_ik || !right_ik) {
                        RCLCPP_ERROR(ctx_->get_logger(),
                                     "  ✗ [PLACE] 松爪前补偿下降 IK失败 Left=%s Right=%s",
                                     left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                        return false;
                    }

                    waypoint_state->update();
                    traj->addSuffixWayPoint(*waypoint_state, 0.0);
                    *seed_state = *waypoint_state;
                }

                if (!parameterizeAndSmoothTrajectory(*traj,
                                                     kReleaseCorrectionVelScale,
                                                     kReleaseCorrectionAccScale,
                                                     ctx_->get_logger(),
                                                     "PLACE_RELEASE_ALIGN")) {
                    return false;
                }

                if (!validateManualTrajectory(*traj, robot_model, ctx_->dual_arm_group_,
                                              ctx_->get_logger(), "PLACE_RELEASE_ALIGN")) {
                    return false;
                }

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                traj->getRobotTrajectoryMsg(plan.trajectory_);
                plan.planning_time_ = 0.0;

                if (ctx_->dual_arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [PLACE] 松爪前补偿下降执行失败");
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                const auto corrected_left = ctx_->left_arm_->getCurrentPose().pose;
                const auto corrected_right = ctx_->right_arm_->getCurrentPose().pose;
                const double corrected_release_hand_z =
                    0.5 * (corrected_left.position.z + corrected_right.position.z);
                RCLCPP_INFO(ctx_->get_logger(),
                            "  ✓ [PLACE] 补偿下降完成: corrected_z=%.3f, target_z=%.3f",
                            corrected_release_hand_z, target_release_hand_z);
                return true;
            };

            if (!settle_release_height()) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            // ========== Step A: 打开夹爪 + 去激活MuJoCo weld约束释放物体 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step A: 打开夹爪并去激活weld约束释放物体");
            
            if (!ctx_->grippers_->controlDualGrippers(0.04)) {         // 完全同步打开
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 夹爪打开失败，保持 weld/attach 状态并停止放置流程");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 夹爪已同步打开");

            // 夹爪确认打开后再解除 weld，避免开爪失败时物体状态被误报为已释放。
            {
                auto weld_msg = std_msgs::msg::Bool();
                weld_msg.data = false;
                ctx_->weld_pub_->publish(weld_msg);
                RCLCPP_INFO(ctx_->get_logger(), "  [MuJoCo] Weld约束已去激活");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            // 等待物理引擎稳定（杆件落地）
            RCLCPP_INFO(ctx_->get_logger(), "  [等待] 物理引擎稳定中（1.5秒）...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            
            // ========== Step B: 解除附着并重新添加为世界碰撞物体 ==========
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step B: 解除物体附着，重新添加为世界碰撞物体");
            if (!ctx_->arms_->detachAndReplaceObject()) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ 解除附着或重建世界物体失败，停止放置流程");
                ctx_->setCurrentState(TaskState::ERROR);
                return;
            }
            RCLCPP_INFO(ctx_->get_logger(), "  ✓ 物体已放置");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // ========== Step B2: 恢复ACM（禁止夹爪与铝棒碰撞）==========
            // GRASP Step C 中调用了 ctx_->arms_->allow_gripper_collision(true) 允许夹爪碰撞铝棒
            // 放置完成后必须恢复为禁止碰撞，保证后续阶段的碰撞检测正确性
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step B2: 允许夹爪-杆件临时碰撞（撤离后再恢复）");
            ctx_->arms_->allow_gripper_collision(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // ========== Step C: 水平撤退 + 垂直抬升（合并为TOTG密集航点）==========
            // 旋转后：左臂在+X端，右臂在-X端，杆件沿X方向
            // 撤退方向：左臂向+X，右臂向-X，同时抬升到安全高度
            // 使用TOTG避免OMPL在紧凑构型下的规划失败（之前3次测试2次OMPL失败）
            
            RCLCPP_INFO(ctx_->get_logger(), "  [PLACE] Step C+D: 连续水平撤退+垂直抬升 (IK轨迹+TOTG)");

            auto left_start_pose = ctx_->left_arm_->getCurrentPose().pose;
            auto right_start_pose = ctx_->right_arm_->getCurrentPose().pose;
            const double retreat_start_z = 0.5 * (left_start_pose.position.z + right_start_pose.position.z);
            const bool near_ground_retreat = retreat_start_z <= ctx_->grasp_height_ + 0.08;

            const int base_waypoints = near_ground_retreat ? 26 : 16;
            const int num_waypoints = scaledWaypointCount(
                base_waypoints, mode_profile.place_waypoint_scale, 12);
            const double vel_scale = clampScale(
                (near_ground_retreat ? 0.18 : 0.28) * mode_profile.place_vel_scale,
                0.06, 0.32);
            const double acc_scale = clampScale(
                (near_ground_retreat ? 0.12 : 0.22) * mode_profile.place_acc_scale,
                0.04, 0.26);
            const double ik_timeout = near_ground_retreat ? 0.15 : 0.10;

            RCLCPP_INFO(ctx_->get_logger(),
                        "  [PLACE] 模式=%s, z=%.3f, near_ground=%s, waypoints=%d, vel=%.2f, acc=%.2f",
                        ctx_->control_mode_.c_str(),
                        retreat_start_z, near_ground_retreat ? "true" : "false", num_waypoints, vel_scale, acc_scale);

            geometry_msgs::msg::Pose left_goal = left_start_pose;
            geometry_msgs::msg::Pose right_goal = right_start_pose;
            left_goal.position.x += 0.08;
            right_goal.position.x -= 0.08;
            left_goal.position.z = ctx_->lift_height_;
            right_goal.position.z = ctx_->lift_height_;

            auto robot_model = ctx_->dual_arm_->getRobotModel();
            auto current_state = ctx_->dual_arm_->getCurrentState();
            auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, ctx_->dual_arm_group_);
            traj->addSuffixWayPoint(*current_state, 0.0);

            const moveit::core::JointModelGroup* left_jmg = current_state->getJointModelGroup(ctx_->left_arm_group_);
            const moveit::core::JointModelGroup* right_jmg = current_state->getJointModelGroup(ctx_->right_arm_group_);
            auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);

            auto smoothstep5 = [](double s) {
                return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
            };

            for (int i = 1; i <= num_waypoints; ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(num_waypoints);
                const double s = smoothstep5(t);

                geometry_msgs::msg::Pose left_target = left_start_pose;
                geometry_msgs::msg::Pose right_target = right_start_pose;
                left_target.position.x = left_start_pose.position.x + (left_goal.position.x - left_start_pose.position.x) * s;
                right_target.position.x = right_start_pose.position.x + (right_goal.position.x - right_start_pose.position.x) * s;
                left_target.position.z = left_start_pose.position.z + (left_goal.position.z - left_start_pose.position.z) * s;
                right_target.position.z = right_start_pose.position.z + (right_goal.position.z - right_start_pose.position.z) * s;

                auto waypoint_state = std::make_shared<moveit::core::RobotState>(*seed_state);
                bool left_ik = waypoint_state->setFromIK(left_jmg, left_target, ik_timeout);
                bool right_ik = waypoint_state->setFromIK(right_jmg, right_target, ik_timeout);

                if (!left_ik || !right_ik) {
                    left_ik = waypoint_state->setFromIK(left_jmg, left_target, 0.5);
                    right_ik = waypoint_state->setFromIK(right_jmg, right_target, 0.5);
                }

                if (!left_ik || !right_ik) {
                    RCLCPP_ERROR(ctx_->get_logger(),
                                 "  ✗ [PLACE] 连续撤离航点 %d/%d IK失败 Left=%s Right=%s",
                                 i, num_waypoints, left_ik ? "OK" : "FAIL", right_ik ? "OK" : "FAIL");
                    ctx_->current_state_ = TaskState::ERROR;
                    return;
                }

                waypoint_state->update();
                traj->addSuffixWayPoint(*waypoint_state, 0.0);
                *seed_state = *waypoint_state;
            }

            if (!parameterizeAndSmoothTrajectory(*traj, vel_scale, acc_scale,
                                                 ctx_->get_logger(), "PLACE")) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            if (!validateManualTrajectory(*traj, robot_model, ctx_->dual_arm_group_,
                                          ctx_->get_logger(), "PLACE", false)) {
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            traj->getRobotTrajectoryMsg(plan.trajectory_);
            plan.planning_time_ = 0.0;

            if (ctx_->dual_arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [PLACE] 连续撤离轨迹执行失败");
                ctx_->current_state_ = TaskState::ERROR;
                return;
            }

            RCLCPP_INFO(ctx_->get_logger(), "  ✓ [PLACE] 连续撤离阶段完成");
            ctx_->current_state_ = TaskState::RETREAT;

            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ctx_->get_logger(), "PLACE 阶段异常: %s", e.what());
            ctx_->current_state_ = TaskState::ERROR;
        }
    }

bool DualArmController::detachAndReplaceObject()
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
        
        // The rod's local long axis is its Y axis. Infer its world yaw from
        // the current hand baseline instead of assuming a fixed 45° target.
        const double hand_dx = right_current.pose.position.x - left_current.pose.position.x;
        const double hand_dy = right_current.pose.position.y - left_current.pose.position.y;
        double rod_yaw = std::atan2(hand_dy, hand_dx) + M_PI / 2.0;
        while (rod_yaw > M_PI) rod_yaw -= 2.0 * M_PI;
        while (rod_yaw < -M_PI) rod_yaw += 2.0 * M_PI;

        RCLCPP_INFO(ctx_->get_logger(), "    杆件放置位置: (%.3f, %.3f, %.3f)，方向 Rz=%.1f°",
            rod_center_x, rod_center_y, rod_center_z, rod_yaw * 180.0 / M_PI);
        
        // ========== 步骤1: 解除附着 ==========
        // 修复：之前只是发了REMOVE消息到话题，但由于ROS 2 MoveIt架构中话题处理的延迟或覆盖，
        // 常常无法正确更新 attached_objects，导致MoveIt依然认为物体粘在手上（RETREAT时发生碰撞）。
        // 改用高层API detachObject 会通过 Service 进行可靠同步。
        bool detach_success = ctx_->left_arm_->detachObject("aluminum_rod");
        if (!detach_success) {
            // The object is normally attached to mj_left_hand. Only fall back
            // to the dual-arm interface when the primary detach request fails;
            // issuing both requests creates a stale second detach and produces
            // misleading "Attached body not found" errors in MoveIt.
            detach_success = ctx_->dual_arm_->detachObject("aluminum_rod");
        }
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 执行解除附着: %s", detach_success ? "成功" : "失败");
        if (!detach_success) {
            RCLCPP_ERROR(ctx_->get_logger(), "    ✗ 解除附着请求失败");
            return false;
        }

        moveit::planning_interface::PlanningSceneInterface psi;
        bool attached_body_removed = false;
        for (int attempt = 0; attempt < 15; ++attempt) {
            auto attached_objects = psi.getAttachedObjects({"aluminum_rod"});
            if (attached_objects.count("aluminum_rod") == 0) {
                attached_body_removed = true;
                break;
            }
            std::this_thread::sleep_for(100ms);
        }
        if (!attached_body_removed) {
            RCLCPP_ERROR(ctx_->get_logger(),
                         "    ✗ PlanningScene 仍存在 attached aluminum_rod，禁止重新加入世界物体");
            return false;
        }
        
        // ========== 步骤2: 重新添加为世界碰撞物体（旋转后姿态）==========
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
        q_rod.setRPY(0, 0, rod_yaw);
        rod_pose.orientation = tf2::toMsg(q_rod);
        
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(rod_pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        
        // 修复：使用 PlanningSceneInterface 的 Apply 方法强制同步更新世界物体
        const bool apply_success = psi.applyCollisionObject(collision_object);
        if (!apply_success) {
            RCLCPP_ERROR(ctx_->get_logger(), "    ✗ 重新添加铝棒为世界碰撞物体失败");
            return false;
        }
        
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 重新添加铝棒为世界碰撞物体（Rz=%.1f°）",
            rod_yaw * 180.0 / M_PI);
        std::this_thread::sleep_for(300ms);
        
        // ========== 步骤3: 验证结果 ==========
        auto objects = psi.getObjects();
        auto attached_objects = psi.getAttachedObjects();
        
        RCLCPP_INFO(ctx_->get_logger(), "    世界物体: %zu (期望≥1), 附着物体: %zu (期望=0)",
                    objects.size(), attached_objects.size());

        const bool world_has_rod = objects.count("aluminum_rod") > 0;
        const bool attached_has_rod = attached_objects.count("aluminum_rod") > 0;

        if (!world_has_rod || attached_has_rod) {
            RCLCPP_ERROR(ctx_->get_logger(),
                         "    ✗ 放置后场景状态不一致: world_has_rod=%s attached_has_rod=%s",
                         world_has_rod ? "true" : "false",
                         attached_has_rod ? "true" : "false");
            return false;
        }
        
        RCLCPP_INFO(ctx_->get_logger(), "    ✓ 解除附着并重新放置完成");
        return true;
    }

void DualArmController::executeRetreat()
    {
        RCLCPP_INFO(ctx_->get_logger(), "[状态: RETREAT] 撤退至初始位置...");
        try {
            if (ctx_->initial_left_joints_.empty() || ctx_->initial_right_joints_.empty()) {
                ctx_->initial_left_joints_ = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
                ctx_->initial_right_joints_ = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
            }
            
            ctx_->dual_arm_->setMaxVelocityScalingFactor(0.8);
            ctx_->dual_arm_->setMaxAccelerationScalingFactor(0.8);
            
            // Allow collision between hand and object for retreat start state
            allow_gripper_collision(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            ctx_->dual_arm_->setStartStateToCurrentState();
            ctx_->dual_arm_->clearPoseTargets();
            ctx_->dual_arm_->setPlannerId("PTP"); // Point-to-point motion in joint space natively supported by Pilz
            
            moveit::core::RobotState target_state(*ctx_->dual_arm_->getCurrentState());
            target_state.setJointGroupPositions(ctx_->left_arm_group_, ctx_->initial_left_joints_);
            target_state.setJointGroupPositions(ctx_->right_arm_group_, ctx_->initial_right_joints_);
            ctx_->dual_arm_->setJointValueTarget(target_state);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (ctx_->dual_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                if (ctx_->dual_arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(ctx_->get_logger(), "  ✓ [RETREAT] Pilz PTP 返航成功");
                    ctx_->current_state_ = TaskState::DONE;
                    allow_gripper_collision(false);
                    return;
                }
            }
            
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [RETREAT] Pilz PTP 规划执行失败");
            ctx_->current_state_ = TaskState::ERROR;
        } catch (const std::exception& e) {
            ctx_->current_state_ = TaskState::ERROR;
        }
}
