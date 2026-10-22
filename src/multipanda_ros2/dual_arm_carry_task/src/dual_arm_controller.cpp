#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#if __has_include(<moveit/trajectory_processing/ruckig_traj_smoothing.h>)
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#define DUAL_ARM_HAS_RUCKIG 1
#else
#define DUAL_ARM_HAS_RUCKIG 0
#endif
#include <cctype>
#include <cmath>
#include <array>
#include <iomanip>
#include <fstream>
#include <limits>
#include <memory>
#include <deque>
#include <unordered_map>
#include <vector>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_model/prismatic_joint_model.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <mujoco_ros_msgs/srv/get_body_state.hpp>
#include <cstdio>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include "dual_arm_carry_task/dual_arm_controller.hpp"
#include "dual_arm_carry_task/gripper_protocol.hpp"
#include "dual_arm_carry_task/gripper_client.hpp"
#include "dual_arm_carry_task/grasp_contact_acceptance.hpp"
#include "dual_arm_carry_task/s1_object_pose.hpp"
#include "dual_arm_carry_task/s1_time_scaling.hpp"

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

        if ((ctx_->contact_hover_control_mode_ == "mmc_hold" ||
             ctx_->contact_abcd_group_ == "C" || ctx_->contact_abcd_group_ == "D") &&
            !ensureTaskRobotStatesService()) {
            RCLCPP_ERROR(ctx_->get_logger(), "[MMC_STARTUP_PREFLIGHT] result=failed reason=state_source_creation_failed");
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
        // directly with command.position. All FOUR joints are read separately:
        // the gripper nodes publish the two real finger values, so a total gap
        // is q1 + q2 per gripper and not one finger doubled.
        auto logFingerJoints = [&](const char* label) {
            const moveit::core::RobotStatePtr st = ctx_->dual_arm_->getCurrentState();
            std::string vars;
            for (const std::string& v : st->getVariableNames()) {
                vars += " " + v;
            }
            static const char* kFour[] = {"mj_left_finger_joint1", "mj_left_finger_joint2",
                                          "mj_right_finger_joint1", "mj_right_finger_joint2"};
            for (const char* v : kFour) {
                if (vars.find(v) == std::string::npos) {
                    RCLCPP_WARN(ctx_->get_logger(),
                                "  [P3A-2A] %s: 机器人模型中不含 %s，无法读取实测指关节",
                                label, v);
                    return;
                }
            }
            st->update();
            const double l1 = st->getVariablePosition(kFour[0]);
            const double l2 = st->getVariablePosition(kFour[1]);
            const double r1 = st->getVariablePosition(kFour[2]);
            const double r2 = st->getVariablePosition(kFour[3]);
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P3A-2A] %s 实测四指: 左(%.4f, %.4f) 右(%.4f, %.4f) m ⇒ "
                        "实测总开口 左=%.4f 右=%.4f m（各为两指之和）",
                        label, l1, l2, r1, r2, l1 + l2, r1 + r2);
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

        if (ctx_->contact_hover_control_mode_ == "mmc_hold" ||
            ctx_->contact_abcd_group_ == "C" || ctx_->contact_abcd_group_ == "D") {
            std::string preflight_error;
            if (!runMmcStartupPreflight(preflight_error)) {
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [A1] %s", preflight_error.c_str());
                ctx_->setCurrentState(TaskState::ERROR);
                return false;
            }
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
                        if (!arrived) {
                            return false;
                        }
                        // A1 P3a-2b: with the descent switch on, continue from
                        // the pose just reached to the grasp pose and stop.
                        // This is a strict continuation of the pre-grasp entry:
                        // it can never reach the legacy GRASP/FSM.
                        if (ctx_->grasp_descent_) {
                            std::string descent_detail;
                            const bool descended = runGraspDescent(truth, descent_detail);
                            RCLCPP_INFO(ctx_->get_logger(),
                                        "[P3A2B_RESULT] state=%s detail=%s",
                                        descended ? "GRASP_POSE_REACHED" : "FAILED",
                                        descent_detail.c_str());
                            if (descended) {
                                if (ctx_->grasp_contact_close_) {
                                    std::string contact_detail;
                                    const bool contact = runGraspContact(truth, contact_detail);
                                    RCLCPP_INFO(ctx_->get_logger(),
                                        "[P3A2C_RESULT] state=%s detail=%s",
                                        contact ? "GRASP_CONTACT_CONFIRMED" : "FAILED",
                                        contact_detail.c_str());
                                    RCLCPP_INFO(ctx_->get_logger(),
                                        "  [P3A-2C] action 协议成功与任务双侧接触验收分开记录；"
                                        "weld=0 attach=0 lift=0 carry=0");
                                    ctx_->setCurrentState(contact ? TaskState::GRASP_CONTACT_CONFIRMED
                                                                  : TaskState::ERROR);
                                } else if (ctx_->grasp_model_ == "contact_probe") {
                                    std::string probe_detail;
                                    const bool probe_ok = runContactLoadProbe(
                                        truth, probe_detail, ctx_->contact_hold_diagnostic_only_,
                                        ctx_->contact_plan_compare_);
                                    if (ctx_->contact_hold_diagnostic_only_) {
                                        RCLCPP_INFO(ctx_->get_logger(),
                                            "[CONTACT_HOLD_MECHANISM_RESULT] state=%s detail=%s",
                                            probe_ok ? "PASS" : "FAILED", probe_detail.c_str());
                                    } else if (ctx_->contact_plan_compare_) {
                                        RCLCPP_INFO(ctx_->get_logger(),
                                            "[CONTACT_PROBE_RESULT] state=%s detail=%s",
                                            probe_ok ? "PLAN_COMPARE_DONE" : "FAILED",
                                            probe_detail.c_str());
                                    } else {
                                        RCLCPP_INFO(ctx_->get_logger(), "[CONTACT_PROBE_RESULT] state=%s detail=%s",
                                            probe_ok ? "DONE" : "FAILED", probe_detail.c_str());
                                    }
                                    if (ctx_->contact_hold_diagnostic_only_) {
                                        ctx_->setCurrentState(probe_ok ?
                                            TaskState::CONTACT_HOLD_MECHANISM_VALIDATED : TaskState::ERROR);
                                    } else if (ctx_->contact_plan_compare_) {
                                        ctx_->setCurrentState(probe_ok ?
                                            TaskState::PLAN_COMPARE_DONE : TaskState::ERROR);
                                    } else {
                                        ctx_->setCurrentState(probe_ok ? TaskState::DONE : TaskState::ERROR);
                                    }
                                } else if (ctx_->grasp_model_ == "ideal_weld") {
                                    std::string carry_detail;
                                    const bool carried = runIdealWeldCarry(truth, carry_detail);
                                    RCLCPP_INFO(ctx_->get_logger(),
                                        "[S1_IDEAL_WELD_RESULT] state=%s detail=%s",
                                        carried ? "DONE" : "FAILED", carry_detail.c_str());
                                    ctx_->setCurrentState(carried ? TaskState::DONE : TaskState::ERROR);
                                } else {
                                    RCLCPP_INFO(ctx_->get_logger(),
                                        "  [P3A-2B] 本轮结束于 GRASP_POSE_REACHED；夹爪保持打开，"
                                        "未闭合、未 weld、未 attach、未抬升、未搬运");
                                    ctx_->setCurrentState(TaskState::GRASP_POSE_REACHED);
                                }
                            } else {
                                RCLCPP_INFO(ctx_->get_logger(),
                                            "  [P3A-2B] 下降失败；不开始闭合");
                                ctx_->setCurrentState(TaskState::ERROR);
                            }
                            return (descended && !ctx_->grasp_contact_close_ &&
                                    ctx_->grasp_model_ != "ideal_weld" && ctx_->grasp_model_ != "contact_probe") ||
                                   (ctx_->getCurrentState() == TaskState::GRASP_CONTACT_CONFIRMED) ||
                                   (ctx_->getCurrentState() == TaskState::DONE);
                        }
                        return true;
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
                if (ctx_->contact_transport_plan_only_) {
                    // S2: build the transport segment from the reconstructed
                    // nominal grasp/lift state and compare the baseline path
                    // with the closed-chain projected path. Planning only.
                    RCLCPP_INFO(ctx_->get_logger(),
                                "\n----- [S2] 运输段闭链投影比较（仅规划，不运动）-----");
                    std::string opt_detail;
                    const bool opt_ok = runContactLoadProbe(truth, opt_detail, false, false);
                    RCLCPP_INFO(ctx_->get_logger(), "[TRANSPORT_OPT_RESULT] state=%s detail=%s",
                                opt_ok ? "PLAN_ONLY_DONE" : "FAILED", opt_detail.c_str());
                    ctx_->setCurrentState(opt_ok ? TaskState::PLAN_COMPARE_DONE
                                                 : TaskState::ERROR);
                    return opt_ok;
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
    // The first real-load trace showed the pad/rod contact centroid 7.3 mm
    // above the rod mid-plane, then migrating to the top edge during lift.
    // Lower only this explicit contact-probe grasp profile to center the pad
    // band on the rod side; keep all legacy and ideal_weld targets unchanged.
    const double contact_probe_grasp_z_offset = ctx_->grasp_model_ == "contact_probe" ? 0.0123 : 0.0;
    gg.pre_grasp_height_m = ctx_->approach_height_ - contact_probe_grasp_z_offset;
    gg.grasp_height_m = ctx_->grasp_height_ - contact_probe_grasp_z_offset;

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
    if (ctx_->grasp_model_ == "contact_probe") {
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_PROBE_GEOMETRY] vertical grasp/pregrasp offset=-%.1f mm; pad band lowered to the rod lower half (slip budget)",
            contact_probe_grasp_z_offset * 1000.0);
    }
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
        a1_verify_.grasp_target = grasp;
        a1_verify_.left_eef = left_eef;
        a1_verify_.right_eef = right_eef;
        a1_verify_.start = std::make_shared<moveit::core::RobotState>(mapState(*start));
        a1_verify_.pregrasp = std::make_shared<moveit::core::RobotState>(mj_pre_state);
        a1_verify_.grasp = std::make_shared<moveit::core::RobotState>(mj_grasp_state);
        for (const std::string& v : pre_state.getVariableNames()) {
            a1_verify_.pregrasp_joints[v] = pre_state.getVariablePosition(v);
        }
        for (const std::string& v : mj_grasp_state.getVariableNames()) {
            a1_verify_.grasp_joints[v] = mj_grasp_state.getVariablePosition(v);
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
    const auto live_model = dual->getRobotModel();
    moveit::core::RobotStatePtr live_current = dual->getCurrentState();
    if (!live_current || !live_model) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_LIVE_STATE_UNAVAILABLE";
        out.path_validation.detail = "MoveIt live model/current state is unavailable";
        return false;
    }

    // Preserve a same-state collision comparison for the P2b rejection
    // diagnostic. This does not alter the planning scene, ACM, or the state
    // used by MoveGroup; it maps one fresh /joint_states snapshot by name into
    // the existing live and criterion models.
    DualArmCarryTask::JointFeedback fresh_feedback;
    const bool have_fresh_feedback = ctx_->latestJointFeedback(fresh_feedback);
    double feedback_age_sec = std::numeric_limits<double>::infinity();
    std::string safe_run_id = ctx_->expected_run_id_.empty() ? "p2b_diagnostic" : ctx_->expected_run_id_;
    for (char& ch : safe_run_id) {
        if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_' && ch != '-') ch = '_';
    }
    const std::string diag_dir = "experiment_results/a1_runs/" + safe_run_id + "/";
    std::ofstream state_csv(diag_dir + "p2b_state_diagnostic.csv", std::ios::out | std::ios::trunc);
    std::set<std::string> live_var_set(live_current->getVariableNames().begin(),
                                      live_current->getVariableNames().end());
    std::set<std::string> criterion_var_set(model->getVariableNames().begin(),
                                            model->getVariableNames().end());
    std::vector<std::string> diagnostic_names;
    for (const auto& name : model->getVariableNames()) diagnostic_names.push_back(name);
    if (state_csv) {
        state_csv << "source,feedback_seq,feedback_stamp_ns,feedback_age_sec";
        for (const auto& name : diagnostic_names) state_csv << ",q_" << name;
        state_csv << '\n';
    }
    auto writeDiagnosticState = [&](const std::string& source,
                                    const moveit::core::RobotState& state) {
        if (!state_csv) return;
        const std::set<std::string> source_variables(
            state.getRobotModel()->getVariableNames().begin(),
            state.getRobotModel()->getVariableNames().end());
        state_csv << source << ',' << (have_fresh_feedback ? fresh_feedback.seq : 0) << ','
                  << (have_fresh_feedback && fresh_feedback.stamp_valid
                          ? fresh_feedback.stamp.nanoseconds() : 0) << ','
                  << (have_fresh_feedback ? feedback_age_sec : -1.0);
        for (const auto& name : diagnostic_names) {
            state_csv << ',' << (source_variables.count(name) != 0
                ? state.getVariablePosition(name) : std::numeric_limits<double>::quiet_NaN());
        }
        state_csv << '\n';
    };

    DualArmCarryTask::GripperContactFeedback left_finger_feedback;
    DualArmCarryTask::GripperContactFeedback right_finger_feedback;
    const bool have_left_finger = ctx_->latestGripperContact("mj_left", left_finger_feedback);
    const bool have_right_finger = ctx_->latestGripperContact("mj_right", right_finger_feedback);
    const double feedback_now_steady_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    feedback_age_sec = have_fresh_feedback
        ? feedback_now_steady_sec - fresh_feedback.recv_steady_sec
        : std::numeric_limits<double>::infinity();
    const double left_finger_age = have_left_finger
        ? feedback_now_steady_sec - left_finger_feedback.recv_steady_sec
        : std::numeric_limits<double>::infinity();
    const double right_finger_age = have_right_finger
        ? feedback_now_steady_sec - right_finger_feedback.recv_steady_sec
        : std::numeric_limits<double>::infinity();
    constexpr double kP2bMaxFingerAgeSec = 0.50;
    constexpr double kFingerAcceptedLowerM = a1v::kFingerNumericLowerBoundM;
    constexpr double kFingerNominalLowerM = a1v::kFingerNominalLowerBoundM;
    constexpr double kFingerAcceptedUpperM = a1v::kFingerAcceptedUpperBoundM;
    const double quiet_nan = std::numeric_limits<double>::quiet_NaN();
    const std::array<double, 4> measured_fingers{{
        have_left_finger ? left_finger_feedback.q1 : quiet_nan,
        have_left_finger ? left_finger_feedback.q2 : quiet_nan,
        have_right_finger ? right_finger_feedback.q1 : quiet_nan,
        have_right_finger ? right_finger_feedback.q2 : quiet_nan}};
    const std::array<bool, 4> finger_sample_present{{
        have_left_finger, have_left_finger, have_right_finger, have_right_finger}};
    const std::array<double, 4> finger_sample_age{{
        left_finger_age, left_finger_age, right_finger_age, right_finger_age}};
    const auto& finger_names = a1v::fingerJointNames();
    const std::array<std::string, 4> feedback_fields{{
        "mj_left.finger_q1", "mj_left.finger_q2",
        "mj_right.finger_q1", "mj_right.finger_q2"}};
    const std::array<std::string, 4> field_joint_names{{
        "mj_left_finger_joint1", "mj_left_finger_joint2",
        "mj_right_finger_joint1", "mj_right_finger_joint2"}};
    const std::array<double, 2> left_right_sim_dt{{
        have_left_finger && have_right_finger
            ? left_finger_feedback.simulation_time - right_finger_feedback.simulation_time
            : quiet_nan,
        have_left_finger && have_right_finger
            ? left_finger_feedback.recv_steady_sec - right_finger_feedback.recv_steady_sec
            : quiet_nan}};
    std::array<a1v::FingerInputClass, 4> finger_classes{};
    std::array<double, 4> model_lower{};
    std::array<double, 4> model_upper{};
    std::array<bool, 4> model_bounded{};
    std::array<double, 4> live_model_lower{};
    std::array<double, 4> live_model_upper{};
    std::array<bool, 4> live_model_bounded{};
    for (std::size_t i = 0; i < finger_names.size(); ++i) {
        const bool criterion_has_name = criterion_var_set.count(finger_names[i]) != 0;
        const bool live_has_name = live_var_set.count(finger_names[i]) != 0;
        const bool order_matches = field_joint_names[i] == finger_names[i];
        if (criterion_has_name) {
            const auto& bounds = model->getVariableBounds(finger_names[i]);
            model_bounded[i] = bounds.position_bounded_;
            model_lower[i] = bounds.position_bounded_ ? bounds.min_position_ : quiet_nan;
            model_upper[i] = bounds.position_bounded_ ? bounds.max_position_ : quiet_nan;
        } else {
            model_bounded[i] = false;
            model_lower[i] = quiet_nan;
            model_upper[i] = quiet_nan;
        }
        if (live_has_name) {
            const auto& bounds = live_model->getVariableBounds(finger_names[i]);
            live_model_bounded[i] = bounds.position_bounded_;
            live_model_lower[i] = bounds.position_bounded_ ? bounds.min_position_ : quiet_nan;
            live_model_upper[i] = bounds.position_bounded_ ? bounds.max_position_ : quiet_nan;
        } else {
            live_model_bounded[i] = false;
            live_model_lower[i] = quiet_nan;
            live_model_upper[i] = quiet_nan;
        }
        const bool sample_present = finger_sample_present[i];
        const double age = finger_sample_age[i];
        finger_classes[i] = a1v::classifyP2bFingerInput(
            measured_fingers[i], sample_present, age, kP2bMaxFingerAgeSec,
            field_joint_names[i], finger_names[i], kFingerAcceptedLowerM, kFingerAcceptedUpperM);
        const auto& source = i < 2 ? left_finger_feedback : right_finger_feedback;
        const std::string expected_arm = i < 2 ? "mj_left" : "mj_right";
        if (sample_present && source.arm != expected_arm) {
            finger_classes[i] = a1v::FingerInputClass::MAPPING_MISMATCH;
        }
        if (!criterion_has_name || !live_has_name) {
            finger_classes[i] = a1v::FingerInputClass::MAPPING_MISMATCH;
        }
        if (!order_matches) finger_classes[i] = a1v::FingerInputClass::MAPPING_MISMATCH;
    }

    // Preserve every raw typed value before validation. Missing samples are
    // written as NA; a received NaN/Inf is written as such and never replaced,
    // clamped, or absolutized.
    std::ofstream raw_finger_csv(diag_dir + "p2b_finger_raw_values.csv",
                                 std::ios::out | std::ios::trunc);
    if (raw_finger_csv) {
        a1v::writeP2bFingerRawDiagnosticCsv(raw_finger_csv, measured_fingers,
                                            finger_sample_present, finger_classes);
    }
    std::ofstream finger_csv(diag_dir + "p2b_finger_input_diagnostic.csv",
                             std::ios::out | std::ios::trunc);
    if (finger_csv) {
        finger_csv << "input_index,source_arm,source_arm_matches,feedback_field,joint_name,"
                      "array_order_matches,sample_present,"
                      "sample_seq,simulation_time_s,header_stamp_ns,receive_steady_sec,age_sec,"
                      "message_has_valid_field,source_q_finite,publisher_publish_gate_eligible,"
                      "raw_value_m,field_finite,nominal_lower_m,accepted_lower_m,accepted_upper_m,"
                      "criterion_position_bounded,criterion_lower_m,"
                      "criterion_upper_m,live_position_bounded,live_lower_m,live_upper_m,classification,"
                      "joint_states_present,joint_states_value_m,"
                      "left_minus_right_sim_time_s,left_minus_right_receive_steady_s\n";
        finger_csv << std::setprecision(17);
        const auto write_number = [&finger_csv](double value, bool present) {
            if (!present) finger_csv << "NA";
            else if (std::isnan(value)) finger_csv << "NaN";
            else if (std::isinf(value)) finger_csv << (value < 0.0 ? "-Inf" : "Inf");
            else finger_csv << value;
        };
        for (std::size_t i = 0; i < finger_names.size(); ++i) {
            const bool left_side = i < 2;
            const bool present = finger_sample_present[i];
            const auto& sample = left_side ? left_finger_feedback : right_finger_feedback;
            const std::string expected_arm = left_side ? "mj_left" : "mj_right";
            const bool source_arm_matches = present && sample.arm == expected_arm;
            const bool raw_finite = present && std::isfinite(measured_fingers[i]);
            const bool derived_q_finite = present && std::isfinite(sample.q1) &&
                                          std::isfinite(sample.q2);
            const bool publisher_gate_eligible = present && sample.seq > 0 && derived_q_finite &&
                std::isfinite(sample.command_per_finger_m);
            const auto global = have_fresh_feedback
                ? fresh_feedback.positions.find(finger_names[i])
                : std::map<std::string, double>::const_iterator{};
            const bool global_present = have_fresh_feedback &&
                                        global != fresh_feedback.positions.end();
            finger_csv << i << ',' << (present ? sample.arm : "NA") << ','
                       << (present ? (source_arm_matches ? 1 : 0) : -1) << ','
                       << feedback_fields[i] << ',' << finger_names[i] << ','
                       << (field_joint_names[i] == finger_names[i] ? 1 : 0) << ','
                       << (present ? 1 : 0) << ',';
            if (present) finger_csv << sample.seq;
            else finger_csv << "NA";
            finger_csv << ',';
            write_number(present ? sample.simulation_time : quiet_nan, present);
            finger_csv << ',';
            if (present) finger_csv << sample.stamp_ns;
            else finger_csv << "NA";
            finger_csv << ',';
            write_number(present ? sample.recv_steady_sec : quiet_nan, present);
            finger_csv << ',';
            write_number(present ? finger_sample_age[i] : quiet_nan, present);
            finger_csv << ",0," << (derived_q_finite ? 1 : 0) << ','
                       << (publisher_gate_eligible ? 1 : 0) << ',';
            write_number(measured_fingers[i], present);
            finger_csv << ',' << (raw_finite ? 1 : 0) << ',' << kFingerNominalLowerM << ','
                       << kFingerAcceptedLowerM << ','
                       << kFingerAcceptedUpperM << ',' << (model_bounded[i] ? 1 : 0) << ',';
            write_number(model_lower[i], model_bounded[i]);
            finger_csv << ',';
            write_number(model_upper[i], model_bounded[i]);
            finger_csv << ',' << (live_model_bounded[i] ? 1 : 0) << ',';
            write_number(live_model_lower[i], live_model_bounded[i]);
            finger_csv << ',';
            write_number(live_model_upper[i], live_model_bounded[i]);
            finger_csv << ',' << a1v::fingerInputClassName(finger_classes[i]) << ','
                       << (global_present ? 1 : 0) << ',';
            write_number(global_present ? global->second : quiet_nan, global_present);
            finger_csv << ',';
            write_number(left_right_sim_dt[0], have_left_finger && have_right_finger);
            finger_csv << ',';
            write_number(left_right_sim_dt[1], have_left_finger && have_right_finger);
            finger_csv << '\n';
        }
        finger_csv.flush();
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[P2B_FINGER_INPUT] diagnostic=%s left_present=%d left_seq=%llu left_sim=%.9f "
        "left_recv=%.9f left_age=%.6f right_present=%d right_seq=%llu right_sim=%.9f "
        "right_recv=%.9f right_age=%.6f sim_dt=%.9f receive_dt=%.9f msg_valid_field=absent "
        "array_order=left.q1,left.q2,right.q1,right.q2; values preserved before validation",
        ("experiment_results/a1_runs/" + safe_run_id + "/p2b_finger_input_diagnostic.csv").c_str(),
        have_left_finger ? 1 : 0,
        static_cast<unsigned long long>(have_left_finger ? left_finger_feedback.seq : 0),
        have_left_finger ? left_finger_feedback.simulation_time : quiet_nan,
        have_left_finger ? left_finger_feedback.recv_steady_sec : quiet_nan,
        have_left_finger ? left_finger_age : quiet_nan,
        have_right_finger ? 1 : 0,
        static_cast<unsigned long long>(have_right_finger ? right_finger_feedback.seq : 0),
        have_right_finger ? right_finger_feedback.simulation_time : quiet_nan,
        have_right_finger ? right_finger_feedback.recv_steady_sec : quiet_nan,
        have_right_finger ? right_finger_age : quiet_nan,
        left_right_sim_dt[0], left_right_sim_dt[1]);

    for (std::size_t i = 0; i < finger_classes.size(); ++i) {
        if (finger_classes[i] == a1v::FingerInputClass::ACCEPTED_WITH_NUMERIC_TOLERANCE) {
            RCLCPP_WARN(ctx_->get_logger(),
                "[P2B_FINGER_RANGE] joint=%s raw=%.17g m accepted_with_numeric_tolerance; "
                "nominal=[%.9g,%.9g] m accepted_lower=%.9g m; raw value unchanged",
                finger_names[i].c_str(), measured_fingers[i], kFingerNominalLowerM,
                kFingerAcceptedUpperM, kFingerAcceptedLowerM);
        } else if (finger_classes[i] == a1v::FingerInputClass::VALID) {
            RCLCPP_INFO(ctx_->get_logger(),
                "[P2B_FINGER_RANGE] joint=%s raw=%.17g m valid_in_nominal_range=[%.9g,%.9g] m",
                finger_names[i].c_str(), measured_fingers[i], kFingerNominalLowerM,
                kFingerAcceptedUpperM);
        }
    }

    std::map<std::string, double> p2b_start_positions;
    std::string start_mapping_reason;
    bool finger_input_valid = true;
    std::string finger_input_failure;
    for (std::size_t i = 0; i < finger_classes.size(); ++i) {
        if (finger_classes[i] != a1v::FingerInputClass::VALID &&
            finger_classes[i] != a1v::FingerInputClass::ACCEPTED_WITH_NUMERIC_TOLERANCE) {
            finger_input_valid = false;
            if (finger_input_failure.empty()) {
                std::ostringstream detail;
                detail << finger_names[i] << '=';
                if (!finger_sample_present[i]) detail << "NA";
                else detail << std::setprecision(17) << measured_fingers[i];
                detail << " class=" << a1v::fingerInputClassName(finger_classes[i]);
                finger_input_failure = detail.str();
            }
        }
    }
    if (!have_fresh_feedback || !have_left_finger || !have_right_finger ||
        !finger_input_valid ||
        !a1v::mapP2bStartPositions(fresh_feedback.positions, measured_fingers,
                                   feedback_age_sec, {{left_finger_age, right_finger_age}},
                                   kP2bMaxFingerAgeSec, p2b_start_positions, start_mapping_reason)) {
        out.path_validation.status = a1v::CheckStatus::FAILED;
        out.path_validation.reason_code = "E_P2B_START_FEEDBACK_INVALID";
        out.path_validation.detail = !finger_input_failure.empty()
            ? "typed finger feedback rejected: " + finger_input_failure
            : "fresh arm/finger start feedback is incomplete: " +
                (start_mapping_reason.empty() ? std::string("required feedback source missing")
                                              : start_mapping_reason);
        RCLCPP_ERROR(ctx_->get_logger(), "[P2B_STATE_DIAG] %s; raw finger snapshot saved before validation",
                     out.path_validation.detail.c_str());
        return false;
    }

    moveit::core::RobotState p2b_start_criterion(*a1_verify_.start);
    moveit::core::RobotState p2b_start_live(*live_current);
    std::size_t mapped_start_count = 0;
    for (const auto& entry : p2b_start_positions) {
        if (criterion_var_set.count(entry.first) == 0 || live_var_set.count(entry.first) == 0) {
            out.path_validation.status = a1v::CheckStatus::FAILED;
            out.path_validation.reason_code = "E_P2B_START_JOINT_MAPPING_INVALID";
            out.path_validation.detail = "live/criterion model missing " + entry.first;
            return false;
        }
        p2b_start_criterion.setVariablePosition(entry.first, entry.second);
        p2b_start_live.setVariablePosition(entry.first, entry.second);
        ++mapped_start_count;
    }
    p2b_start_criterion.update();
    p2b_start_live.update();
    moveit::core::RobotState same_state_criterion(p2b_start_criterion);
    moveit::core::RobotState same_state_live(p2b_start_live);
    writeDiagnosticState("fresh_joint_feedback_plus_typed_fingers", same_state_criterion);
    writeDiagnosticState("criterion_recorded_start", *a1_verify_.start);
    writeDiagnosticState("moveit_live_current_state", *live_current);
    RCLCPP_INFO(ctx_->get_logger(),
        "[P2B_STATE_DIAG] joint_states_topic=%s seq=%llu stamp_ns=%lld age=%.6fs "
        "arm_feedback_names=14/14 measured_start_variables=%zu/18 finger_seq=(%llu,%llu) "
        "finger_age=(%.6f,%.6f)s criterion_model_vars=%zu live_model_vars=%zu criterion_hand_mesh=%s",
        ctx_->joint_states_topic_.c_str(),
        static_cast<unsigned long long>(have_fresh_feedback ? fresh_feedback.seq : 0),
        static_cast<long long>(have_fresh_feedback && fresh_feedback.stamp_valid
                                   ? fresh_feedback.stamp.nanoseconds() : 0),
        have_fresh_feedback ? feedback_age_sec : -1.0, mapped_start_count,
        static_cast<unsigned long long>(left_finger_feedback.seq),
        static_cast<unsigned long long>(right_finger_feedback.seq), left_finger_age,
        right_finger_age, diagnostic_names.size(), live_model->getVariableNames().size(),
        a1_verify_.mesh_path.c_str());
    for (const auto& name : diagnostic_names) {
        const bool feedback_has = have_fresh_feedback && fresh_feedback.positions.count(name) != 0;
        const bool live_has = live_var_set.count(name) != 0;
        const double measured = feedback_has ? fresh_feedback.positions.at(name)
            : std::numeric_limits<double>::quiet_NaN();
        const double csm = live_has ? live_current->getVariablePosition(name)
            : std::numeric_limits<double>::quiet_NaN();
        const double recorded = criterion_var_set.count(name) != 0
            ? a1_verify_.start->getVariablePosition(name) : std::numeric_limits<double>::quiet_NaN();
        RCLCPP_INFO(ctx_->get_logger(),
            "[P2B_STATE_JOINT] name=%s feedback_present=%d feedback=%.9f live_present=%d live=%.9f "
            "criterion=%.9f feedback_minus_live=%.9f feedback_minus_criterion=%.9f",
            name.c_str(), feedback_has ? 1 : 0, measured, live_has ? 1 : 0, csm, recorded,
            measured - csm, measured - recorded);
    }
    for (const std::string side : {"mj_left", "mj_right"}) {
        const auto& finger_feedback = side == "mj_left" ? left_finger_feedback : right_finger_feedback;
        const double finger_age = side == "mj_left" ? left_finger_age : right_finger_age;
        const std::array<std::string, 2> finger_joint_names = side == "mj_left"
            ? std::array<std::string, 2>{{"mj_left_finger_joint1", "mj_left_finger_joint2"}}
            : std::array<std::string, 2>{{"mj_right_finger_joint1", "mj_right_finger_joint2"}};
        for (std::size_t fi = 0; fi < finger_joint_names.size(); ++fi) {
            const auto global = have_fresh_feedback
                ? fresh_feedback.positions.find(finger_joint_names[fi])
                : std::map<std::string, double>::const_iterator{};
            const bool global_present = have_fresh_feedback && global != fresh_feedback.positions.end();
            const double typed = fi == 0 ? finger_feedback.q1 : finger_feedback.q2;
            const double joint_state = global_present ? global->second
                : std::numeric_limits<double>::quiet_NaN();
            RCLCPP_INFO(ctx_->get_logger(),
                "[P2B_FINGER_DIAG] source=typed_hardware+joint_states side=%s joint=%s "
                "typed=%.9f typed_seq=%llu typed_stamp_ns=%lld typed_age=%.6fs "
                "joint_state_present=%d joint_state=%.9f delta=%.9f",
                side.c_str(), finger_joint_names[fi].c_str(), typed,
                static_cast<unsigned long long>(finger_feedback.seq),
                static_cast<long long>(finger_feedback.stamp_ns), finger_age,
                global_present ? 1 : 0, joint_state, typed - joint_state);
        }
    }

    // Same q/finger values and identical world/ACM, checked against each
    // model's own collision geometry. Contact details are retained for the
    // named pairs that caused the old MoveGroup rejection.
    auto logModelCollision = [&](const char* label,
                                 const planning_scene::PlanningScene& collision_scene,
                                 const moveit::core::RobotState& state) {
        collision_detection::CollisionRequest request;
        request.contacts = true;
        request.max_contacts = 100;
        request.max_contacts_per_pair = 20;
        collision_detection::CollisionResult result;
        collision_scene.checkCollision(request, result, state, a1_verify_.base_acm);
        RCLCPP_INFO(ctx_->get_logger(), "[P2B_STATE_COLLISION] model=%s collision=%d pair_count=%zu",
                    label, result.collision ? 1 : 0, result.contacts.size());
        for (const auto& pair : result.contacts) {
            for (const auto& contact : pair.second) {
                RCLCPP_INFO(ctx_->get_logger(),
                    "[P2B_STATE_CONTACT] model=%s pair=%s<->%s pos=(%.7f,%.7f,%.7f) "
                    "normal=(%.7f,%.7f,%.7f) depth=%.9f",
                    label, pair.first.first.c_str(), pair.first.second.c_str(),
                    contact.pos.x(), contact.pos.y(), contact.pos.z(),
                    contact.normal.x(), contact.normal.y(), contact.normal.z(), contact.depth);
            }
        }
    };
    logModelCollision("criterion_mujoco_hand_aabb", *scene, same_state_criterion);
    moveit_msgs::msg::PlanningScene criterion_scene_msg;
    scene->getPlanningSceneMsg(criterion_scene_msg);
    auto live_compare_scene = std::make_shared<planning_scene::PlanningScene>(live_model);
    live_compare_scene->setPlanningSceneMsg(criterion_scene_msg);
    logModelCollision("live_robot_model_same_world_acm", *live_compare_scene, same_state_live);

    // Conservative motion scaling for the plan that may be executed.
    dual->setMaxVelocityScalingFactor(ctx_->pregrasp_vel_scale_);
    dual->setMaxAccelerationScalingFactor(ctx_->pregrasp_acc_scale_);
    dual->setPlanningTime(ctx_->p2b_planning_time_sec_);
    dual->setNumPlanningAttempts(ctx_->p2b_planning_attempts_);
    dual->setPlannerId(ctx_->p2b_planner_id_);
    dual->setStartState(p2b_start_live);

    // Save the exact name-mapped feedback state passed explicitly to MoveGroup.
    writeDiagnosticState("movegroup_start_state_requested_by_name_mapped_feedback", p2b_start_live);

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
    const auto& diagnostic_traj = plan.trajectory_.joint_trajectory;
    if (!plan.start_state_.joint_state.name.empty()) {
        moveit::core::RobotState planner_start(*live_current);
        std::size_t mapped_start_names = 0;
        for (std::size_t i = 0; i < plan.start_state_.joint_state.name.size() &&
             i < plan.start_state_.joint_state.position.size(); ++i) {
            const auto& name = plan.start_state_.joint_state.name[i];
            if (live_var_set.count(name) != 0) {
                planner_start.setVariablePosition(name, plan.start_state_.joint_state.position[i]);
                ++mapped_start_names;
            }
        }
        planner_start.update();
        writeDiagnosticState("moveit_plan_returned_start_state", planner_start);
        RCLCPP_INFO(ctx_->get_logger(), "[P2B_PLAN_START_STATE] returned_names=%zu mapped_names=%zu",
                    plan.start_state_.joint_state.name.size(), mapped_start_names);
    }
    std::ofstream path_csv(diag_dir + "p2b_joint_path.csv", std::ios::out | std::ios::trunc);
    if (path_csv && !diagnostic_traj.joint_names.empty()) {
        path_csv << "source,index,time_from_start_sec";
        for (const auto& name : diagnostic_traj.joint_names) path_csv << ",q_" << name;
        path_csv << '\n';
        for (std::size_t pi = 0; pi < diagnostic_traj.points.size(); ++pi) {
            const auto& point = diagnostic_traj.points[pi];
            path_csv << "planner_response," << pi << ','
                     << rclcpp::Duration(point.time_from_start).seconds();
            for (const double value : point.positions) path_csv << ',' << value;
            path_csv << '\n';
        }
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[P2B_PLANNER_DIAG] result=%d start_state_joint_count=%zu trajectory_names=%zu "
        "trajectory_points=%zu state_csv=%s",
        plan_code.val, plan.start_state_.joint_state.name.size(), diagnostic_traj.joint_names.size(),
        diagnostic_traj.points.size(), (diag_dir + "p2b_state_diagnostic.csv").c_str());
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
        moveit::core::RobotState st(p2b_start_criterion);
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
        for (const std::string& v : p2b_start_criterion.getVariableNames()) {
            start_vars.insert(v);
        }
        double worst = 0.0;
        for (std::size_t ji = 0; ji < traj_names.size(); ++ji) {
            if (start_vars.count(traj_names[ji]) == 0) {
                continue;
            }
            worst = std::max(worst, std::fabs(traj.points.front().positions[ji] -
                                              p2b_start_criterion.getVariablePosition(traj_names[ji])));
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
        moveit::core::RobotState st(p2b_start_criterion);
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

    // ---- 5. Arrival: a short window of PROVABLY FRESH feedback. ------------
    // Same helper and the same semantics as the descent stage: only samples
    // whose joint_states sequence number has advanced are counted, the settle
    // test is the whole window's per-joint spread, and the end-effector error is
    // read from the newest fresh sample.
    {
        const FreshArrival arrival = awaitFreshArrival(
            traj.joint_names, ctx_->pregrasp_stable_window_sec_, ctx_->pregrasp_stable_tol_rad_);
        if (!arrival.ok) {
            detail = "E_P3A_" + arrival.reason_code + ": " + arrival.detail;
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-1] %s", detail.c_str());
            return false;
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-1] 到位窗口（**新鲜反馈**）: %zu 个样本 / %.3f s（序号 %llu），"
                    "窗口内最大单关节变化 %.6f rad（%s，阈值 %.6f）",
                    arrival.samples, arrival.window_sec, arrival.last_seq,
                    arrival.worst_joint_change_rad,
                    arrival.worst_joint.empty() ? "-" : arrival.worst_joint.c_str(),
                    ctx_->pregrasp_stable_tol_rad_);

        moveit::core::RobotState arrived(ctx_->dual_arm_->getRobotModel());
        if (!freshStateFor(arrival.last_positions, arrived)) {
            detail = "E_P3A_ARRIVAL_STATE_INVALID: the fresh sample could not build a robot state";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-1] %s", detail.c_str());
            return false;
        }
        const a1v::ArmTarget got_left =
            fromEigenPose(arrived.getGlobalLinkTransform(a1_verify_.left_eef));
        const a1v::ArmTarget got_right =
            fromEigenPose(arrived.getGlobalLinkTransform(a1_verify_.right_eef));
        const a1v::PoseError el = a1v::poseError(a1_verify_.pregrasp_target.left, got_left);
        const a1v::PoseError er = a1v::poseError(a1_verify_.pregrasp_target.right, got_right);

        RCLCPP_INFO(ctx_->get_logger(),
                    "     左臂 位置误差 %.4f mm，姿态误差 %.4f°  |  右臂 位置误差 %.4f mm，"
                    "姿态误差 %.4f°",
                    el.pos_m * 1000.0, el.angle_rad * 180.0 / M_PI, er.pos_m * 1000.0,
                    er.angle_rad * 180.0 / M_PI);
        RCLCPP_INFO(ctx_->get_logger(), "     到位容差: 位置 %.1f mm / 姿态 %.2f°",
                    ctx_->pregrasp_arrival_pos_tol_m_ * 1000.0,
                    ctx_->pregrasp_arrival_angle_tol_deg_);

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

        // Observation only: the rod must not have been disturbed. This reads the
        // PLANNING SCENE object, not MuJoCo physics; the physical check belongs
        // to the descent stage's before/after comparison.
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
    // Record for the next stage: the descent may only start from a pre-grasp
    // pose that was actually reached in THIS process, by the plan verified in
    // THIS round -- not from a pose that merely looks similar.
    a1_verify_.pregrasp_reached_this_round = true;
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
            // MoveIt reports the attached body colliding with its exact parent
            // link at the closed rigid grasp. Permit only this attachment-parent
            // left/right grasp-parent pairs plus the four finger links; all
            // arm links and environment pairs remain under existing rules.
            "mj_left_hand",
            "mj_right_hand",
            "mj_left_leftfinger", "mj_left_rightfinger",
            "mj_right_leftfinger", "mj_right_rightfinger"};
        
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
            // PlanningSceneInterface can store the world placement in the
            // CollisionObject pose and leave primitive_poses[0] at identity.
            // Compose both frames; using only the primitive pose attaches the
            // rod at the world origin for A1 truth-synchronised objects.
            tf2::Transform object_frame_tf;
            tf2::Transform primitive_tf;
            tf2::fromMsg(existing_objects["aluminum_rod"].pose, object_frame_tf);
            tf2::fromMsg(existing_objects["aluminum_rod"].primitive_poses[0], primitive_tf);
            tf2::toMsg(object_frame_tf * primitive_tf, object_world_pose);
            RCLCPP_INFO(ctx_->get_logger(), "    ✓ 从 PlanningScene 获取物体位姿: [%.3f, %.3f, %.3f]",
                       object_world_pose.position.x, object_world_pose.position.y, object_world_pose.position.z);
        } else {
            if (ctx_->grasp_model_ == "ideal_weld") {
                RCLCPP_ERROR(ctx_->get_logger(),
                             "    ✗ ideal_weld 不允许从检测或硬编码位置回退：PlanningScene 真值物体缺失");
                return false;
            }
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

        if (ctx_->grasp_model_ == "ideal_weld") {
            const MujocoRodTruth physical = readMujocoRodTruth();
            if (!physical.ok) {
                RCLCPP_ERROR(ctx_->get_logger(), "    ✗ 附着前物理杆真值不可用: %s",
                             physical.detail.c_str());
                return false;
            }
            const double dx = object_world_pose.position.x - physical.position.x;
            const double dy = object_world_pose.position.y - physical.position.y;
            const double dz = object_world_pose.position.z - physical.position.z;
            const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);
            const a1t::Quat scene_q{object_world_pose.orientation.x, object_world_pose.orientation.y,
                                    object_world_pose.orientation.z, object_world_pose.orientation.w};
            const double angle_err = a1t::rotationDistanceRad(scene_q, physical.orientation);
            RCLCPP_INFO(ctx_->get_logger(),
                "[S1_ATTACH_ALIGNMENT] planning_scene=(%.6f,%.6f,%.6f) physical=(%.6f,%.6f,%.6f) "
                "position_error=%.3fmm angle_error=%.3fdeg",
                object_world_pose.position.x, object_world_pose.position.y, object_world_pose.position.z,
                physical.position.x, physical.position.y, physical.position.z, pos_err * 1000.0,
                angle_err * 180.0 / M_PI);
            if (pos_err > 0.005 || angle_err > 5.0 * M_PI / 180.0) {
                RCLCPP_ERROR(ctx_->get_logger(),
                             "    ✗ PlanningScene 与物理杆相差超过 5 mm / 5°，保留 world object 并拒绝 attach");
                return false;
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

// ===========================================================================
// A1 P3a-2b: descend from the reached pre-grasp pose to the grasp pose, then
// stop. Nothing here closes the gripper, welds, attaches or lifts.
// ===========================================================================

DualArmController::FreshArrival DualArmController::awaitFreshArrival(
    const std::vector<std::string>& joint_names, double window_sec, double settle_tol_rad)
{
    using namespace std::chrono_literals;
    FreshArrival out;
    if (joint_names.empty() || !(window_sec > 0.0)) {
        out.reason_code = "E_ARRIVAL_BAD_CONFIG";
        out.detail = "the arrival window needs joint names and a positive window length";
        return out;
    }
    // The accumulation itself lives in the pure, offline-testable FreshWindow,
    // so the "a repeated read is not a new sample" rule is the SAME code that
    // the unit tests drive. Only the polling lives here.
    a1v::FreshWindow window(joint_names, /*min_samples=*/2, window_sec, settle_tol_rad);

    // Bounded: the window plus a fixed allowance to collect it. Running out of
    // time is a failure, never a silent pass.
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(window_sec + 2.0);
    bool incomplete = false;
    while (std::chrono::steady_clock::now() < deadline) {
        DualArmCarryTask::JointFeedback sample;
        if (!ctx_->latestJointFeedback(sample)) {
            std::this_thread::sleep_for(10ms);
            continue;
        }
        std::string reject;
        window.add(sample.seq, sample.recv_steady_sec, sample.positions, reject);
        if (reject == "E_FRESH_INCOMPLETE") {
            incomplete = true;
            break;
        }
        // Keep going until a WHOLE window is quiet, not merely until one is
        // full: the first full window still contains the tail of the motion.
        // The window slides, so a stretch that settles later is found, and the
        // tolerance is unchanged.
        if (window.verdict().ok) {
            break;
        }
        std::this_thread::sleep_for(10ms);
    }
    if (incomplete) {
        out.reason_code = "E_ARRIVAL_FEEDBACK_INCOMPLETE";
        out.detail = "a fresh joint_states sample did not carry every arm joint";
        return out;
    }

    const a1v::FreshWindow::Verdict v = window.verdict();
    out.ok = v.ok;
    out.samples = v.samples;
    out.window_sec = v.span_sec;
    out.worst_joint_change_rad = v.worst_change_rad;
    out.worst_joint = v.worst_joint;
    out.last_seq = v.last_seq;
    out.last_positions = v.last_positions;
    out.reason_code = v.reason_code;
    out.detail = v.detail;
    return out;
}

bool DualArmController::freshStateFor(const std::map<std::string, double>& positions,
                                      moveit::core::RobotState& out)
{
    // The live model is used for kinematics; it and the criterion model share
    // the same joint values and differ only in hand collision geometry.
    std::vector<std::string> names;
    std::vector<double> values;
    for (const auto& kv : positions) {
        names.push_back(kv.first);
        values.push_back(kv.second);
    }
    if (names.empty()) {
        return false;
    }
    out.setVariablePositions(names, values);
    out.update();
    return true;
}

MujocoRodTruth DualArmController::readMujocoRodTruth(const std::string& service_override)
{
    using namespace std::chrono_literals;
    MujocoRodTruth out;
    const std::string& service =
        service_override.empty() ? ctx_->mujoco_get_body_state_service_ : service_override;
    auto client = ctx_->create_client<mujoco_ros_msgs::srv::GetBodyState>(service);
    const auto budget = std::chrono::duration<double>(ctx_->mujoco_truth_timeout_sec_);
    if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::milliseconds>(budget))) {
        out.detail = "the MuJoCo truth service is unavailable; the physical rod was NOT measured";
        return out;
    }
    auto request = std::make_shared<mujoco_ros_msgs::srv::GetBodyState::Request>();
    request->name = ctx_->mujoco_rod_body_name_;
    request->admin_hash = ctx_->mujoco_admin_hash_;
    auto future = client->async_send_request(request);
    if (future.wait_for(budget) != std::future_status::ready) {
        out.detail = "the MuJoCo truth call timed out; the physical rod was NOT measured";
        return out;
    }
    const auto response = future.get();
    if (!response->success) {
        out.detail = "MuJoCo refused the truth request: " + response->status_message;
        return out;
    }
    const auto& p = response->state.pose.pose;
    out.position = {p.position.x, p.position.y, p.position.z};
    out.orientation = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
    const auto& twist = response->state.twist.twist;
    out.linear_velocity = {twist.linear.x, twist.linear.y, twist.linear.z};
    out.angular_velocity = {twist.angular.x, twist.angular.y, twist.angular.z};
    if (!std::isfinite(out.position.x) || !std::isfinite(out.position.y) ||
        !std::isfinite(out.position.z) || !std::isfinite(out.orientation.x) ||
        !std::isfinite(out.orientation.y) || !std::isfinite(out.orientation.z) ||
        !std::isfinite(out.orientation.w) || !std::isfinite(out.linear_velocity.x) ||
        !std::isfinite(out.linear_velocity.y) || !std::isfinite(out.linear_velocity.z) ||
        !std::isfinite(out.angular_velocity.x) || !std::isfinite(out.angular_velocity.y) ||
        !std::isfinite(out.angular_velocity.z)) {
        out.detail = "MuJoCo returned a non-finite rod pose";
        return out;
    }
    out.ok = true;
    out.detail = "ok";
    return out;
}

namespace {

// The planning-scene pose of the rod, composed exactly as the prepare step does
// (object.pose * primitive_poses[0]; the primitive pose alone is the identity).
bool readSceneRodPose(const std::string& id, a1v::Vec3& pos, a1v::Quat& quat)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto objects = psi.getObjects(std::vector<std::string>{id});
    const auto it = objects.find(id);
    if (it == objects.end() || it->second.primitive_poses.empty()) {
        return false;
    }
    tf2::Transform object_tf;
    tf2::Transform primitive_tf;
    tf2::fromMsg(it->second.pose, object_tf);
    tf2::fromMsg(it->second.primitive_poses[0], primitive_tf);
    geometry_msgs::msg::Pose actual;
    tf2::toMsg(object_tf * primitive_tf, actual);
    pos = {actual.position.x, actual.position.y, actual.position.z};
    quat = {actual.orientation.x, actual.orientation.y, actual.orientation.z,
            actual.orientation.w};
    return true;
}

std::string rodDeltaText(const a1v::Vec3& a, const a1v::Quat& qa, const a1v::Vec3& b,
                         const a1v::Quat& qb)
{
    const double d = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) +
                               std::pow(a.z - b.z, 2));
    const double ang = a1t::rotationDistanceRad(qa, qb);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "位移 %.4f mm，转角 %.4f°", d * 1000.0,
                  ang * 180.0 / M_PI);
    return std::string(buf);
}

}  // namespace

bool DualArmController::awaitFreshOpenFingerSnapshot(
    double opened_after_steady_sec,
    const std::array<std::uint64_t, 2>& baseline_seq,
    a1v::FingerJointSnapshot& out,
    std::string& detail)
{
    using namespace std::chrono_literals;
    constexpr double kFeedbackWaitSec = 2.0;
    constexpr double kMaxSampleAgeSec = 0.50;
    constexpr double kSourceToleranceM = 1e-4;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(kFeedbackWaitSec);
    std::string last_reason = "no feedback samples received";
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        DualArmCarryTask::GripperContactFeedback left;
        DualArmCarryTask::GripperContactFeedback right;
        DualArmCarryTask::JointFeedback global;
        if (ctx_->latestGripperContact("mj_left", left) &&
            ctx_->latestGripperContact("mj_right", right) &&
            ctx_->latestJointFeedback(global)) {
            const auto to_sample = [](const DualArmCarryTask::GripperContactFeedback& src) {
                a1v::FingerFeedbackSample out_sample;
                out_sample.q1 = src.q1;
                out_sample.q2 = src.q2;
                out_sample.seq = src.seq;
                out_sample.stamp_ns = src.stamp_ns;
                out_sample.recv_steady_sec = src.recv_steady_sec;
                return out_sample;
            };
            const std::int64_t global_stamp_ns = global.stamp_valid
                ? global.stamp.nanoseconds() : 0;
            const double now_sec = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (a1v::makeFingerJointSnapshot(
                    to_sample(left), to_sample(right), global.positions, global_stamp_ns,
                    global.recv_steady_sec, opened_after_steady_sec, now_sec, baseline_seq,
                    kMaxSampleAgeSec, kSourceToleranceM, out, last_reason)) {
                const auto& names = a1v::fingerJointNames();
                RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B][finger_snapshot] source=typed_hardware global_joint_states_match=%d "
                    "seq(left=%llu,right=%llu) stamp_ns(left=%lld,right=%lld) "
                    "recv_age=(%.3f,%.3f) s tolerance=%.6f m",
                    out.global_joint_state_matched ? 1 : 0,
                    static_cast<unsigned long long>(out.source_seq[0]),
                    static_cast<unsigned long long>(out.source_seq[1]),
                    static_cast<long long>(out.source_stamp_ns[0]),
                    static_cast<long long>(out.source_stamp_ns[1]),
                    now_sec - out.source_recv_steady_sec[0],
                    now_sec - out.source_recv_steady_sec[1], kSourceToleranceM);
                for (std::size_t i = 0; i < names.size(); ++i) {
                    const auto global_it = global.positions.find(names[i]);
                    if (global_it == global.positions.end()) {
                        RCLCPP_WARN(ctx_->get_logger(),
                            "  [P3A-2B][finger_snapshot] %s measured=%.6f m "
                            "global_joint_states=<missing>; measured hardware snapshot will be "
                            "explicitly mapped into IK/criterion state",
                            names[i].c_str(), out.positions[i]);
                    } else {
                        RCLCPP_INFO(ctx_->get_logger(),
                            "  [P3A-2B][finger_snapshot] %s measured=%.6f m global=%.6f m",
                            names[i].c_str(), out.positions[i], global_it->second);
                    }
                }
                return true;
            }
        }
        rclcpp::sleep_for(20ms);
    }
    detail = "E_2B_FINGER_SNAPSHOT_INVALID: fresh measured four-finger feedback and matching "
             "global /joint_states were not available within " +
             std::to_string(kFeedbackWaitSec) + " s: " + last_reason;
    return false;
}

bool DualArmController::buildAndVerifyGraspDescent(
    const a1t::TruthPayload& truth,
    const a1v::FingerJointSnapshot& fingers,
    std::string& detail)
{
    a1_verify_.descent_plan_valid = false;
    a1_verify_.descent_waypoints.clear();

    const auto model = a1_verify_.model;
    const auto scene = a1_verify_.scene;
    const moveit::core::JointModelGroup* dual_jmg =
        model->getJointModelGroup(ctx_->dual_arm_group_);
    const moveit::core::JointModelGroup* left_jmg = model->getJointModelGroup(ctx_->left_arm_group_);
    const moveit::core::JointModelGroup* right_jmg =
        model->getJointModelGroup(ctx_->right_arm_group_);
    if (dual_jmg == nullptr || left_jmg == nullptr || right_jmg == nullptr) {
        detail = "E_2B_NO_GROUP: a joint group is missing from the criterion model";
        return false;
    }

    // (a) The schedule: same frozen truth, same generator, same height / offset /
    //     yaw definitions as the pre-grasp and grasp targets.
    a1v::GraspGeometry gg;
    gg.grasp_offset_m = ctx_->p2a_grasp_offset_m_;
    gg.eef_base_yaw_rad = ctx_->p2a_eef_base_yaw_deg_ * M_PI / 180.0;
    const double contact_probe_grasp_z_offset = ctx_->grasp_model_ == "contact_probe" ? 0.0123 : 0.0;
    gg.pre_grasp_height_m = ctx_->approach_height_ - contact_probe_grasp_z_offset;
    gg.grasp_height_m = ctx_->grasp_height_ - contact_probe_grasp_z_offset;
    double yaw = 0.0;
    const std::vector<a1v::DualTarget> schedule =
        a1v::buildDescentSchedule(truth, gg, ctx_->descent_steps_, &yaw);
    if (schedule.empty()) {
        detail = "E_2B_BAD_CONFIG: descent_steps must be positive; refusing to descend in one jump";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
        return false;
    }
    a1_verify_.descent_span_m = gg.pre_grasp_height_m - gg.grasp_height_m;
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-2B] 下降计划: 自 z=%.4f 到 z=%.4f（跨度 %.4f m），%zu 个目标（%d 段），"
                "杆 yaw=%.2f°，抓手偏移 %.4f m，末端基准 yaw=%.1f°",
                gg.pre_grasp_height_m, gg.grasp_height_m, a1_verify_.descent_span_m,
                schedule.size(), ctx_->descent_steps_, yaw * 180.0 / M_PI, gg.grasp_offset_m,
                ctx_->p2a_eef_base_yaw_deg_);

    // The schedule's own endpoints must reproduce the two targets this round
    // already validated, or the descent would be aimed at something else.
    {
        const a1v::PoseError s0l = a1v::poseError(a1_verify_.pregrasp_target.left, schedule.front().left);
        const a1v::PoseError sNl = a1v::poseError(a1_verify_.grasp_target.left, schedule.back().left);
        const a1v::PoseError s0r = a1v::poseError(a1_verify_.pregrasp_target.right, schedule.front().right);
        const a1v::PoseError sNr = a1v::poseError(a1_verify_.grasp_target.right, schedule.back().right);
        const double worst = std::max({s0l.pos_m, sNl.pos_m, s0r.pos_m, sNr.pos_m});
        const double worst_ang = std::max({s0l.angle_rad, sNl.angle_rad, s0r.angle_rad, sNr.angle_rad});
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 计划端点核对（对 P2a 的预抓取/抓取目标）: 最大位置差 %.6f mm，"
                    "最大姿态差 %.6f°",
                    worst * 1000.0, worst_ang * 180.0 / M_PI);
        if (worst > 1e-9 || worst_ang > 1e-9) {
            detail = "E_2B_SCHEDULE_ENDPOINT_MISMATCH: the descent schedule does not start at the "
                     "verified pre-grasp target or end at the verified grasp target";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
    }

    // (b) The start is the ACTUAL current state, mapped into the criterion
    //     model. Never a stored or default pose.
    auto mapInto = [&](const moveit::core::RobotState& src) {
        moveit::core::RobotState dst(model);
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
    moveit::core::RobotState live_start(*ctx_->dual_arm_->getCurrentState());
    const std::set<std::string> live_variable_names(
        live_start.getVariableNames().begin(), live_start.getVariableNames().end());
    std::map<std::string, double> measured_fingers_by_name;
    if (!a1v::mapMeasuredFingerPositions(fingers.positions, measured_fingers_by_name, detail)) {
        detail = "E_2B_FINGER_MAPPING_INVALID: " + detail;
        return false;
    }
    std::vector<std::string> finger_names;
    std::vector<double> finger_values;
    for (const auto& entry : measured_fingers_by_name) {
        if (live_variable_names.count(entry.first) == 0) {
            detail = "E_2B_FINGER_JOINT_MISSING: live model has no " + entry.first;
            return false;
        }
        const double csm_value = live_start.getVariablePosition(entry.first);
        RCLCPP_INFO(ctx_->get_logger(),
            "  [P3A-2B][finger_map] %s private_measured=%.6f m CurrentStateMonitor=%.6f m "
            "mapped=%.6f m source_seq(left=%llu,right=%llu)",
            entry.first.c_str(), entry.second, csm_value, entry.second,
            static_cast<unsigned long long>(fingers.source_seq[0]),
            static_cast<unsigned long long>(fingers.source_seq[1]));
        finger_names.push_back(entry.first);
        finger_values.push_back(entry.second);
    }
    // The arm values remain the live CurrentStateMonitor snapshot. The four
    // passive finger values are explicitly replaced by the new, source-checked
    // hardware snapshot above, so IK and criterion validation start identically.
    live_start.setVariablePositions(finger_names, finger_values);
    live_start.update();
    moveit::core::RobotState seed = mapInto(live_start);
    a1_verify_.descent_start = std::make_shared<moveit::core::RobotState>(seed);

    // IK is solved on the LIVE model and the solutions are mapped into the
    // criterion model, exactly as P2a does. The criterion model is assembled
    // from URDF + SRDF alone and therefore carries no IK solver plugin, so
    // asking it to solve would fail for reasons that have nothing to do with
    // the descent.
    auto live_model = ctx_->dual_arm_->getRobotModel();
    const moveit::core::JointModelGroup* live_left_jmg =
        live_model->getJointModelGroup(ctx_->left_arm_group_);
    const moveit::core::JointModelGroup* live_right_jmg =
        live_model->getJointModelGroup(ctx_->right_arm_group_);
    if (live_left_jmg == nullptr || live_right_jmg == nullptr) {
        detail = "E_2B_NO_LIVE_GROUP: an arm group is missing from the live model";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
        return false;
    }
    moveit::core::RobotState live_seed(live_start);
    live_seed.update();

    // Check the actual URDF finger-joint axis through runtime FK. In this
    // model finger_joint1 is +Y in its joint frame (finger_joint2 is -Y),
    // while hand orientation is the full non-commuting chain from link8.
    const double rod_yaw = a1v::quatYaw(truth.orientation);
    const Eigen::Quaterniond rod_rotation(truth.orientation.w, truth.orientation.x,
                                          truth.orientation.y, truth.orientation.z);
    const Eigen::Vector3d rod_long_axis =
        rod_rotation.normalized() * Eigen::Vector3d::UnitY();
    const a1v::Vec3 predicted_axis = a1v::fingerClosureAxisForRodYaw(rod_yaw);
    const Eigen::Vector3d predicted(predicted_axis.x, predicted_axis.y, predicted_axis.z);
    for (const std::string& arm : {std::string("mj_left"), std::string("mj_right")}) {
        const std::string joint_name = arm + "_finger_joint1";
        const auto* joint = dynamic_cast<const moveit::core::PrismaticJointModel*>(
            live_model->getJointModel(joint_name));
        if (joint == nullptr || joint->getChildLinkModel() == nullptr) {
            detail = "E_2B_FINGER_AXIS_UNAVAILABLE: runtime model has no prismatic " + joint_name;
            return false;
        }
        const Eigen::Isometry3d& child_tf =
            live_seed.getGlobalLinkTransform(joint->getChildLinkModel()->getName());
        const Eigen::Vector3d actual_axis = (child_tf.linear() * joint->getAxis()).normalized();
        const double axis_error = std::acos(std::clamp(
            std::fabs(actual_axis.dot(rod_long_axis)), 0.0, 1.0)) * 180.0 / M_PI;
        const double fk_vs_chain = std::min((actual_axis - predicted).norm(),
                                            (actual_axis + predicted).norm());
        RCLCPP_INFO(ctx_->get_logger(),
            "  [P3A-2B][finger_axis_fk] arm=%s joint=%s local_axis=(%.1f,%.1f,%.1f) "
            "world_axis=(%.6f,%.6f,%.6f) rod_localY_world=(%.6f,%.6f,%.6f) "
            "unoriented_angle_to_rod_long=%.3f deg chain_error=%.6g",
            arm.c_str(), joint_name.c_str(), joint->getAxis().x(), joint->getAxis().y(),
            joint->getAxis().z(), actual_axis.x(), actual_axis.y(), actual_axis.z(),
            rod_long_axis.x(), rod_long_axis.y(), rod_long_axis.z(), axis_error, fk_vs_chain);
    }
    std::vector<moveit::core::RobotState> live_waypoints;

    auto mapLiveInto = [&](const moveit::core::RobotState& src) {
        moveit::core::RobotState dst(model);
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

    // The start must still be the pre-grasp solution this round executed, not
    // merely somewhere plausible. Only the ARM joints are compared: the descent
    // is run after deliberately opening the grippers, so the finger variables
    // have moved on purpose and must not count as a start mismatch.
    {
        double worst = 0.0;
        std::string worst_joint;
        for (const std::string& name : dual_jmg->getVariableNames()) {
            const auto it = a1_verify_.pregrasp_joints.find(name);
            if (it == a1_verify_.pregrasp_joints.end()) {
                continue;
            }
            const double d = std::fabs(seed.getVariablePosition(name) - it->second);
            if (d > worst) {
                worst = d;
                worst_joint = name;
            }
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 下降起点 = 本次实际状态；与已到位预抓取解的最大**手臂**关节差 "
                    "%.6f rad（%s）；夹爪指关节因本轮开爪而变化，不参与此判据",
                    worst, worst_joint.empty() ? "-" : worst_joint.c_str());
        if (worst > 0.02) {
            detail = "E_2B_START_NOT_PREGRASP: the current state is not this round's reached "
                     "pre-grasp solution (max arm joint delta " + std::to_string(worst) + " rad at " +
                     worst_joint + ")";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
    }

    // (c) Combined-state IK. Each target is solved ON THE SAME RobotState, seeded
    //     by the previous solution, so the two arms are solved as one
    //     configuration and the path stays continuous. Two independent
    //     single-arm trajectories are deliberately NOT produced.
    const double ang_tol = ctx_->p2a_ik_angle_tol_deg_ * M_PI / 180.0;
    for (std::size_t i = 0; i < schedule.size(); ++i) {
        // KDL restarts randomly inside its timeout, so a single call can fail
        // even from a seed that already satisfies the target. Several attempts
        // are made and the one that changes the arm configuration least is
        // kept, which is what keeps the descent continuous rather than jumping
        // between IK branches.
        std::vector<double> seed_q, l3, u3;
        std::vector<std::string> vars;
        collectArmJoints(live_seed, live_left_jmg, seed_q, l3, u3, vars);
        collectArmJoints(live_seed, live_right_jmg, seed_q, l3, u3, vars);

        moveit::core::RobotState best(live_seed);
        bool have = false;
        double best_delta = std::numeric_limits<double>::infinity();
        std::string last_failure;
        for (int attempt = 0; attempt < ctx_->p2a_ik_attempts_; ++attempt) {
            moveit::core::RobotState trial(live_seed);
            const bool solved =
                trial.setFromIK(live_left_jmg, toEigenPose(schedule[i].left),
                                ctx_->p2a_ik_timeout_sec_) &&
                trial.setFromIK(live_right_jmg, toEigenPose(schedule[i].right),
                                ctx_->p2a_ik_timeout_sec_);
            if (!solved) {
                last_failure = "no combined solution";
                continue;
            }
            trial.update();

            std::vector<double> q, lo, hi;
            std::vector<std::string> nm;
            collectArmJoints(trial, live_left_jmg, q, lo, hi, nm);
            collectArmJoints(trial, live_right_jmg, q, lo, hi, nm);
            const a1v::JointLimitCheck lim = a1v::checkJointLimits(q, lo, hi, nm);
            if (!lim.ok) {
                last_failure = "joint " + lim.offending_name + " outside limits";
                continue;
            }
            const a1v::PoseError el = a1v::poseError(
                schedule[i].left, fromEigenPose(trial.getGlobalLinkTransform(a1_verify_.left_eef)));
            const a1v::PoseError er = a1v::poseError(
                schedule[i].right, fromEigenPose(trial.getGlobalLinkTransform(a1_verify_.right_eef)));
            if (el.pos_m > ctx_->p2a_ik_pos_tol_m_ || er.pos_m > ctx_->p2a_ik_pos_tol_m_ ||
                el.angle_rad > ang_tol || er.angle_rad > ang_tol) {
                last_failure = "FK residual left " + std::to_string(el.pos_m * 1000.0) +
                               " mm, right " + std::to_string(er.pos_m * 1000.0) + " mm";
                continue;
            }
            const double d = a1v::maxAbsJointDelta(seed_q, q);
            if (!have || d < best_delta) {
                have = true;
                best_delta = d;
                best = trial;
            }
        }
        if (!have) {
            detail = "E_2B_IK_NO_SOLUTION at step " + std::to_string(i) + " (z=" +
                     std::to_string(schedule[i].left.position.z) + "): " +
                     (last_failure.empty() ? "no attempt succeeded" : last_failure) + " (" +
                     std::to_string(ctx_->p2a_ik_attempts_) + " attempts)";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        // The criterion-model twin is what the collision checks and the
        // straightness report use; the live-model state is what the dispatched
        // trajectory is built from. The kinematics are identical -- the
        // criterion model only differs in hand collision geometry.
        a1_verify_.descent_waypoints.push_back(
            std::make_shared<moveit::core::RobotState>(mapLiveInto(best)));
        live_waypoints.push_back(best);
        live_seed = best;
    }
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-2B] 组合状态 IK: %zu 个航点全部求解成功（同一 RobotState 内两臂联合，"
                "非两条独立单臂轨迹）",
                a1_verify_.descent_waypoints.size());

    // The arm-joint naming, fixed once. collectArmJoints APPENDS, so a shared
    // name vector would grow with every call and stop matching the values.
    std::vector<std::string> arm_names;
    {
        std::vector<double> v, lo, hi;
        collectArmJoints(*a1_verify_.descent_waypoints.front(), left_jmg, v, lo, hi, arm_names);
        collectArmJoints(*a1_verify_.descent_waypoints.front(), right_jmg, v, lo, hi, arm_names);
    }
    auto armValues = [&](const moveit::core::RobotState& st) {
        std::vector<double> v, lo, hi;
        std::vector<std::string> discard;
        collectArmJoints(st, left_jmg, v, lo, hi, discard);
        collectArmJoints(st, right_jmg, v, lo, hi, discard);
        return v;
    };

    // Continuity between adjacent solutions.
    double max_jump = 0.0;
    std::size_t worst_jump_index = 0;
    for (std::size_t i = 1; i < a1_verify_.descent_waypoints.size(); ++i) {
        const std::vector<double> a = armValues(*a1_verify_.descent_waypoints[i - 1]);
        const std::vector<double> b = armValues(*a1_verify_.descent_waypoints[i]);
        if (a.size() != arm_names.size() || b.size() != arm_names.size()) {
            detail = "E_2B_INTERNAL_DIM: arm joint count mismatch";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        const double d = a1v::maxAbsJointDelta(a, b);
        if (d > max_jump) {
            max_jump = d;
            worst_jump_index = i;
        }
    }
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-2B] 相邻航点最大单关节变化 %.4f rad（第 %zu 段，阈值 %.4f）", max_jump,
                worst_jump_index, ctx_->p2a_max_joint_jump_rad_);
    if (max_jump > ctx_->p2a_max_joint_jump_rad_) {
        detail = "E_2B_JOINT_JUMP: adjacent descent waypoints differ by " + std::to_string(max_jump) +
                 " rad, above the configured gate";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
        return false;
    }

    // (d) Straightness of the planned path, measured by FK. This is what the
    //     report may call "along world Z" -- and only to this measured extent.
    {
        std::vector<a1v::ArmTarget> pl;
        std::vector<a1v::ArmTarget> pr;
        for (const auto& wp : a1_verify_.descent_waypoints) {
            pl.push_back(fromEigenPose(wp->getGlobalLinkTransform(a1_verify_.left_eef)));
            pr.push_back(fromEigenPose(wp->getGlobalLinkTransform(a1_verify_.right_eef)));
        }
        const a1v::StraightDescentDeviation dl = a1v::straightDescentDeviation(pl);
        const a1v::StraightDescentDeviation dr = a1v::straightDescentDeviation(pr);
        a1_verify_.descent_deviation = dl;
        const double ang_lim = ctx_->descent_max_angle_dev_deg_ * M_PI / 180.0;
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 计划竖直性（FK 实测，对首点）: 左 max XY %.4f mm / max 姿态 %.4f°；"
                    "右 max XY %.4f mm / max 姿态 %.4f°（阈值 XY %.4f mm / 姿态 %.4f°）",
                    dl.max_xy_m * 1000.0, dl.max_angle_rad * 180.0 / M_PI, dr.max_xy_m * 1000.0,
                    dr.max_angle_rad * 180.0 / M_PI, ctx_->descent_max_xy_dev_m_ * 1000.0,
                    ctx_->descent_max_angle_dev_deg_);
        if (!dl.ok || !dr.ok) {
            detail = "E_2B_DEVIATION_UNMEASURABLE: " + dl.reason_code;
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        if (dl.max_xy_m > ctx_->descent_max_xy_dev_m_ || dr.max_xy_m > ctx_->descent_max_xy_dev_m_ ||
            dl.max_angle_rad > ang_lim || dr.max_angle_rad > ang_lim) {
            detail = "E_2B_NOT_STRAIGHT: the descent departs from a straight vertical line by more "
                     "than the configured gate (left " + std::to_string(dl.max_xy_m * 1000.0) +
                     " mm, right " + std::to_string(dr.max_xy_m * 1000.0) + " mm)";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
    }

    // (e) Time parameterisation FIRST, so that everything checked below is the
    //     trajectory that would actually be dispatched -- not the pre-TOTG
    //     waypoints, which are a different message with a different point count.
    auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(
        live_model, ctx_->dual_arm_group_);
    for (const auto& wp : live_waypoints) {
        traj->addSuffixWayPoint(wp, 0.0);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(*traj, ctx_->pregrasp_vel_scale_, ctx_->pregrasp_acc_scale_)) {
        detail = "E_2B_TIME_PARAMETERISATION_FAILED";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
        return false;
    }
    traj->getRobotTrajectoryMsg(a1_verify_.descent_plan.trajectory_);
    a1_verify_.descent_plan.planning_time_ = 0.0;

    const auto& jt = a1_verify_.descent_plan.trajectory_.joint_trajectory;
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-2B] TOTG: %zu 轨迹点（IK 航点 %zu），时长 %.3f s —— 以下所有检查针对"
                "**这条最终轨迹**，不再针对 TOTG 之前的航点",
                jt.points.size(), a1_verify_.descent_waypoints.size(),
                rclcpp::Duration(jt.points.back().time_from_start).seconds());

    const double step = ctx_->p2b_max_joint_step_rad_;
    constexpr std::size_t kMaxSubdivisions = 2000;

    // Test-only negative injection, applied to the FINAL message so that the
    // checks below are the ones under test. Empty by default.
    bool injected_obstacle = false;
    if (ctx_->descent_negative_final_ == "position") {
        a1_verify_.descent_plan.trajectory_.joint_trajectory.points.back().positions[0] += 100.0;
        RCLCPP_WARN(ctx_->get_logger(),
                    "  [P3A-2B][负例-注入] 已把最终轨迹末点的第 1 个关节位置 +100 rad"
                    "（仅验证用；不执行任何运动）");
    }

    // (f) Structure of the FINAL message.
    {
        const a1v::JointNameCheck nc =
            a1v::validateJointNames(jt.joint_names, dual_jmg->getVariableNames());
        if (!nc.ok) {
            detail = "E_2B_JOINT_NAMES: " + nc.detail;
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        if (jt.points.empty()) {
            detail = "E_2B_EMPTY_TRAJECTORY: the final trajectory carries no points";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        for (std::size_t pi = 0; pi < jt.points.size(); ++pi) {
            const auto& pos = jt.points[pi].positions;
            if (pos.size() != jt.joint_names.size()) {
                detail = "E_2B_DIM_MISMATCH: final point " + std::to_string(pi) + " has " +
                         std::to_string(pos.size()) + " values for " +
                         std::to_string(jt.joint_names.size()) + " names";
                RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
                return false;
            }
            for (std::size_t ji = 0; ji < pos.size(); ++ji) {
                if (!std::isfinite(pos[ji])) {
                    detail = "E_2B_NOT_FINITE: final point " + std::to_string(pi) + ", joint " +
                             jt.joint_names[ji];
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
                    return false;
                }
            }
        }
        std::vector<double> times;
        times.reserve(jt.points.size());
        for (const auto& pt : jt.points) {
            times.push_back(rclcpp::Duration(pt.time_from_start).seconds());
        }
        const a1v::TimeCheck tc = a1v::validateTimeFromStart(times);
        if (!tc.ok) {
            detail = "E_2B_TIME: " + tc.detail;
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
    }

    // (g) The final trajectory, state by state, in the criterion model. The base
    //     state carries THIS round's real finger values -- the grippers are open
    //     by now and that is the configuration that will be dispatched -- so the
    //     collision picture includes the actually-open fingers.
    moveit::core::RobotState final_base(mapLiveInto(live_start));
    final_base.update();
    {
        double worst_finger = 0.0;
        for (const std::string& v : final_base.getVariableNames()) {
            if (v.find("finger_joint") != std::string::npos) {
                worst_finger = std::max(worst_finger, final_base.getVariablePosition(v));
            }
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 最终轨迹检查基线: 判据模型 + 本轮**真实打开的指关节**"
                    "（最大指位移 %.4f m）",
                    worst_finger);
    }
    auto finalStateOf = [&](const std::vector<double>& pos) {
        moveit::core::RobotState st(final_base);
        st.setVariablePositions(jt.joint_names, pos);
        st.update();
        return st;
    };

    if (ctx_->descent_negative_final_ == "collision") {
        // Placed on the FINAL point's own end-effector link, so the injected
        // obstacle is guaranteed to overlap the robot: a box parked somewhere
        // "near the grasp" would prove nothing if the hands were not there.
        const moveit::core::RobotState last = finalStateOf(jt.points.back().positions);
        const Eigen::Isometry3d eef = last.getGlobalLinkTransform(a1_verify_.left_eef);
        moveit_msgs::msg::CollisionObject obstacle;
        obstacle.header.frame_id = scene->getPlanningFrame();
        obstacle.id = "a1_descent_final_obstacle_injected";
        obstacle.primitives.clear();
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {0.04, 0.04, 0.04};
        obstacle.primitives.push_back(box);
        geometry_msgs::msg::Pose op;
        op.position.x = eef.translation().x();
        op.position.y = eef.translation().y();
        op.position.z = eef.translation().z();
        op.orientation.w = 1.0;
        obstacle.primitive_poses.push_back(op);
        obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
        scene->processCollisionObjectMsg(obstacle);
        injected_obstacle = true;
        RCLCPP_WARN(ctx_->get_logger(),
                    "  [P3A-2B][负例-注入] 已在最终轨迹**末点**的左末端(%.3f, %.3f, %.3f) 插入"
                    " 4 cm 验证障碍（仅验证用；不执行任何运动）",
                    op.position.x, op.position.y, op.position.z);
    }

    std::size_t final_checked = 0;
    ContactTally final_tally;
    for (std::size_t pi = 0; pi < jt.points.size(); ++pi) {
        const moveit::core::RobotState st = finalStateOf(jt.points[pi].positions);
        std::vector<double> q, lo, hi;
        std::vector<std::string> nm;
        collectArmJoints(st, left_jmg, q, lo, hi, nm);
        collectArmJoints(st, right_jmg, q, lo, hi, nm);
        const a1v::JointLimitCheck lim = a1v::checkJointLimits(q, lo, hi, nm);
        if (!lim.ok) {
            detail = "E_2B_FINAL_LIMIT at point " + std::to_string(pi) + ": joint " +
                     lim.offending_name + " = " + std::to_string(lim.value);
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        const ContactTally t = tallyContactsIn(scene, model, st, a1_verify_.base_acm,
                                               a1_verify_.rod_id, a1_verify_.support_id);
        ++final_checked;
        if (t.total() > 0) {
            final_tally.rod += t.rod;
            final_tally.support += t.support;
            final_tally.dual_arm += t.dual_arm;
            final_tally.self += t.self;
            final_tally.world += t.world;
            if (final_tally.examples.empty()) {
                final_tally.examples = t.examples;
            }
        }
    }

    // Segments of the FINAL trajectory, interpolated between its own points.
    std::size_t final_subdivisions = 0;
    for (std::size_t pi = 0; pi + 1 < jt.points.size(); ++pi) {
        const auto& a = jt.points[pi].positions;
        const auto& b = jt.points[pi + 1].positions;
        const std::size_t n = a1v::subdivisionsFor(a1v::maxAbsJointDelta(a, b), step);
        if (n > kMaxSubdivisions) {
            detail = "E_2B_FINAL_SEGMENT_TOO_LONG: final segment " + std::to_string(pi) +
                     " needs " + std::to_string(n) + " subdivisions; refusing to skip any of it";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        final_subdivisions += n;
        for (std::size_t k = 0; k < n; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(n);
            const std::vector<double> pos = a1v::interpolateJoints(a, b, t);
            const moveit::core::RobotState st = finalStateOf(pos);
            const ContactTally tly = tallyContactsIn(scene, model, st, a1_verify_.base_acm,
                                                     a1_verify_.rod_id, a1_verify_.support_id);
            ++final_checked;
            if (tly.total() > 0) {
                final_tally.rod += tly.rod;
                final_tally.support += tly.support;
                final_tally.dual_arm += tly.dual_arm;
                final_tally.self += tly.self;
                final_tally.world += tly.world;
                if (final_tally.examples.empty()) {
                    final_tally.examples = tly.examples;
                }
            }
        }
    }
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-2B] 最终轨迹碰撞检查（严格 ACM，未放行指-杆）: %zu 个状态"
                "（含 %zu 个航点、%zu 段细分），碰撞 %zu 处",
                final_checked, jt.points.size(), final_subdivisions, final_tally.total());
    if (final_tally.total() > 0) {
        detail = "E_2B_FINAL_PATH_COLLISION: " + describeTally(final_tally) +
                 " -- stopping and reporting instead of exempting the contact";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
        return false;
    }

    // (i) Endpoint FK from the LAST point of the FINAL message, not from the
    //     pre-TOTG IK waypoints.
    {
        const moveit::core::RobotState end = finalStateOf(jt.points.back().positions);
        const a1v::PoseError fl = a1v::poseError(
            a1_verify_.grasp_target.left, fromEigenPose(end.getGlobalLinkTransform(a1_verify_.left_eef)));
        const a1v::PoseError fr = a1v::poseError(
            a1_verify_.grasp_target.right, fromEigenPose(end.getGlobalLinkTransform(a1_verify_.right_eef)));
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 最终轨迹**末点** FK 残差（对抓取目标，取自最终消息）: "
                    "左 %.4f mm/%.4f°，右 %.4f mm/%.4f°",
                    fl.pos_m * 1000.0, fl.angle_rad * 180.0 / M_PI, fr.pos_m * 1000.0,
                    fr.angle_rad * 180.0 / M_PI);
        if (fl.pos_m > ctx_->p2a_ik_pos_tol_m_ || fr.pos_m > ctx_->p2a_ik_pos_tol_m_ ||
            fl.angle_rad > ang_tol || fr.angle_rad > ang_tol) {
            detail = "E_2B_ENDPOINT_MISMATCH: the final point does not reproduce the grasp target";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
    }

    // (j) Derivative fields against the LIVE model's limits. A trajectory that
    //     has not passed every check above never becomes valid, so nothing is
    //     dispatched on a partially verified message.
    {
        std::vector<double> vel_limits, acc_limits;
        std::vector<bool> vel_bounded, acc_bounded;
        for (const std::string& j : jt.joint_names) {
            double vmax = 0.0, amax = 0.0;
            bool vb = false, ab = false;
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
        double vel_worst = 0.0;
        double acc_worst = 0.0;
        std::size_t vel_points = 0;
        std::size_t acc_points = 0;
        for (std::size_t pi = 0; pi < jt.points.size(); ++pi) {
            if (!jt.points[pi].velocities.empty()) {
                ++vel_points;
                const a1v::DerivativeCheck c = a1v::checkDerivativeField(
                    jt.points[pi].velocities, vel_limits, vel_bounded, jt.joint_names);
                if (!c.ok) {
                    detail = "E_2B_VELOCITY_INVALID at point " + std::to_string(pi) + ": " +
                             c.reason_code + " (" + c.detail + ")";
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
                    return false;
                }
                vel_worst = std::max(vel_worst, c.worst_ratio);
            }
            if (!jt.points[pi].accelerations.empty()) {
                ++acc_points;
                const a1v::DerivativeCheck c = a1v::checkDerivativeField(
                    jt.points[pi].accelerations, acc_limits, acc_bounded, jt.joint_names);
                if (!c.ok) {
                    detail = "E_2B_ACCELERATION_INVALID at point " + std::to_string(pi) + ": " +
                             c.reason_code + " (" + c.detail + ")";
                    RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
                    return false;
                }
                acc_worst = std::max(acc_worst, c.worst_ratio);
            }
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 最终轨迹复核: %zu 关节名唯一完整；末点 t=%.3f s；"
                    "速度字段 %zu/%zu 点（最大占用比 %.4f）；加速度字段 %zu/%zu 点"
                    "（最大占用比 %.4f）",
                    jt.joint_names.size(),
                    rclcpp::Duration(jt.points.back().time_from_start).seconds(), vel_points,
                    jt.points.size(), vel_worst, acc_points, jt.points.size(), acc_worst);
    }

    if (injected_obstacle) {
        // The negative example must not leak into the criterion scene.
        moveit_msgs::msg::CollisionObject rm;
        rm.id = "a1_descent_final_obstacle_injected";
        rm.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        rm.header.frame_id = scene->getPlanningFrame();
        scene->processCollisionObjectMsg(rm);
        RCLCPP_INFO(ctx_->get_logger(), "  [P3A-2B][负例-注入] 验证障碍已移除");
    }

    a1_verify_.descent_plan_valid = true;
    detail = "DESCENT_VERIFIED";
    return true;
}

bool DualArmController::runGraspContact(const a1t::TruthPayload& truth, std::string& detail)
{
    using namespace std::chrono_literals;
    constexpr int kWindowSamples = 16;
    constexpr double kGapSpreadTolM = 0.0002;
    constexpr double kArmSpreadTolRad = 0.002;
    constexpr double kArmStableWindowSec = 0.30;
    constexpr double kEefPositionTolM = 0.005;
    constexpr double kEefAngleTolRad = 2.0 * M_PI / 180.0;
    RCLCPP_INFO(ctx_->get_logger(),
        "[P3A-2C] 接触窗口预设: 每手16个新序号样本(约0.30 s)，总gap整窗spread<=%.4f m；"
        "接触后关节新反馈窗>=%.2f s、逐关节range<=%.4f rad；末端相对原抓取目标<=%.3f m/%.2f deg；"
        "杆真值复用下降门槛 %.4f m / %.2f deg",
        kGapSpreadTolM, kArmStableWindowSec, kArmSpreadTolRad, kEefPositionTolM,
        kEefAngleTolRad * 180.0 / M_PI, ctx_->descent_truth_pos_tol_m_,
        ctx_->descent_truth_angle_tol_deg_);

    const MujocoRodTruth before = readMujocoRodTruth();
    auto quaternionDistance = [](const a1t::Quat& a, const a1t::Quat& b) {
        const double an = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);
        const double bn = std::sqrt(b.x*b.x + b.y*b.y + b.z*b.z + b.w*b.w);
        if (!(an > 1e-12) || !(bn > 1e-12)) return std::numeric_limits<double>::infinity();
        const double dot = std::abs((a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w) / (an*bn));
        return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
    };
    const a1v::Vec3 frozen{truth.position.x, truth.position.y, truth.position.z};
    auto positionDistance = [](const a1v::Vec3& a, const a1v::Vec3& b) {
        const double x = a.x-b.x, y = a.y-b.y, z = a.z-b.z;
        return std::sqrt(x*x+y*y+z*z);
    };
    const a1t::Quat frozen_q{truth.orientation.x, truth.orientation.y,
                             truth.orientation.z, truth.orientation.w};
    if (!before.ok) {
        detail = "E_P3A2C_TRUTH_BEFORE_UNAVAILABLE: " + before.detail;
        RCLCPP_ERROR(ctx_->get_logger(), "%s", detail.c_str());
        return false;
    }
    const double before_pos_err = positionDistance(before.position, frozen);
    const double before_ang_err = quaternionDistance(before.orientation, frozen_q);
    if (before_pos_err > ctx_->descent_truth_pos_tol_m_ ||
        before_ang_err > ctx_->descent_truth_angle_tol_deg_ * M_PI / 180.0) {
        detail = "E_P3A2C_TRUTH_REFERENCE_STALE";
        RCLCPP_ERROR(ctx_->get_logger(),
            "[P3A-2C] 闭合前杆真值偏离冻结参考: pos=%.6f m angle=%.4f deg; 停止，不闭合",
            before_pos_err, before_ang_err * 180.0 / M_PI);
        return false;
    }

    const std::string arm_names[2] = {"mj_left", "mj_right"};
    DualArmCarryTask::JointFeedback arm_before;
    if (!ctx_->latestJointFeedback(arm_before)) {
        detail = "E_P3A2C_ARM_FEEDBACK_UNAVAILABLE_BEFORE";
        return false;
    }

    // Separate natural arm drift from motion that occurs during the contact
    // action. Observe a short fixed hold before sending either Grasp goal.
    constexpr auto kPreContactHold = 300ms;
    constexpr auto kArmSamplePeriod = 20ms;
    DualArmCarryTask::JointFeedback arm_contact_start = arm_before;
    unsigned long long previous_arm_seq = arm_before.seq;
    int pre_hold_fresh_samples = 0;
    double pre_hold_max_delta = 0.0;
    std::string pre_hold_worst_joint = "-";
    const auto hold_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() - hold_start < kPreContactHold) {
        std::this_thread::sleep_for(kArmSamplePeriod);
        DualArmCarryTask::JointFeedback sample;
        if (!ctx_->latestJointFeedback(sample) || sample.seq <= previous_arm_seq) continue;
        previous_arm_seq = sample.seq;
        ++pre_hold_fresh_samples;
        arm_contact_start = sample;
        for (int side = 0; side < 2; ++side) {
            for (int joint = 1; joint <= 7; ++joint) {
                const std::string name = arm_names[side] + "_joint" + std::to_string(joint);
                const auto initial = arm_before.positions.find(name);
                const auto current = sample.positions.find(name);
                if (initial == arm_before.positions.end() || current == sample.positions.end()) continue;
                const double delta = std::abs(current->second - initial->second);
                if (delta > pre_hold_max_delta) {
                    pre_hold_max_delta = delta;
                    pre_hold_worst_joint = name;
                }
            }
        }
    }
    if (pre_hold_fresh_samples < 5) {
        detail = "E_P3A2C_PRECONTACT_ARM_HOLD_FEEDBACK_STALE";
        RCLCPP_ERROR(ctx_->get_logger(),
            "[P3A2C_ARM_BASELINE] only %d fresh joint samples during %.3f s pre-contact hold; no Grasp goal sent",
            pre_hold_fresh_samples,
            std::chrono::duration<double>(std::chrono::steady_clock::now() - hold_start).count());
        return false;
    }

    auto handTransforms = [&](const DualArmCarryTask::JointFeedback& feedback,
                              Eigen::Isometry3d& left, Eigen::Isometry3d& right) {
        if (!a1_verify_.model) return false;
        moveit::core::RobotState state(a1_verify_.model);
        for (int side = 0; side < 2; ++side) {
            for (int joint = 1; joint <= 7; ++joint) {
                const std::string name = arm_names[side] + "_joint" + std::to_string(joint);
                const auto it = feedback.positions.find(name);
                if (it == feedback.positions.end() || !std::isfinite(it->second)) return false;
                state.setVariablePosition(name, it->second);
            }
        }
        state.update();
        left = state.getGlobalLinkTransform(a1_verify_.left_eef);
        right = state.getGlobalLinkTransform(a1_verify_.right_eef);
        return true;
    };
    auto poseDelta = [](const Eigen::Isometry3d& before_pose,
                        const Eigen::Isometry3d& after_pose) {
        const double position_m = (after_pose.translation() - before_pose.translation()).norm();
        const Eigen::AngleAxisd rotation(before_pose.linear().transpose() * after_pose.linear());
        return std::pair<double, double>{position_m, rotation.angle()};
    };
    Eigen::Isometry3d left_before_fk, right_before_fk, left_contact_fk, right_contact_fk;
    const bool fk_baseline_valid = handTransforms(arm_before, left_before_fk, right_before_fk) &&
                                   handTransforms(arm_contact_start, left_contact_fk, right_contact_fk);
    double baseline_left_pos = std::numeric_limits<double>::quiet_NaN();
    double baseline_left_ang = std::numeric_limits<double>::quiet_NaN();
    double baseline_right_pos = std::numeric_limits<double>::quiet_NaN();
    double baseline_right_ang = std::numeric_limits<double>::quiet_NaN();
    if (fk_baseline_valid) {
        const auto left = poseDelta(left_before_fk, left_contact_fk);
        const auto right = poseDelta(right_before_fk, right_contact_fk);
        baseline_left_pos = left.first; baseline_left_ang = left.second;
        baseline_right_pos = right.first; baseline_right_ang = right.second;
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[P3A2C_ARM_BASELINE] hold=%.3f s fresh_samples=%d seq=%llu->%llu max_joint_excursion=%.6f rad joint=%s FK_valid=%d left_dpos=%.6f m left_drot=%.5f deg right_dpos=%.6f m right_drot=%.5f deg",
        std::chrono::duration<double>(std::chrono::steady_clock::now() - hold_start).count(),
        pre_hold_fresh_samples, arm_before.seq, arm_contact_start.seq,
        pre_hold_max_delta, pre_hold_worst_joint.c_str(), fk_baseline_valid ? 1 : 0,
        baseline_left_pos, baseline_left_ang * 180.0 / M_PI,
        baseline_right_pos, baseline_right_ang * 180.0 / M_PI);

    RCLCPP_INFO(ctx_->get_logger(),
                "[P3A-2C] contact action protocol start; grasp action only; weld=0 attach=0");
    const bool action_ok = ctx_->grippers_->controlDualGrippersContact();

    a1g::BilateralContactWindow windows[2] = {
        a1g::BilateralContactWindow(kWindowSamples, kGapSpreadTolM),
        a1g::BilateralContactWindow(kWindowSamples, kGapSpreadTolM)};
    bool contact_ok = false;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (action_ok && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        for (int i = 0; i < 2; ++i) {
            DualArmCarryTask::GripperContactFeedback fb;
            if (!ctx_->latestGripperContact(arm_names[i], fb)) continue;
            windows[i].update(fb.seq, fb.q1, fb.q2, fb.left_contact, fb.right_contact);
        }
        contact_ok = windows[0].confirmed() && windows[1].confirmed();
        if (contact_ok) break;
        std::this_thread::sleep_for(20ms);
    }

    // Arm stability is a fresh 0.30 s observation after both action results;
    // cumulative displacement from the original snapshot remains a separate
    // reported metric and is not compared to this short-window threshold.
    a1g::JointStabilityWindow arm_window(kArmStableWindowSec, kArmSpreadTolRad);
    DualArmCarryTask::JointFeedback arm_after;
    bool arm_after_valid = false;
    const auto arm_window_start = std::chrono::steady_clock::now();
    DualArmCarryTask::JointFeedback arm_window_fence;
    const bool have_arm_window_fence = ctx_->latestJointFeedback(arm_window_fence);
    unsigned long long arm_window_seq = have_arm_window_fence ? arm_window_fence.seq : 0;
    while (rclcpp::ok() && std::chrono::steady_clock::now() - arm_window_start < 360ms) {
        DualArmCarryTask::JointFeedback sample;
        if (ctx_->latestJointFeedback(sample) && sample.seq > arm_window_seq) {
            std::vector<double> positions;
            positions.reserve(14);
            bool complete = true;
            for (int side = 0; side < 2; ++side) {
                for (int joint = 1; joint <= 7; ++joint) {
                    const std::string name = arm_names[side] + "_joint" + std::to_string(joint);
                    const auto it = sample.positions.find(name);
                    if (it == sample.positions.end() || !std::isfinite(it->second)) {
                        complete = false;
                        break;
                    }
                    positions.push_back(it->second);
                }
                if (!complete) break;
            }
            if (complete && arm_window.update(sample.seq, sample.recv_steady_sec, positions)) {
                arm_window_seq = sample.seq;
                arm_after = sample;
                arm_after_valid = true;
            }
        }
        std::this_thread::sleep_for(10ms);
    }
    double arm_max_delta = 0.0;
    double contact_arm_max_delta = 0.0;
    std::string worst_joint = "-";
    std::string contact_worst_joint = "-";
    bool arm_feedback_complete = arm_after_valid && arm_after.seq > arm_before.seq;
    if (arm_after_valid) {
        for (int side = 0; side < 2; ++side) {
            for (int joint = 1; joint <= 7; ++joint) {
                const std::string name = arm_names[side] + "_joint" + std::to_string(joint);
                const auto b = arm_before.positions.find(name);
                const auto a = arm_after.positions.find(name);
                if (b == arm_before.positions.end() || a == arm_after.positions.end() ||
                    !std::isfinite(b->second) || !std::isfinite(a->second)) {
                    arm_feedback_complete = false;
                    continue;
                }
                const double delta = std::abs(a->second - b->second);
                if (delta > arm_max_delta) { arm_max_delta = delta; worst_joint = name; }
                const auto c = arm_contact_start.positions.find(name);
                if (c != arm_contact_start.positions.end() && std::isfinite(c->second)) {
                    const double contact_delta = std::abs(a->second - c->second);
                    if (contact_delta > contact_arm_max_delta) {
                        contact_arm_max_delta = contact_delta;
                        contact_worst_joint = name;
                    }
                }
            }
        }
    }
    Eigen::Isometry3d left_after_fk, right_after_fk;
    const bool fk_after_valid = arm_after_valid &&
        handTransforms(arm_after, left_after_fk, right_after_fk);
    double contact_left_pos = std::numeric_limits<double>::quiet_NaN();
    double contact_left_ang = std::numeric_limits<double>::quiet_NaN();
    double contact_right_pos = std::numeric_limits<double>::quiet_NaN();
    double contact_right_ang = std::numeric_limits<double>::quiet_NaN();
    if (fk_after_valid) {
        const auto left = poseDelta(left_contact_fk, left_after_fk);
        const auto right = poseDelta(right_contact_fk, right_after_fk);
        contact_left_pos = left.first; contact_left_ang = left.second;
        contact_right_pos = right.first; contact_right_ang = right.second;
    }
    a1v::PoseError left_target_error{std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::infinity()};
    a1v::PoseError right_target_error = left_target_error;
    if (fk_after_valid) {
        left_target_error = a1v::poseError(a1_verify_.grasp_target.left,
                                           fromEigenPose(left_after_fk));
        right_target_error = a1v::poseError(a1_verify_.grasp_target.right,
                                            fromEigenPose(right_after_fk));
    }
    const double arm_feedback_age_sec = arm_after_valid
        ? std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() -
              arm_after.recv_steady_sec
        : std::numeric_limits<double>::infinity();
    const bool arm_short_window_ok = arm_window.stable() && arm_after_valid &&
        arm_feedback_age_sec <= 0.10 && arm_after.seq == arm_window.lastSeq();
    const bool eef_target_ok = fk_after_valid &&
        a1g::poseWithinLimits(left_target_error.pos_m, left_target_error.angle_rad,
                             kEefPositionTolM, kEefAngleTolRad) &&
        a1g::poseWithinLimits(right_target_error.pos_m, right_target_error.angle_rad,
                             kEefPositionTolM, kEefAngleTolRad);
    const MujocoRodTruth after = readMujocoRodTruth();
    double rod_delta = std::numeric_limits<double>::infinity();
    double rod_angle_delta = std::numeric_limits<double>::infinity();
    if (after.ok) {
        rod_delta = positionDistance(after.position, before.position);
        rod_angle_delta = quaternionDistance(after.orientation, before.orientation);
    }

    bool collision_ok = false;
    std::string collision_detail = "verification model unavailable";
    if (action_ok && contact_ok && a1_verify_.valid && a1_verify_.model && a1_verify_.scene) {
        DualArmCarryTask::JointFeedback joints;
        if (ctx_->latestJointFeedback(joints)) {
            moveit::core::RobotState closed_state(a1_verify_.model);
            bool complete = true;
            for (const auto& variable : a1_verify_.model->getVariableNames()) {
                const auto it = joints.positions.find(variable);
                if (it == joints.positions.end() || !std::isfinite(it->second)) {
                    complete = false;
                    collision_detail = "missing/non-finite joint feedback for " + variable;
                    break;
                }
                closed_state.setVariablePosition(variable, it->second);
            }
            if (complete) {
                closed_state.update();
                if (!closed_state.satisfiesBounds()) {
                    collision_detail = "closed-state joint limit violation";
                } else {
                    collision_detection::CollisionRequest request;
                    request.contacts = true;
                    request.max_contacts = 100;
                    collision_detection::CollisionResult result;
                    a1_verify_.scene->checkCollision(request, result, closed_state, a1_verify_.grasp_acm);
                    collision_ok = !result.collision;
                    collision_detail = result.collision
                        ? "collision remains after allowing only the four finger-to-rod pairs"
                        : "no other collision; only four finger-to-rod pairs allowed in validation ACM";
                    RCLCPP_INFO(ctx_->get_logger(),
                        "[P3A2C_COLLISION] result=%s contact_pairs=%zu bounds=PASS detail=%s",
                        collision_ok ? "PASS" : "FAIL", result.contacts.size(), collision_detail.c_str());
                }
            }
        }
    }

    for (int i = 0; i < 2; ++i) {
        DualArmCarryTask::GripperContactFeedback fb;
        ctx_->latestGripperContact(arm_names[i], fb);
        RCLCPP_INFO(ctx_->get_logger(),
            "[P3A2C_FEEDBACK] arm=%s q1=%.6f m q2=%.6f m total_gap=%.6f m left_finger_obj_rod_01=%d "
            "right_finger_obj_rod_01=%d update_seq=%llu action_protocol=%s window_samples=%zu",
            arm_names[i].c_str(), fb.q1, fb.q2, fb.q1+fb.q2, fb.left_contact ? 1 : 0,
            fb.right_contact ? 1 : 0, fb.seq, action_ok ? "SUCCEEDED" : "FAILED",
            windows[i].count());
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[P3A2C_ARM_CONTACT_DELTA] max_joint_delta=%.6f rad joint=%s left_dpos=%.6f m left_drot=%.5f deg right_dpos=%.6f m right_drot=%.5f deg FK_valid=%d",
        contact_arm_max_delta, contact_worst_joint.c_str(),
        contact_left_pos, contact_left_ang * 180.0 / M_PI,
        contact_right_pos, contact_right_ang * 180.0 / M_PI, fk_after_valid ? 1 : 0);
    RCLCPP_INFO(ctx_->get_logger(),
        "[P3A2C_ARM_STABILITY] fresh_window=%.3f s samples=%zu seq=%llu range=%.6f rad limit=%.6f pass=%d feedback_age=%.3f s; "
        "eef_target left=%.6f m/%.4f deg right=%.6f m/%.4f deg limits=%.3f m/%.2f deg pass=%d; "
        "cumulative_duration=%.3f s max_joint_delta=%.6f rad joint=%s contact_delta=%.6f rad contact_joint=%s baseline_excursion=%.6f rad; "
        "rod_before=(%.6f,%.6f,%.6f) rod_after_valid=%d rod_delta=%.6f m rod_angle_delta=%.5f deg",
        arm_window.duration(), arm_window.samples(), arm_window.lastSeq(), arm_window.maxSpread(),
        kArmSpreadTolRad, arm_short_window_ok ? 1 : 0, arm_feedback_age_sec,
        left_target_error.pos_m, left_target_error.angle_rad * 180.0 / M_PI,
        right_target_error.pos_m, right_target_error.angle_rad * 180.0 / M_PI,
        kEefPositionTolM, kEefAngleTolRad * 180.0 / M_PI, eef_target_ok ? 1 : 0,
        arm_after_valid ? arm_after.recv_steady_sec - arm_before.recv_steady_sec : 0.0,
        arm_max_delta, worst_joint.c_str(), contact_arm_max_delta, contact_worst_joint.c_str(),
        pre_hold_max_delta,
        before.position.x, before.position.y, before.position.z, after.ok ? 1 : 0,
        rod_delta, rod_angle_delta * 180.0 / M_PI);

    if (!action_ok) {
        detail = "E_P3A2C_ACTION_FAILED_OR_UNCERTAIN";
        return false;
    }
    if (!contact_ok) {
        detail = "E_P3A2C_BILATERAL_CONTACT_WINDOW_FAILED";
        return false;
    }
    if (!collision_ok) {
        detail = "E_P3A2C_CLOSED_STATE_COLLISION: " + collision_detail;
        return false;
    }
    if (!arm_feedback_complete || !arm_short_window_ok) {
        detail = "E_P3A2C_ARM_SHORT_WINDOW_UNSTABLE_OR_FEEDBACK_INCOMPLETE";
        return false;
    }
    if (!eef_target_ok) {
        detail = "E_P3A2C_EEF_OUTSIDE_GRASP_TARGET_TOLERANCE";
        return false;
    }
    if (!after.ok || rod_delta > ctx_->descent_truth_pos_tol_m_ ||
        rod_angle_delta > ctx_->descent_truth_angle_tol_deg_ * M_PI / 180.0) {
        detail = "E_P3A2C_ROD_TRUTH_CHANGED_OR_UNAVAILABLE";
        return false;
    }
    detail = "both grippers have fresh bilateral obj_rod_01 contacts; gap windows stable; no weld/attach/lift";
    return true;
}

bool DualArmController::runIdealWeldCarry(const a1t::TruthPayload& truth, std::string& detail)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1] grasp_model=ideal_weld: 理想刚性抓持；未验证摩擦夹持或抗滑。"
        "不要求真实双指接触，不执行接触闭合验收。");

    auto awaitStableRodTruth = [&](const char* phase, MujocoRodTruth& stable,
                                   std::string& reason) {
        using Sample = std::pair<std::chrono::steady_clock::time_point, MujocoRodTruth>;
        std::deque<Sample> window;
        const auto deadline = std::chrono::steady_clock::now() + 6s;
        std::size_t total_samples = 0;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            MujocoRodTruth sample = readMujocoRodTruth();
            if (!sample.ok) {
                reason = sample.detail;
                return false;
            }
            const auto now = std::chrono::steady_clock::now();
            ++total_samples;
            window.emplace_back(now, sample);
            while (!window.empty() && now - window.front().first > 600ms) window.pop_front();
            if (window.size() >= 8 && now - window.front().first >= 500ms) {
                double max_position_spread = 0.0;
                double max_angle_spread = 0.0;
                double max_linear_speed = 0.0;
                double max_angular_speed = 0.0;
                const auto& reference = window.front().second;
                for (const auto& entry : window) {
                    const auto& s = entry.second;
                    max_position_spread = std::max(max_position_spread,
                        std::sqrt(std::pow(s.position.x - reference.position.x, 2) +
                                  std::pow(s.position.y - reference.position.y, 2) +
                                  std::pow(s.position.z - reference.position.z, 2)));
                    max_angle_spread = std::max(max_angle_spread,
                        a1t::rotationDistanceRad(s.orientation, reference.orientation));
                    max_linear_speed = std::max(max_linear_speed,
                        std::sqrt(s.linear_velocity.x * s.linear_velocity.x +
                                  s.linear_velocity.y * s.linear_velocity.y +
                                  s.linear_velocity.z * s.linear_velocity.z));
                    max_angular_speed = std::max(max_angular_speed,
                        std::sqrt(s.angular_velocity.x * s.angular_velocity.x +
                                  s.angular_velocity.y * s.angular_velocity.y +
                                  s.angular_velocity.z * s.angular_velocity.z));
                }
                if (max_position_spread <= 0.0005 && max_angle_spread <= 0.2 * M_PI / 180.0 &&
                    max_linear_speed <= 0.01 && max_angular_speed <= 0.05) {
                    stable = window.back().second;
                    RCLCPP_INFO(ctx_->get_logger(),
                        "[S1_ROD_STABILITY] phase=%s samples=%zu window=%.3fs pos_spread=%.3fmm "
                        "angle_spread=%.3fdeg max_linear_speed=%.5fm/s max_angular_speed=%.5frad/s",
                        phase, window.size(), std::chrono::duration<double>(
                            window.back().first - window.front().first).count(),
                        max_position_spread * 1000.0, max_angle_spread * 180.0 / M_PI,
                        max_linear_speed, max_angular_speed);
                    return true;
                }
            }
            std::this_thread::sleep_for(30ms);
        }
        reason = std::string("stable rod truth window not achieved within bound; samples=") +
                 std::to_string(total_samples);
        return false;
    };

    if (!a1_verify_.valid || !a1_verify_.pregrasp_reached_this_round ||
        !a1_verify_.descent_plan_valid || ctx_->getCurrentState() != TaskState::PREGRASP_REACHED) {
        detail = "E_S1_PRECONDITION: this run did not pass verified A1 pregrasp and descent";
        return false;
    }
    std::string controller_detail;
    if (!checkControllerStatesEnabled(controller_detail)) {
        detail = "E_S1_CONTROLLER_GATE: " + controller_detail;
        return false;
    }

    auto physical_before = readMujocoRodTruth();
    if (!physical_before.ok) {
        detail = "E_S1_TRUTH_BEFORE_WELD: " + physical_before.detail;
        return false;
    }
    const a1v::Vec3 frozen{truth.position.x, truth.position.y, truth.position.z};
    if (std::sqrt(std::pow(physical_before.position.x - frozen.x, 2) +
                  std::pow(physical_before.position.y - frozen.y, 2) +
                  std::pow(physical_before.position.z - frozen.z, 2)) >
            ctx_->descent_truth_pos_tol_m_ ||
        a1t::rotationDistanceRad(physical_before.orientation,
            {truth.orientation.x, truth.orientation.y, truth.orientation.z, truth.orientation.w}) >
            ctx_->descent_truth_angle_tol_deg_ * M_PI / 180.0) {
        detail = "E_S1_TRUTH_REFERENCE_MISMATCH: physical rod no longer matches frozen A1 truth";
        return false;
    }

    auto model = ctx_->dual_arm_->getRobotModel();
    auto current = ctx_->dual_arm_->getCurrentState();
    if (!model || !current || a1_verify_.left_eef.empty() || a1_verify_.right_eef.empty()) {
        detail = "E_S1_ROBOT_STATE_UNAVAILABLE";
        return false;
    }
    const Eigen::Isometry3d object_initial = toEigenPose(a1v::ArmTarget{
        {physical_before.position.x, physical_before.position.y, physical_before.position.z},
        {physical_before.orientation.x, physical_before.orientation.y,
         physical_before.orientation.z, physical_before.orientation.w}});
    const Eigen::Isometry3d left_hand_initial = current->getGlobalLinkTransform(a1_verify_.left_eef);
    const Eigen::Isometry3d right_hand_initial = current->getGlobalLinkTransform(a1_verify_.right_eef);
    const Eigen::Isometry3d object_to_left = object_initial.inverse() * left_hand_initial;
    const Eigen::Isometry3d object_to_right = object_initial.inverse() * right_hand_initial;
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_GRASP_TRANSFORM] object->left_hand=(%.5f,%.5f,%.5f) object->right_hand=(%.5f,%.5f,%.5f)",
        object_to_left.translation().x(), object_to_left.translation().y(),
        object_to_left.translation().z(), object_to_right.translation().x(),
        object_to_right.translation().y(), object_to_right.translation().z());

    if (!ctx_->weld_pub_ || !ctx_->weld_state_sub_) {
        detail = "E_S1_WELD_INTERFACE_UNAVAILABLE";
        return false;
    }
    ctx_->weld_state_received_.store(false, std::memory_order_release);
    std_msgs::msg::Bool weld_on;
    weld_on.data = true;
    ctx_->weld_pub_->publish(weld_on);
    const auto weld_deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < weld_deadline &&
           !ctx_->weld_state_received_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(20ms);
    }
    if (!ctx_->weld_state_received_.load(std::memory_order_acquire) ||
        !ctx_->weld_state_active_.load(std::memory_order_acquire)) {
        detail = "E_S1_DUAL_WELD_NOT_CONFIRMED: physical-thread state feedback did not confirm both welds";
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(), "[S1_WELD_STATE] dual_weld_active=true confirmed by physics-thread feedback");
    std::this_thread::sleep_for(100ms);
    const MujocoRodTruth after_weld = readMujocoRodTruth();
    if (!after_weld.ok) {
        detail = "E_S1_TRUTH_AFTER_WELD: " + after_weld.detail;
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(), "[S1_WELD_CONTINUITY] before->after=%s",
        rodDeltaText(physical_before.position, physical_before.orientation,
                     after_weld.position, after_weld.orientation).c_str());
    if (std::sqrt(std::pow(after_weld.position.x - physical_before.position.x, 2) +
                  std::pow(after_weld.position.y - physical_before.position.y, 2) +
                  std::pow(after_weld.position.z - physical_before.position.z, 2)) > 0.005 ||
        a1t::rotationDistanceRad(after_weld.orientation, physical_before.orientation) >
            2.0 * M_PI / 180.0) {
        detail = "E_S1_WELD_POSE_JUMP: rod moved outside 5 mm / 2 deg during weld activation";
        return false;
    }

    if (!attachObject()) {
        std_msgs::msg::Bool weld_off;
        weld_off.data = false;
        ctx_->weld_pub_->publish(weld_off);
        detail = "E_S1_MOVEIT_ATTACH_FAILED: dual weld rolled back";
        return false;
    }
    moveit::planning_interface::PlanningSceneInterface psi;
    const auto attached = psi.getAttachedObjects({"aluminum_rod"});
    if (attached.count("aluminum_rod") != 1) {
        detail = "E_S1_ATTACH_STATE_MISMATCH";
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_ATTACH_STATE] MoveIt attached aluminum_rod to mj_left_hand; touch_links are exact parent + finger grasp links only");

    const auto* left_jmg = model->getJointModelGroup(ctx_->left_arm_group_);
    const auto* right_jmg = model->getJointModelGroup(ctx_->right_arm_group_);
    if (!left_jmg || !right_jmg) {
        detail = "E_S1_JOINT_GROUP_MISSING";
        return false;
    }
    auto interpolateObject = [](const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double t) {
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        out.translation() = (1.0 - t) * a.translation() + t * b.translation();
        const Eigen::Quaterniond qa(a.linear()), qb(b.linear());
        out.linear() = qa.slerp(t, qb).normalized().toRotationMatrix();
        return out;
    };
    moveit::planning_interface::MoveGroupInterface::Plan carry_plan;
    Eigen::Isometry3d object_place = Eigen::Isometry3d::Identity();
    const std::array<std::array<double, 2>, 1> placement_candidates{{
        {{ctx_->rod_target_x_, ctx_->rod_target_y_}}}};
    int selected_candidate = -1;
    for (std::size_t candidate_index = 0; candidate_index < placement_candidates.size(); ++candidate_index) {
        const double target_x = placement_candidates[candidate_index][0];
        const double target_y = placement_candidates[candidate_index][1];
        detail.clear();
        RCLCPP_INFO(ctx_->get_logger(),
            "[S1_CANDIDATE_BEGIN] index=%zu/%zu x=%.3f y=%.3f yaw_deg=%.3f mode=planning_only_until_selected",
            candidate_index + 1, placement_candidates.size(), target_x, target_y,
            ctx_->rod_target_yaw_rad_ * 180.0 / M_PI);
        auto planPlacementCandidate = [&](double target_x, double target_y,
                moveit::planning_interface::MoveGroupInterface::Plan& candidate_plan_out,
                Eigen::Isometry3d& selected_place_out) -> bool {
    const Eigen::Isometry3d object_start = object_initial;
    const double lift_delta = std::max(0.0, ctx_->lift_height_ - ctx_->grasp_height_);
    Eigen::Isometry3d object_lift = object_start;
    object_lift.translation().z() += lift_delta;
    Eigen::Isometry3d object_transport = object_lift;
    object_transport.translation().x() = target_x;
    object_transport.translation().y() = target_y;
    Eigen::Isometry3d object_rotate = object_transport;
    const double initial_yaw = std::atan2(object_transport.linear()(1, 0),
                                          object_transport.linear()(0, 0));
    const double yaw_delta = s1::absoluteYawDelta(initial_yaw, ctx_->rod_target_yaw_rad_);
    object_rotate.linear() = Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                             object_transport.linear();
    const double constructed_yaw = std::atan2(object_rotate.linear()(1, 0),
                                              object_rotate.linear()(0, 0));
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_ROTATION_TARGET] semantics=absolute_world_yaw initial=%.3fdeg requested=%.3fdeg "
        "delta=%.3fdeg constructed_endpoint=%.3fdeg rotation_tested=%s",
        initial_yaw * 180.0 / M_PI, ctx_->rod_target_yaw_rad_ * 180.0 / M_PI,
        yaw_delta * 180.0 / M_PI, constructed_yaw * 180.0 / M_PI,
        std::abs(yaw_delta) > 1e-9 ? "true" : "false");
    Eigen::Isometry3d object_place = object_rotate;
    object_place.translation().z() = ctx_->rod_center_ground_z_;
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_TARGET] object_xyz=(%.4f,%.4f,%.4f) world_yaw=%.3fdeg rotation_delta=%.3fdeg",
        object_place.translation().x(), object_place.translation().y(), object_place.translation().z(),
        constructed_yaw * 180.0 / M_PI, yaw_delta * 180.0 / M_PI);

    struct ObjectPathPoint { Eigen::Isometry3d pose; std::string stage; };
    std::vector<ObjectPathPoint> object_path;
    auto appendObjectSegment = [&](const Eigen::Isometry3d& from, const Eigen::Isometry3d& to,
                                   const std::string& stage, int steps) {
        for (int i = 1; i <= steps; ++i) {
            object_path.push_back({interpolateObject(from, to,
                static_cast<double>(i) / static_cast<double>(steps)), stage});
        }
    };
    const double path_step_m = 0.005;
    auto stepsForDistance = [&](const Eigen::Isometry3d& from, const Eigen::Isometry3d& to) {
        return std::max(1, static_cast<int>(std::ceil(
            (to.translation() - from.translation()).norm() / path_step_m)));
    };
    const int lift_steps = stepsForDistance(object_start, object_lift);
    const int transport_steps = stepsForDistance(object_lift, object_transport);
    const int place_steps = stepsForDistance(object_rotate, object_place);
    appendObjectSegment(object_start, object_lift, "LIFT", lift_steps);
    appendObjectSegment(object_lift, object_transport, "TRANSPORT", transport_steps);
    if (std::abs(yaw_delta) > 1e-9) {
        appendObjectSegment(object_transport, object_rotate, "ROTATE", 5);
    } else {
        RCLCPP_INFO(ctx_->get_logger(),
            "[S1_ROTATION_PLAN] rotation_delta=0 rotation_tested=false; no rotation waypoints added");
    }
    appendObjectSegment(object_rotate, object_place, "DESCEND_PLACE", place_steps);
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_OBJECT_PATH_SAMPLING] max_translation_step=%.1fmm lift=%d transport=%d place=%d total=%zu",
        path_step_m * 1000.0, lift_steps, transport_steps, place_steps, object_path.size());

    auto placementProjectionMatches = [&](const Eigen::Isometry3d& pose) {
        const double xy = (pose.translation().head<2>() - object_place.translation().head<2>()).norm();
        const double angle = Eigen::AngleAxisd(
            pose.linear().transpose() * object_place.linear()).angle();
        return xy <= 0.005 && angle <= 2.0 * M_PI / 180.0;
    };
    auto supportClearance = [&](const Eigen::Isometry3d& pose) {
        return pose.translation().z() - ctx_->rod_half_height_ - ctx_->p2a_support_z_;
    };
    auto expectedSupportBand = [&](const Eigen::Isometry3d& pose) {
        return placementProjectionMatches(pose) && s1::supportHeightEligible(
            pose.translation().z(), ctx_->rod_half_height_, ctx_->p2a_support_z_,
            ctx_->p2a_resting_z_tol_);
    };

    // Plan the complete object-space path before dispatching any part of it.
    // This uses the same hand-adjusted criterion model, support plane, attached
    // rod, actual finger positions, and narrow grasp ACM as the A1 checks.
    if (!a1_verify_.valid || !a1_verify_.scene || !a1_verify_.model || !current) {
        detail = "E_S1_CRITERION_SCENE_UNAVAILABLE";
        return false;
    }
    const auto attached_objects = psi.getAttachedObjects({"aluminum_rod"});
    if (attached_objects.count("aluminum_rod") != 1) {
        detail = "E_S1_ATTACHED_ROD_MISSING_DURING_PREPLAN";
        return false;
    }
    auto criterion_scene = a1_verify_.scene->diff();
    if (!criterion_scene->processAttachedCollisionObjectMsg(attached_objects.at("aluminum_rod"))) {
        detail = "E_S1_CRITERION_ATTACH_IMPORT_FAILED";
        return false;
    }
    auto criterion_model = a1_verify_.model;
    const auto* criterion_dual = criterion_model->getJointModelGroup(ctx_->dual_arm_group_);
    const auto* criterion_left = criterion_model->getJointModelGroup(ctx_->left_arm_group_);
    const auto* criterion_right = criterion_model->getJointModelGroup(ctx_->right_arm_group_);
    if (!criterion_dual || !criterion_left || !criterion_right) {
        detail = "E_S1_CRITERION_GROUP_MISSING";
        return false;
    }
    const std::set<std::string> criterion_variables(
        criterion_model->getVariableNames().begin(), criterion_model->getVariableNames().end());
    const auto live_model = ctx_->dual_arm_->getRobotModel();
    const auto* live_dual = live_model->getJointModelGroup(ctx_->dual_arm_group_);
    const std::set<std::string> live_variables(
        live_model->getVariableNames().begin(), live_model->getVariableNames().end());
    if (!live_dual) {
        detail = "E_S1_LIVE_JOINT_GROUP_MISSING";
        return false;
    }
    moveit::core::RobotState criterion_start(*a1_verify_.start);
    for (const auto& name : current->getVariableNames()) {
        if (criterion_variables.count(name) != 0) {
            criterion_start.setVariablePosition(name, current->getVariablePosition(name));
        }
    }
    // The descent criterion's private feedback mapping is the authoritative
    // measured finger snapshot. Preserve those four values if the live
    // CurrentStateMonitor still carries an older/default value.
    std::size_t mapped_finger_count = 0;
    for (const auto& name : criterion_model->getVariableNames()) {
        if (name.find("finger_joint") == std::string::npos) continue;
        if (!a1_verify_.descent_start || live_variables.count(name) == 0) continue;
        const double measured = a1_verify_.descent_start->getVariablePosition(name);
        const double csm = current->getVariablePosition(name);
        criterion_start.setVariablePosition(name, measured);
        RCLCPP_INFO(ctx_->get_logger(),
            "[S1_FINGER_STATE] joint=%s measured_snapshot=%.6f live_CSM=%.6f criterion=%.6f",
            name.c_str(), measured, csm, criterion_start.getVariablePosition(name));
        ++mapped_finger_count;
    }
    if (mapped_finger_count != 4) {
        detail = "E_S1_MEASURED_FINGER_SNAPSHOT_INCOMPLETE count=" +
                 std::to_string(mapped_finger_count);
        return false;
    }
    criterion_start.update();
    if (!criterion_start.satisfiesBounds(criterion_dual, 1e-4)) {
        detail = "E_S1_START_STATE_BOUNDS";
        return false;
    }
    const Eigen::Isometry3d start_inferred_left =
        criterion_start.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
    const Eigen::Isometry3d start_inferred_right =
        criterion_start.getGlobalLinkTransform(a1_verify_.right_eef) * object_to_right.inverse();
    const double start_dual_position_residual =
        (start_inferred_left.translation() - start_inferred_right.translation()).norm();
    const double start_dual_angle_residual = Eigen::AngleAxisd(
        start_inferred_left.linear().transpose() * start_inferred_right.linear()).angle();
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_FK_START] left_rod=(%.6f,%.6f,%.6f) right_rod=(%.6f,%.6f,%.6f) "
        "dual_residual=(%.3fmm,%.3fdeg) measured_fingers_mapped=%zu",
        start_inferred_left.translation().x(), start_inferred_left.translation().y(),
        start_inferred_left.translation().z(), start_inferred_right.translation().x(),
        start_inferred_right.translation().y(), start_inferred_right.translation().z(),
        start_dual_position_residual * 1000.0, start_dual_angle_residual * 180.0 / M_PI,
        mapped_finger_count);
    if (!std::isfinite(start_dual_position_residual) || !std::isfinite(start_dual_angle_residual) ||
        start_dual_position_residual > 0.005 || start_dual_angle_residual > 2.0 * M_PI / 180.0) {
        detail = "E_S1_START_GRASP_MAPPING_RESIDUAL exceeds 5 mm / 2 deg";
        return false;
    }
    auto logTransform = [&](const char* side, const Eigen::Isometry3d& transform) {
        const Eigen::Quaterniond q(transform.linear());
        RCLCPP_INFO(ctx_->get_logger(),
            "[S1_GRASP_TRANSFORM_FULL] side=%s xyz=(%.7f,%.7f,%.7f) qxyzw=(%.8f,%.8f,%.8f,%.8f)",
            side, transform.translation().x(), transform.translation().y(), transform.translation().z(),
            q.x(), q.y(), q.z(), q.w());
    };
    logTransform("left", object_to_left);
    logTransform("right", object_to_right);
    std::ofstream fk_replay;
    if (!ctx_->expected_run_id_.empty()) {
        std::string safe_run_id = ctx_->expected_run_id_;
        for (auto& ch : safe_run_id) {
            if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_' && ch != '-') ch = '_';
        }
        const std::string replay_path = "experiment_results/a1_runs/" + safe_run_id +
            "/s1_fk_replay_candidate_" + std::to_string(candidate_index + 1) + ".csv";
        fk_replay.open(replay_path, std::ios::out | std::ios::trunc);
        if (fk_replay) {
            fk_replay << "source,stage,index,alpha,left_x,left_y,left_z,left_qx,left_qy,left_qz,left_qw,"
                         "right_x,right_y,right_z,right_qx,right_qy,right_qz,right_qw,dual_pos_mm,dual_angle_deg";
            for (const auto& name : criterion_dual->getVariableNames()) fk_replay << ",q_" << name;
            fk_replay << '\n';
        } else {
            RCLCPP_WARN(ctx_->get_logger(), "[S1_FK_REPLAY] could not open %s", replay_path.c_str());
        }
    }
    auto writeFkReplay = [&](const char* source, const std::string& stage, std::size_t index,
                             double alpha, const moveit::core::RobotState& state) {
        if (!fk_replay) return;
        const Eigen::Isometry3d left =
            state.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
        const Eigen::Isometry3d right =
            state.getGlobalLinkTransform(a1_verify_.right_eef) * object_to_right.inverse();
        const Eigen::Quaterniond ql(left.linear()), qr(right.linear());
        fk_replay << source << ',' << stage << ',' << index << ',' << alpha << ','
            << left.translation().x() << ',' << left.translation().y() << ',' << left.translation().z() << ','
            << ql.x() << ',' << ql.y() << ',' << ql.z() << ',' << ql.w() << ','
            << right.translation().x() << ',' << right.translation().y() << ',' << right.translation().z() << ','
            << qr.x() << ',' << qr.y() << ',' << qr.z() << ',' << qr.w() << ','
            << (left.translation() - right.translation()).norm() * 1000.0 << ','
            << Eigen::AngleAxisd(left.linear().transpose() * right.linear()).angle() * 180.0 / M_PI;
        for (const auto& name : criterion_dual->getVariableNames()) {
            fk_replay << ',' << state.getVariablePosition(name);
        }
        fk_replay << '\n';
    };
    writeFkReplay("actual_start", "START", 0, 0.0, criterion_start);
    moveit::core::RobotState seed_live(*current);
    for (const auto& name : criterion_model->getVariableNames()) {
        if (name.find("finger_joint") != std::string::npos && live_variables.count(name) != 0) {
            seed_live.setVariablePosition(name, criterion_start.getVariablePosition(name));
        }
    }
    seed_live.update();
    collision_detection::CollisionRequest path_collision_request;
    path_collision_request.contacts = true;
    path_collision_request.max_contacts = 20;
    path_collision_request.group_name = ctx_->dual_arm_group_;

    auto makeEndpointPose = [](const Eigen::Isometry3d& tf) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.translation().x();
        pose.position.y = tf.translation().y();
        pose.position.z = tf.translation().z();
        const Eigen::Quaterniond q(tf.linear());
        pose.orientation.x = q.x(); pose.orientation.y = q.y();
        pose.orientation.z = q.z(); pose.orientation.w = q.w();
        return pose;
    };
    std::size_t endpoint_ik_rejected = 0;
    std::size_t endpoint_collision_rejected = 0;
    std::size_t endpoint_bounds_rejected = 0;
    bool endpoint_accepted = false;
    std::string endpoint_reason = "all finite endpoint candidates rejected";
    const Eigen::Isometry3d endpoint_left = object_place * object_to_left;
    const Eigen::Isometry3d endpoint_right = object_place * object_to_right;
    for (int endpoint_candidate = 0; endpoint_candidate < 5 && !endpoint_accepted; ++endpoint_candidate) {
        moveit::core::RobotState live_candidate(seed_live);
        if (endpoint_candidate > 0) {
            const std::size_t joint_index = static_cast<std::size_t>(endpoint_candidate - 1) % 3 + 1;
            const double sign = endpoint_candidate <= 3 ? 1.0 : -1.0;
            for (const auto* arm : {left_jmg, right_jmg}) {
                const auto& variables = arm->getVariableNames();
                if (joint_index < variables.size()) {
                    live_candidate.setVariablePosition(variables[joint_index],
                        live_candidate.getVariablePosition(variables[joint_index]) + sign * 0.18);
                }
            }
            live_candidate.update();
        }
        const bool left_ok = live_candidate.setFromIK(left_jmg, makeEndpointPose(endpoint_left), 0.30);
        const bool right_ok = left_ok && live_candidate.setFromIK(right_jmg, makeEndpointPose(endpoint_right), 0.30);
        if (!left_ok || !right_ok) {
            ++endpoint_ik_rejected;
            endpoint_reason = "combined endpoint IK failed";
            continue;
        }
        live_candidate.update();
        moveit::core::RobotState criterion_candidate(criterion_start);
        for (const auto& name : live_candidate.getVariableNames()) {
            if (criterion_variables.count(name) != 0) {
                criterion_candidate.setVariablePosition(name, live_candidate.getVariablePosition(name));
            }
        }
        criterion_candidate.update();
        if (!criterion_candidate.satisfiesBounds(criterion_dual, 1e-4)) {
            ++endpoint_bounds_rejected;
            endpoint_reason = "endpoint IK solution exceeded joint limits";
            continue;
        }
        const Eigen::Isometry3d inferred =
            criterion_candidate.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
        const Eigen::Isometry3d inferred_right =
            criterion_candidate.getGlobalLinkTransform(a1_verify_.right_eef) * object_to_right.inverse();
        const double endpoint_target_position_error =
            (inferred.translation() - object_place.translation()).norm();
        const double endpoint_target_angle_error = Eigen::AngleAxisd(
            inferred.linear().transpose() * object_place.linear()).angle();
        const double endpoint_position_residual =
            (inferred.translation() - inferred_right.translation()).norm();
        const double endpoint_angle_residual = Eigen::AngleAxisd(
            inferred.linear().transpose() * inferred_right.linear()).angle();
        if (!std::isfinite(endpoint_target_position_error) || !std::isfinite(endpoint_target_angle_error) ||
            endpoint_target_position_error > 0.005 || endpoint_target_angle_error > 2.0 * M_PI / 180.0) {
            ++endpoint_collision_rejected;
            endpoint_reason = "endpoint object pose residual exceeds 5 mm / 2 deg";
            continue;
        }
        if (!std::isfinite(endpoint_position_residual) || !std::isfinite(endpoint_angle_residual) ||
            endpoint_position_residual > 0.005 || endpoint_angle_residual > 2.0 * M_PI / 180.0) {
            ++endpoint_collision_rejected;
            endpoint_reason = "endpoint dual-grasp residual exceeds 5 mm / 2 deg";
            continue;
        }
        if (supportClearance(inferred) < -1e-4) {
            ++endpoint_collision_rejected;
            endpoint_reason = "attached rod would penetrate the support surface";
            continue;
        }
        collision_detection::AllowedCollisionMatrix endpoint_acm = a1_verify_.grasp_acm;
        if (expectedSupportBand(inferred)) endpoint_acm.setEntry(a1_verify_.rod_id, a1_verify_.support_id, true);
        collision_detection::CollisionResult endpoint_collision;
        criterion_scene->checkCollision(path_collision_request, endpoint_collision,
                                        criterion_candidate, endpoint_acm);
        if (endpoint_collision.collision) {
            ++endpoint_collision_rejected;
            endpoint_reason = endpoint_collision.contacts.empty() ? "criterion collision" :
                endpoint_collision.contacts.begin()->first.first + "<->" +
                endpoint_collision.contacts.begin()->first.second;
            continue;
        }
        endpoint_accepted = true;
        endpoint_reason = "accepted";
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_CANDIDATE_ENDPOINT] x=%.3f y=%.3f accepted=%s finite=5 ik_rejected=%zu "
        "collision_rejected=%zu bounds_rejected=%zu reason=%s",
        target_x, target_y, endpoint_accepted ? "true" : "false", endpoint_ik_rejected,
        endpoint_collision_rejected, endpoint_bounds_rejected, endpoint_reason.c_str());
    if (!endpoint_accepted) {
        detail = "E_S1_ENDPOINT_REJECT x=" + std::to_string(target_x) + " y=" +
            std::to_string(target_y) + " reason=" + endpoint_reason;
        return false;
    }
    std::vector<moveit::core::RobotState> coordinated_states;
    coordinated_states.emplace_back(criterion_start);
    std::vector<moveit::core::RobotState> coordinated_live_states;
    coordinated_live_states.emplace_back(seed_live);
    moveit::core::RobotState seed(criterion_start);
    std::size_t ik_rejected = 0, collision_rejected = 0, bounds_rejected = 0;
    std::size_t geometry_rejected = 0, geometry_samples = 0;
    double max_ik_dual_position_residual = 0.0, max_ik_dual_angle_residual = 0.0;
    double max_geometry_dual_position_residual = 0.0, max_geometry_dual_angle_residual = 0.0;
    std::string last_reject;
    for (std::size_t wi = 0; wi < object_path.size(); ++wi) {
        const Eigen::Isometry3d desired_left = object_path[wi].pose * object_to_left;
        const Eigen::Isometry3d desired_right = object_path[wi].pose * object_to_right;
        auto makePose = [](const Eigen::Isometry3d& tf) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = tf.translation().x();
            pose.position.y = tf.translation().y();
            pose.position.z = tf.translation().z();
            const Eigen::Quaterniond q(tf.linear());
            pose.orientation.x = q.x(); pose.orientation.y = q.y();
            pose.orientation.z = q.z(); pose.orientation.w = q.w();
            return pose;
        };
        bool accepted = false;
        for (int candidate_index = 0; candidate_index < 5 && !accepted; ++candidate_index) {
            moveit::core::RobotState candidate_live(seed_live);
            if (candidate_index > 0) {
                const std::size_t joint_index = static_cast<std::size_t>(candidate_index - 1) % 3 + 1;
                const double sign = candidate_index <= 3 ? 1.0 : -1.0;
                for (const auto* arm : {left_jmg, right_jmg}) {
                    const auto& variables = arm->getVariableNames();
                    if (joint_index < variables.size()) {
                        candidate_live.setVariablePosition(variables[joint_index],
                            candidate_live.getVariablePosition(variables[joint_index]) + sign * 0.18);
                    }
                }
                candidate_live.update();
                if (!candidate_live.satisfiesBounds(ctx_->dual_arm_->getRobotModel()->getJointModelGroup(
                        ctx_->dual_arm_group_), 1e-4)) {
                    ++bounds_rejected;
                    last_reject = "seed perturbation exceeded a joint limit";
                    continue;
                }
            }
            const bool left_ok = candidate_live.setFromIK(left_jmg, makePose(desired_left), 0.30);
            const bool right_ok = left_ok && candidate_live.setFromIK(right_jmg, makePose(desired_right), 0.30);
            if (!left_ok || !right_ok) {
                ++ik_rejected;
                last_reject = "coordinated left/right IK did not solve this candidate";
                continue;
            }
            candidate_live.update();
            moveit::core::RobotState candidate(seed);
            for (const auto& name : candidate_live.getVariableNames()) {
                if (criterion_variables.count(name) != 0) {
                    candidate.setVariablePosition(name, candidate_live.getVariablePosition(name));
                }
            }
            candidate.update();
            if (!candidate.satisfiesBounds(criterion_dual, 1e-4)) {
                ++bounds_rejected;
                last_reject = "IK solution exceeded a joint limit";
                continue;
            }
            const Eigen::Isometry3d inferred_left =
                candidate.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
            const Eigen::Isometry3d inferred_right =
                candidate.getGlobalLinkTransform(a1_verify_.right_eef) * object_to_right.inverse();
            writeFkReplay("pre_time_ik_candidate", object_path[wi].stage, wi, 1.0, candidate);
            const double ik_position_residual =
                (inferred_left.translation() - inferred_right.translation()).norm();
            const double ik_angle_residual = Eigen::AngleAxisd(
                inferred_left.linear().transpose() * inferred_right.linear()).angle();
            const double left_target_position_error =
                (inferred_left.translation() - object_path[wi].pose.translation()).norm();
            const double right_target_position_error =
                (inferred_right.translation() - object_path[wi].pose.translation()).norm();
            const double left_target_angle_error = Eigen::AngleAxisd(
                inferred_left.linear().transpose() * object_path[wi].pose.linear()).angle();
            const double right_target_angle_error = Eigen::AngleAxisd(
                inferred_right.linear().transpose() * object_path[wi].pose.linear()).angle();
            if (!std::isfinite(ik_position_residual) || !std::isfinite(ik_angle_residual) ||
                ik_position_residual > 0.005 || ik_angle_residual > 2.0 * M_PI / 180.0 ||
                left_target_position_error > 0.005 || right_target_position_error > 0.005 ||
                left_target_angle_error > 2.0 * M_PI / 180.0 ||
                right_target_angle_error > 2.0 * M_PI / 180.0) {
                ++geometry_rejected;
                last_reject = "IK point violates fixed dual-grasp transform / object target residual";
                continue;
            }
            const Eigen::Isometry3d inferred_object =
                candidate.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
            if (object_path[wi].stage == "DESCEND_PLACE" &&
                placementProjectionMatches(inferred_object) && supportClearance(inferred_object) < -1e-4) {
                ++collision_rejected;
                last_reject = "attached rod would penetrate the support surface";
                continue;
            }
            collision_detection::AllowedCollisionMatrix waypoint_acm = a1_verify_.grasp_acm;
            if (object_path[wi].stage == "DESCEND_PLACE" && expectedSupportBand(inferred_object)) {
                waypoint_acm.setEntry(a1_verify_.rod_id, a1_verify_.support_id, true);
            }
            collision_detection::CollisionResult collision;
            criterion_scene->checkCollision(path_collision_request, collision, candidate,
                                             waypoint_acm);
            if (collision.collision) {
                ++collision_rejected;
                last_reject = collision.contacts.empty() ? "criterion collision" :
                    collision.contacts.begin()->first.first + "<->" +
                    collision.contacts.begin()->first.second;
                continue;
            }
            const moveit::core::RobotState& previous = coordinated_states.back();
            std::vector<double> previous_q, candidate_q;
            previous.copyJointGroupPositions(criterion_dual, previous_q);
            candidate.copyJointGroupPositions(criterion_dual, candidate_q);
            const std::size_t segment_subdivisions = a1v::subdivisionsFor(
                a1v::maxAbsJointDelta(previous_q, candidate_q), 0.01);
            bool segment_valid = true;
            const Eigen::Isometry3d previous_object = wi == 0 ? object_start : object_path[wi - 1].pose;
            for (std::size_t si = 1; si < segment_subdivisions; ++si) {
                const double alpha = static_cast<double>(si) / static_cast<double>(segment_subdivisions);
                const auto q = a1v::interpolateJoints(previous_q, candidate_q, alpha);
                moveit::core::RobotState interpolated(criterion_start);
                interpolated.setJointGroupPositions(criterion_dual, q);
                interpolated.update();
                if (!interpolated.satisfiesBounds(criterion_dual, 1e-4)) {
                    ++geometry_rejected;
                    last_reject = "pre-time geometric interpolation exceeds joint limits";
                    segment_valid = false;
                    break;
                }
                const Eigen::Isometry3d expected_object =
                    interpolateObject(previous_object, object_path[wi].pose, alpha);
                const Eigen::Isometry3d segment_left =
                    interpolated.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
                const Eigen::Isometry3d segment_right =
                    interpolated.getGlobalLinkTransform(a1_verify_.right_eef) * object_to_right.inverse();
                writeFkReplay("geometric_interpolation", object_path[wi].stage, wi, alpha, interpolated);
                const double segment_position_residual =
                    (segment_left.translation() - segment_right.translation()).norm();
                const double segment_angle_residual = Eigen::AngleAxisd(
                    segment_left.linear().transpose() * segment_right.linear()).angle();
                const bool segment_pose_ok =
                    (segment_left.translation() - expected_object.translation()).norm() <= 0.005 &&
                    (segment_right.translation() - expected_object.translation()).norm() <= 0.005 &&
                    Eigen::AngleAxisd(segment_left.linear().transpose() * expected_object.linear()).angle() <=
                        2.0 * M_PI / 180.0 &&
                    Eigen::AngleAxisd(segment_right.linear().transpose() * expected_object.linear()).angle() <=
                        2.0 * M_PI / 180.0;
                if (!std::isfinite(segment_position_residual) || !std::isfinite(segment_angle_residual) ||
                    segment_position_residual > 0.005 || segment_angle_residual > 2.0 * M_PI / 180.0 ||
                    !segment_pose_ok) {
                    ++geometry_rejected;
                    last_reject = "pre-time geometric interpolation violates fixed dual-grasp transform";
                    segment_valid = false;
                    break;
                }
                max_geometry_dual_position_residual = std::max(
                    max_geometry_dual_position_residual, segment_position_residual);
                max_geometry_dual_angle_residual = std::max(
                    max_geometry_dual_angle_residual, segment_angle_residual);
                ++geometry_samples;
                if (placementProjectionMatches(segment_left) && supportClearance(segment_left) < -1e-4) {
                    ++collision_rejected;
                    last_reject = "geometric interpolation makes attached rod penetrate support";
                    segment_valid = false;
                    break;
                }
                collision_detection::AllowedCollisionMatrix segment_acm = a1_verify_.grasp_acm;
                if (expectedSupportBand(segment_left)) {
                    segment_acm.setEntry(a1_verify_.rod_id, a1_verify_.support_id, true);
                }
                collision_detection::CollisionResult segment_collision;
                criterion_scene->checkCollision(path_collision_request, segment_collision,
                                               interpolated, segment_acm);
                if (segment_collision.collision) {
                    ++collision_rejected;
                    last_reject = segment_collision.contacts.empty() ? "pre-time geometric interpolation collision" :
                        segment_collision.contacts.begin()->first.first + "<->" +
                        segment_collision.contacts.begin()->first.second;
                    segment_valid = false;
                    break;
                }
            }
            if (!segment_valid) continue;
            seed = candidate;
            seed_live = candidate_live;
            coordinated_states.emplace_back(candidate);
            coordinated_live_states.emplace_back(candidate_live);
            max_ik_dual_position_residual = std::max(max_ik_dual_position_residual, ik_position_residual);
            max_ik_dual_angle_residual = std::max(max_ik_dual_angle_residual, ik_angle_residual);
            accepted = true;
            RCLCPP_INFO(ctx_->get_logger(),
                "[S1_PREPLAN_WAYPOINT] stage=%s index=%zu/%zu candidate=%d pos=(%.4f,%.4f,%.4f)",
                object_path[wi].stage.c_str(), wi + 1, object_path.size(), candidate_index,
                object_path[wi].pose.translation().x(), object_path[wi].pose.translation().y(),
                object_path[wi].pose.translation().z());
        }
        if (!accepted) {
            detail = "E_S1_PREPLAN_NO_CONTINUOUS_CANDIDATE at " + object_path[wi].stage +
                " waypoint " + std::to_string(wi + 1) + "/" + std::to_string(object_path.size()) +
                "; finite candidates=5 IK_rejected=" + std::to_string(ik_rejected) +
                " collision_rejected=" + std::to_string(collision_rejected) +
                " bounds_rejected=" + std::to_string(bounds_rejected) + " last=" + last_reject;
            RCLCPP_ERROR(ctx_->get_logger(), "[S1_PREPLAN_REJECT] %s", detail.c_str());
            return false;
        }
    }

    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_GEOMETRY_VALIDATED] ik_points=%zu interpolated_samples=%zu "
        "max_ik_residual=(%.3fmm,%.3fdeg) max_segment_residual=(%.3fmm,%.3fdeg)",
        coordinated_states.size() - 1, geometry_samples,
        max_ik_dual_position_residual * 1000.0, max_ik_dual_angle_residual * 180.0 / M_PI,
        max_geometry_dual_position_residual * 1000.0,
        max_geometry_dual_angle_residual * 180.0 / M_PI);
    robot_trajectory::RobotTrajectory coordinated(live_model, ctx_->dual_arm_group_);
    for (const auto& state : coordinated_live_states) coordinated.addSuffixWayPoint(state, 0.0);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(coordinated, 0.15, 0.12)) {
        detail = "E_S1_PREPLAN_IPTP_TIME_PARAMETERIZATION_FAILED";
        return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;
    coordinated.getRobotTrajectoryMsg(candidate_plan.trajectory_);
    moveit::core::robotStateToRobotStateMsg(*current, candidate_plan.start_state_);
    auto& points = candidate_plan.trajectory_.joint_trajectory.points;
    if (points.size() < 2 ||
        points.front().positions.size() != candidate_plan.trajectory_.joint_trajectory.joint_names.size()) {
        detail = "E_S1_PREPLAN_FINAL_TRAJECTORY_INVALID";
        return false;
    }
    const auto& trajectory_names = candidate_plan.trajectory_.joint_trajectory.joint_names;
    std::vector<s1::DirectionalJointLimits> time_limits;
    time_limits.reserve(trajectory_names.size());
    for (const auto& name : trajectory_names) {
        const auto& bounds = live_model->getVariableBounds(name);
        time_limits.push_back({bounds.min_velocity_, bounds.max_velocity_,
            bounds.min_acceleration_, bounds.max_acceleration_,
            bounds.velocity_bounded_, bounds.acceleration_bounded_});
    }
    std::vector<s1::TimedJointPoint> raw_iptp_points;
    raw_iptp_points.reserve(points.size());
    for (const auto& point : points) {
        raw_iptp_points.push_back({
            rclcpp::Duration(point.time_from_start).seconds(), point.positions,
            point.velocities, point.accelerations});
    }
    const auto pre_scale_iptp_points = raw_iptp_points;
    s1::TimeScalingReport time_scaling;
    std::string time_scaling_error;
    if (!s1::scaleTimedTrajectory(raw_iptp_points, time_limits, 0.15, 0.12, 0.01,
                                  time_scaling, time_scaling_error)) {
        detail = "E_S1_IPTP_TIME_SCALING_INPUT_INVALID: " + time_scaling_error;
        return false;
    }
    struct LimitPeak
    {
        double ratio{0.0};
        std::size_t point{0};
        std::size_t joint{0};
        double time{0.0};
        double value{0.0};
        double lower{0.0};
        double upper{0.0};
        double effective{0.0};
    } max_velocity_peak, max_acceleration_peak;
    for (std::size_t pi = 0; pi < pre_scale_iptp_points.size(); ++pi) {
        const auto& point = pre_scale_iptp_points[pi];
        for (std::size_t ji = 0; ji < trajectory_names.size(); ++ji) {
            const auto& bounds = time_limits[ji];
            const double effective_velocity = 0.15 *
                (point.velocities[ji] >= 0.0 ? bounds.max_velocity : -bounds.min_velocity);
            const double velocity_ratio = std::abs(point.velocities[ji]) / effective_velocity;
            if (velocity_ratio > max_velocity_peak.ratio) {
                max_velocity_peak = {velocity_ratio, pi, ji, point.time, point.velocities[ji],
                    bounds.min_velocity, bounds.max_velocity, effective_velocity};
            }
            const double effective_acceleration = 0.12 *
                (point.accelerations[ji] >= 0.0 ? bounds.max_acceleration : -bounds.min_acceleration);
            const double acceleration_ratio = std::abs(point.accelerations[ji]) / effective_acceleration;
            if (acceleration_ratio > max_acceleration_peak.ratio) {
                max_acceleration_peak = {acceleration_ratio, pi, ji, point.time, point.accelerations[ji],
                    bounds.min_acceleration, bounds.max_acceleration, effective_acceleration};
            }
        }
    }
    for (std::size_t pi = 0; pi < points.size(); ++pi) {
        points[pi].time_from_start = static_cast<builtin_interfaces::msg::Duration>(
            rclcpp::Duration::from_seconds(raw_iptp_points[pi].time));
        points[pi].velocities = raw_iptp_points[pi].velocities;
        points[pi].accelerations = raw_iptp_points[pi].accelerations;
        if (points[pi].positions != pre_scale_iptp_points[pi].positions) {
            detail = "E_S1_IPTP_TIME_STRETCH_CHANGED_POSITION at point " + std::to_string(pi);
            return false;
        }
        if (pi > 0 && !(rclcpp::Duration(points[pi].time_from_start).seconds() >
                        rclcpp::Duration(points[pi - 1].time_from_start).seconds())) {
            detail = "E_S1_IPTP_TIME_STRETCH_INVALID_TIME at point " + std::to_string(pi);
            return false;
        }
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_IPTP_TIME_STRETCH] duration_before=%.9fs duration_after=%.9fs "
        "scale=%.9f margin=0.010000 Rv_before=%.9f Ra_before=%.9f "
        "Rv_after=%.9f Ra_after=%.9f max_velocity=(%s point=%zu t=%.9fs value=%.9g "
        "live=[%.9g,%.9g] effective_signed=%.9g ratio=%.9f) "
        "max_acceleration=(%s point=%zu t=%.9fs value=%.9g live=[%.9g,%.9g] "
        "effective_signed=%.9g ratio=%.9f)",
        time_scaling.duration_before, time_scaling.duration_after, time_scaling.scale,
        time_scaling.velocity_ratio_before, time_scaling.acceleration_ratio_before,
        time_scaling.velocity_ratio_after, time_scaling.acceleration_ratio_after,
        trajectory_names[max_velocity_peak.joint].c_str(), max_velocity_peak.point,
        max_velocity_peak.time, max_velocity_peak.value, max_velocity_peak.lower,
        max_velocity_peak.upper, max_velocity_peak.effective, max_velocity_peak.ratio,
        trajectory_names[max_acceleration_peak.joint].c_str(), max_acceleration_peak.point,
        max_acceleration_peak.time, max_acceleration_peak.value, max_acceleration_peak.lower,
        max_acceleration_peak.upper, max_acceleration_peak.effective, max_acceleration_peak.ratio);

    // Export both the unmodified IPTP output and the uniformly stretched message values.
    if (!ctx_->expected_run_id_.empty()) {
        std::string safe_run_id = ctx_->expected_run_id_;
        for (auto& ch : safe_run_id) {
            if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_' && ch != '-') ch = '_';
        }
        const std::string path = "experiment_results/a1_runs/" + safe_run_id +
            "/s1_iptp_trajectory_candidate_" + std::to_string(candidate_index + 1) + ".csv";
        std::ofstream output(path, std::ios::out | std::ios::trunc);
        if (output) {
            output << "source,point,time_sec,time_scale";
            for (const auto& name : trajectory_names) output << ",q_" << name;
            for (const auto& name : trajectory_names) output << ",v_" << name;
            for (const auto& name : trajectory_names) output << ",a_" << name;
            output << '\n';
            auto write_trajectory_row = [&](const char* source, std::size_t point_index,
                                            const s1::TimedJointPoint& point, double scale) {
                output << source << ',' << point_index << ',' << point.time << ',' << scale;
                for (const auto value : point.positions) output << ',' << value;
                for (const auto value : point.velocities) output << ',' << value;
                for (const auto value : point.accelerations) output << ',' << value;
                output << '\n';
            };
            for (std::size_t pi = 0; pi < pre_scale_iptp_points.size(); ++pi) {
                write_trajectory_row("IPTP_RAW", pi, pre_scale_iptp_points[pi], 1.0);
                write_trajectory_row("IPTP_STRETCHED", pi,
                    s1::TimedJointPoint{rclcpp::Duration(points[pi].time_from_start).seconds(),
                        raw_iptp_points[pi].positions, points[pi].velocities, points[pi].accelerations},
                    time_scaling.scale);
            }
        } else {
            RCLCPP_WARN(ctx_->get_logger(), "[S1_IPTP_EXPORT] could not open %s", path.c_str());
        }
    }

    moveit::core::RobotState validation_base(criterion_start);
    auto stateAt = [&](const std::vector<double>& positions) {
        moveit::core::RobotState state(validation_base);
        state.setVariablePositions(trajectory_names, positions);
        state.update();
        return state;
    };
    std::vector<std::vector<double>> geometric_joint_path;
    geometric_joint_path.reserve(coordinated_live_states.size());
    for (const auto& state : coordinated_live_states) {
        std::vector<double> q;
        q.reserve(trajectory_names.size());
        for (const auto& name : trajectory_names) q.push_back(state.getVariablePosition(name));
        geometric_joint_path.emplace_back(std::move(q));
    }
    auto distanceToGeometricPath = [&](const std::vector<double>& q) {
        double min_distance_sq = std::numeric_limits<double>::infinity();
        for (std::size_t segment = 0; segment + 1 < geometric_joint_path.size(); ++segment) {
            const auto& a = geometric_joint_path[segment];
            const auto& b = geometric_joint_path[segment + 1];
            double dot = 0.0, norm_sq = 0.0;
            for (std::size_t joint = 0; joint < q.size(); ++joint) {
                const double d = b[joint] - a[joint];
                dot += (q[joint] - a[joint]) * d;
                norm_sq += d * d;
            }
            const double alpha = norm_sq > 1e-16 ? std::clamp(dot / norm_sq, 0.0, 1.0) : 0.0;
            double distance_sq = 0.0;
            for (std::size_t joint = 0; joint < q.size(); ++joint) {
                const double nearest = a[joint] + alpha * (b[joint] - a[joint]);
                const double delta = q[joint] - nearest;
                distance_sq += delta * delta;
            }
            min_distance_sq = std::min(min_distance_sq, distance_sq);
        }
        return std::sqrt(min_distance_sq);
    };
    auto checkCoGrasp = [&](const moveit::core::RobotState& state, std::size_t index) {
        const Eigen::Isometry3d inferred_left =
            state.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
        const Eigen::Isometry3d inferred_right =
            state.getGlobalLinkTransform(a1_verify_.right_eef) * object_to_right.inverse();
        const double position_residual = (inferred_left.translation() - inferred_right.translation()).norm();
        const double angle_residual = Eigen::AngleAxisd(
            inferred_left.linear().transpose() * inferred_right.linear()).angle();
        if (!std::isfinite(position_residual) || !std::isfinite(angle_residual) ||
            position_residual > 0.005 || angle_residual > 2.0 * M_PI / 180.0) {
            last_reject = "dual hand-to-rod residual exceeds 5 mm / 2 deg at sample " +
                std::to_string(index) + " (" + std::to_string(position_residual * 1000.0) +
                " mm, " + std::to_string(angle_residual * 180.0 / M_PI) + " deg)";
            return false;
        }
        return true;
    };
    for (std::size_t ji = 0; ji < trajectory_names.size(); ++ji) {
        const double start_delta = wrappedAbsJointDelta(
            points.front().positions[ji], criterion_start.getVariablePosition(trajectory_names[ji]));
        if (start_delta > 0.002) {
            detail = "E_S1_FINAL_START_STATE_MISMATCH at " + trajectory_names[ji] +
                     " delta=" + std::to_string(start_delta) + " rad";
            return false;
        }
    }
    std::size_t checked_samples = 0;
    double max_position_residual = 0.0, max_angle_residual = 0.0;
    double max_time_geometry_deviation_rad = 0.0;
    auto validateSample = [&](const std::vector<double>& positions, const char* source,
                              std::size_t index, double alpha) {
        moveit::core::RobotState state = stateAt(positions);
        writeFkReplay(source, "IPTP", index, alpha, state);
        max_time_geometry_deviation_rad = std::max(
            max_time_geometry_deviation_rad, distanceToGeometricPath(positions));
        if (!state.satisfiesBounds(criterion_dual, 1e-4) || !checkCoGrasp(state, checked_samples)) return false;
        const Eigen::Isometry3d inferred_left =
            state.getGlobalLinkTransform(a1_verify_.left_eef) * object_to_left.inverse();
        const Eigen::Isometry3d inferred_right =
            state.getGlobalLinkTransform(a1_verify_.right_eef) * object_to_right.inverse();
        max_position_residual = std::max(max_position_residual,
            (inferred_left.translation() - inferred_right.translation()).norm());
        max_angle_residual = std::max(max_angle_residual,
            Eigen::AngleAxisd(inferred_left.linear().transpose() * inferred_right.linear()).angle());
        collision_detection::CollisionResult collision;
        if (placementProjectionMatches(inferred_left) && supportClearance(inferred_left) < -1e-4) {
            last_reject = "attached rod would penetrate the support surface";
            return false;
        }
        collision_detection::AllowedCollisionMatrix sample_acm = a1_verify_.grasp_acm;
        if (expectedSupportBand(inferred_left)) {
            sample_acm.setEntry(a1_verify_.rod_id, a1_verify_.support_id, true);
        }
        criterion_scene->checkCollision(path_collision_request, collision, state, sample_acm);
        ++checked_samples;
        if (collision.collision) {
            last_reject = collision.contacts.empty() ? "criterion collision" :
                collision.contacts.begin()->first.first + "<->" + collision.contacts.begin()->first.second;
            return false;
        }
        return true;
    };
    for (std::size_t pi = 0; pi < points.size(); ++pi) {
        const auto& point = points[pi];
        if (point.positions.size() != trajectory_names.size() ||
            (!point.velocities.empty() && point.velocities.size() != trajectory_names.size()) ||
            (!point.accelerations.empty() && point.accelerations.size() != trajectory_names.size())) {
            detail = "E_S1_FINAL_DERIVATIVE_DIMENSION at point " + std::to_string(pi);
            return false;
        }
        for (std::size_t ji = 0; ji < trajectory_names.size(); ++ji) {
            const auto& bounds = live_model->getVariableBounds(trajectory_names[ji]);
            const double effective_velocity_bound = 0.15 *
                (point.velocities[ji] >= 0.0 ? bounds.max_velocity_ : -bounds.min_velocity_);
            const double effective_acceleration_bound = 0.12 *
                (point.accelerations[ji] >= 0.0 ? bounds.max_acceleration_ : -bounds.min_acceleration_);
            if (!point.velocities.empty() && (!std::isfinite(point.velocities[ji]) ||
                !bounds.velocity_bounded_ || !(effective_velocity_bound > 0.0) ||
                std::abs(point.velocities[ji]) > effective_velocity_bound + 1e-9)) {
                detail = "E_S1_FINAL_VELOCITY_LIMIT at " + trajectory_names[ji] +
                         " point " + std::to_string(pi);
                return false;
            }
            if (!point.accelerations.empty() && (!std::isfinite(point.accelerations[ji]) ||
                !bounds.acceleration_bounded_ || !(effective_acceleration_bound > 0.0) ||
                std::abs(point.accelerations[ji]) > effective_acceleration_bound + 1e-9)) {
                detail = "E_S1_FINAL_ACCELERATION_LIMIT at " + trajectory_names[ji] +
                         " point " + std::to_string(pi);
                return false;
            }
        }
        if (!validateSample(points[pi].positions, "totg_point", pi, 1.0)) {
            detail = "E_S1_FINAL_TRAJECTORY_REJECTED at point " + std::to_string(pi) + ": " + last_reject;
            return false;
        }
        if (pi + 1 < points.size()) {
            const std::size_t subdivisions = a1v::subdivisionsFor(
                a1v::maxAbsJointDelta(points[pi].positions, points[pi + 1].positions),
                ctx_->p2b_max_joint_step_rad_);
            if (subdivisions > 2000) {
                detail = "E_S1_FINAL_SEGMENT_SUBDIVISION_LIMIT at segment " + std::to_string(pi);
                return false;
            }
            for (std::size_t si = 1; si < subdivisions; ++si) {
                const double alpha = static_cast<double>(si) / static_cast<double>(subdivisions);
                const auto interpolated = a1v::interpolateJoints(
                    points[pi].positions, points[pi + 1].positions, alpha);
                if (!validateSample(interpolated, "totg_interpolation", pi, alpha)) {
                    detail = "E_S1_FINAL_INTERPOLATION_REJECTED at segment " +
                        std::to_string(pi) + ": " + last_reject;
                    return false;
                }
            }
        }
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_PREPLAN_VALIDATED] object_waypoints=%zu trajectory_points=%zu checked_waypoint_and_interpolated_states=%zu "
        "max_dual_rod_residual=(%.3fmm,%.3fdeg) max_time_geometry_deviation=%.6frad timing=IPTP "
        "velocity_scale=0.15 acceleration_scale=0.12",
        object_path.size(), points.size(), checked_samples, max_position_residual * 1000.0,
        max_angle_residual * 180.0 / M_PI, max_time_geometry_deviation_rad);
    selected_place_out = object_place;
    candidate_plan_out = std::move(candidate_plan);
    return true;
        };
        if (planPlacementCandidate(target_x, target_y, carry_plan, object_place)) {
            selected_candidate = static_cast<int>(candidate_index);
            RCLCPP_INFO(ctx_->get_logger(),
                "[S1_CANDIDATE_ACCEPT] index=%zu x=%.3f y=%.3f full_path=validated",
                candidate_index + 1, target_x, target_y);
            break;
        }
        RCLCPP_WARN(ctx_->get_logger(),
            "[S1_CANDIDATE_REJECT] index=%zu x=%.3f y=%.3f reason=%s",
            candidate_index + 1, target_x, target_y, detail.c_str());
    }
    if (selected_candidate < 0) {
        detail = "E_S1_FIXED_PLACEMENT_NOT_FEASIBLE x=" + std::to_string(ctx_->rod_target_x_) +
                 " y=" + std::to_string(ctx_->rod_target_y_);
        RCLCPP_ERROR(ctx_->get_logger(), "[S1_CANDIDATE_SET_REJECT] %s", detail.c_str());
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_CANDIDATE_FROZEN] x=%.3f y=%.3f world_yaw_deg=%.3f rotation_tested=false "
        "planar_transport_distance=%.3fm",
        object_place.translation().x(), object_place.translation().y(),
        ctx_->rod_target_yaw_rad_ * 180.0 / M_PI,
        std::hypot(object_place.translation().x() - object_initial.translation().x(),
                   object_place.translation().y() - object_initial.translation().y()));
    const auto& points = carry_plan.trajectory_.joint_trajectory.points;
    const double planned = rclcpp::Duration(points.back().time_from_start).seconds();
    auto iface = ctx_->dual_arm_;
    auto dispatched = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(carry_plan);
    const auto result = a1v::runBoundedExec(
        [iface, dispatched]() { return iface->execute(*dispatched).val; },
        [iface]() { iface->stop(); }, std::max(30.0, 3.0 * planned + 8.0), 0.2, 0.05);
    if (result.status != a1v::BoundedExecOutcome::Status::COMPLETED ||
        result.value != moveit::core::MoveItErrorCode::SUCCESS) {
        detail = "E_S1_CARRY_EXECUTION_FAILED status=" + std::to_string(static_cast<int>(result.status)) +
                 " code=" + std::to_string(result.value);
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(), "[S1_CARRY_EXECUTED] planned=%.3fs execute=%.3fs result=SUCCESS",
                planned, result.elapsed_sec);

    MujocoRodTruth placed;
    std::string support_wait_detail;
    if (!awaitStableRodTruth("pre_release_support", placed, support_wait_detail)) {
        detail = "E_S1_SUPPORT_STABILITY_FAILED: " + support_wait_detail;
        return false;
    }
    const a1v::Vec3 target_pos{object_place.translation().x(), object_place.translation().y(),
                               object_place.translation().z()};
    const Eigen::Quaterniond target_q(object_place.linear());
    const a1t::Quat target_quat{target_q.x(), target_q.y(), target_q.z(), target_q.w()};
    const double place_pos_err = std::sqrt(std::pow(placed.position.x - target_pos.x, 2) +
        std::pow(placed.position.y - target_pos.y, 2) + std::pow(placed.position.z - target_pos.z, 2));
    const double place_angle_err = a1t::rotationDistanceRad(placed.orientation, target_quat);
    double support_clearance = 0.0;
    const bool support_height_ok = s1::supportHeightEligible(
        placed.position.z, ctx_->rod_half_height_, ctx_->p2a_support_z_,
        ctx_->p2a_resting_z_tol_, &support_clearance);
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_RELEASE_TARGET_ERROR] physical_rod=(%.5f,%.5f,%.5f) target=(%.5f,%.5f,%.5f) "
        "position_error=%.3fmm angle_error=%.3fdeg support_clearance=%.3fmm support_height_ok=%s",
        placed.position.x, placed.position.y, placed.position.z, target_pos.x, target_pos.y,
        target_pos.z, place_pos_err * 1000.0, place_angle_err * 180.0 / M_PI,
        support_clearance * 1000.0, support_height_ok ? "true" : "false");
    if (!support_height_ok) {
        detail = "E_S1_SUPPORT_NOT_CONFIRMED: rod truth is outside the support-height band or penetrates ground";
        return false;
    }
    if (place_pos_err > 0.01 || place_angle_err > 5.0 * M_PI / 180.0) {
        detail = "E_S1_PLACE_ERROR: physical rod exceeds 10 mm / 5 deg target tolerance";
        return false;
    }

    std_msgs::msg::Bool weld_off;
    weld_off.data = false;
    ctx_->weld_state_received_.store(false, std::memory_order_release);
    ctx_->weld_pub_->publish(weld_off);
    const auto off_deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < off_deadline &&
           (!ctx_->weld_state_received_.load(std::memory_order_acquire) ||
            ctx_->weld_state_active_.load(std::memory_order_acquire))) std::this_thread::sleep_for(20ms);
    if (!ctx_->weld_state_received_.load(std::memory_order_acquire) ||
        ctx_->weld_state_active_.load(std::memory_order_acquire)) {
        detail = "E_S1_WELD_RELEASE_NOT_CONFIRMED";
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(), "[S1_WELD_STATE] dual_weld_active=false confirmed");

    if (!ctx_->left_arm_->detachObject("aluminum_rod")) {
        detail = "E_S1_MOVEIT_DETACH_FAILED";
        return false;
    }
    bool detached = false;
    for (int i = 0; i < 20; ++i) {
        if (psi.getAttachedObjects({"aluminum_rod"}).count("aluminum_rod") == 0) {
            detached = true;
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    if (!detached) {
        detail = "E_S1_MOVEIT_DETACH_NOT_VISIBLE";
        return false;
    }
    moveit_msgs::msg::CollisionObject world_rod;
    world_rod.header.frame_id = ctx_->dual_arm_->getPlanningFrame();
    world_rod.id = "aluminum_rod";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {truth.size.x, truth.size.y, truth.size.z};
    geometry_msgs::msg::Pose actual_pose;
    actual_pose.position.x = placed.position.x; actual_pose.position.y = placed.position.y;
    actual_pose.position.z = placed.position.z;
    actual_pose.orientation.x = placed.orientation.x; actual_pose.orientation.y = placed.orientation.y;
    actual_pose.orientation.z = placed.orientation.z; actual_pose.orientation.w = placed.orientation.w;
    world_rod.primitives.push_back(box);
    world_rod.primitive_poses.push_back(actual_pose);
    world_rod.operation = world_rod.ADD;
    if (!psi.applyCollisionObject(world_rod)) {
        detail = "E_S1_RELEASE_WORLD_OBJECT_SYNC_FAILED";
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(), "[S1_DETACH] MoveIt attached object removed and physical pose restored as world object");

    std::array<std::uint64_t, 2> retreat_finger_baseline_seq {{0, 0}};
    DualArmCarryTask::GripperContactFeedback retreat_feedback;
    if (ctx_->latestGripperContact("mj_left", retreat_feedback)) {
        retreat_finger_baseline_seq[0] = retreat_feedback.seq;
    }
    if (ctx_->latestGripperContact("mj_right", retreat_feedback)) {
        retreat_finger_baseline_seq[1] = retreat_feedback.seq;
    }
    const bool open_ok = ctx_->grippers_->controlDualGrippers(0.04);
    RCLCPP_INFO(ctx_->get_logger(), "[S1_RELEASE] open_action=%s", open_ok ? "SUCCESS" : "FAILED");
    const double opened_after_steady_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    MujocoRodTruth released_immediate = readMujocoRodTruth(ctx_->mujoco_get_body_state_service_);
    if (released_immediate.ok) {
        RCLCPP_INFO(ctx_->get_logger(),
            "[S1_POST_RELEASE_TRUTH] xyz=(%.6f,%.6f,%.6f) quat_xyzw=(%.7f,%.7f,%.7f,%.7f)",
            released_immediate.position.x, released_immediate.position.y, released_immediate.position.z,
            released_immediate.orientation.x, released_immediate.orientation.y,
            released_immediate.orientation.z, released_immediate.orientation.w);
    } else {
        RCLCPP_ERROR(ctx_->get_logger(), "[S1_POST_RELEASE_TRUTH_UNAVAILABLE] %s",
                     released_immediate.detail.c_str());
    }
    if (!open_ok) {
        detail = "E_S1_GRIPPER_OPEN_FAILED";
        if (!released_immediate.ok) detail += "; post-release truth unavailable: " + released_immediate.detail;
        return false;
    }
    if (!released_immediate.ok) {
        detail = "E_S1_POST_RELEASE_TRUTH_UNAVAILABLE: " + released_immediate.detail;
        return false;
    }
    a1v::FingerJointSnapshot retreat_fingers;
    if (!awaitFreshOpenFingerSnapshot(opened_after_steady_sec, retreat_finger_baseline_seq,
                                      retreat_fingers, detail)) {
        detail = "E_S1_RETREAT_FINGER_FEEDBACK: " + detail;
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_RETREAT_FINGERS] seq=(%llu,%llu) stamp_ns=(%lld,%lld) q=(%.6f,%.6f,%.6f,%.6f) m",
        static_cast<unsigned long long>(retreat_fingers.source_seq[0]),
        static_cast<unsigned long long>(retreat_fingers.source_seq[1]),
        static_cast<long long>(retreat_fingers.source_stamp_ns[0]),
        static_cast<long long>(retreat_fingers.source_stamp_ns[1]),
        retreat_fingers.positions[0], retreat_fingers.positions[1],
        retreat_fingers.positions[2], retreat_fingers.positions[3]);

    // Keep MoveIt's world obstacle pose synchronized to the just-measured rod
    // after opening. This only updates the planning scene; MuJoCo remains the
    // source of physical truth and is never moved by this path.
    moveit_msgs::msg::CollisionObject released_world_rod;
    released_world_rod.header.frame_id = ctx_->dual_arm_->getPlanningFrame();
    released_world_rod.id = "aluminum_rod";
    shape_msgs::msg::SolidPrimitive released_box;
    released_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    released_box.dimensions = {truth.size.x, truth.size.y, truth.size.z};
    geometry_msgs::msg::Pose released_pose;
    released_pose.position.x = released_immediate.position.x;
    released_pose.position.y = released_immediate.position.y;
    released_pose.position.z = released_immediate.position.z;
    released_pose.orientation.x = released_immediate.orientation.x;
    released_pose.orientation.y = released_immediate.orientation.y;
    released_pose.orientation.z = released_immediate.orientation.z;
    released_pose.orientation.w = released_immediate.orientation.w;
    released_world_rod.primitives.push_back(released_box);
    released_world_rod.primitive_poses.push_back(released_pose);
    released_world_rod.operation = released_world_rod.ADD;
    if (!psi.applyCollisionObject(released_world_rod)) {
        detail = "E_S1_RETREAT_WORLD_ROD_SYNC_FAILED";
        return false;
    }

    bool retreat_ok = a1_verify_.valid && a1_verify_.model && a1_verify_.scene;
    auto retreat_start_live = ctx_->dual_arm_->getCurrentState(2.0);
    retreat_ok = retreat_ok && static_cast<bool>(retreat_start_live);
    const auto live_model = ctx_->dual_arm_->getRobotModel();
    const auto criterion_model = a1_verify_.model;
    const auto* live_dual = live_model ? live_model->getJointModelGroup(ctx_->dual_arm_group_) : nullptr;
    const auto* live_left = live_model ? live_model->getJointModelGroup(ctx_->left_arm_group_) : nullptr;
    const auto* live_right = live_model ? live_model->getJointModelGroup(ctx_->right_arm_group_) : nullptr;
    const auto* criterion_dual = criterion_model ?
        criterion_model->getJointModelGroup(ctx_->dual_arm_group_) : nullptr;
    retreat_ok = retreat_ok && live_dual && live_left && live_right && criterion_dual;
    if (!retreat_ok) {
        detail = "E_S1_RETREAT_MODEL_OR_START_UNAVAILABLE";
        return false;
    }
    if (!live_left->getSolverInstance()) {
        detail = "E_S1_RETREAT_IK_SOLVER_UNAVAILABLE side=left model=live";
        return false;
    }
    if (!live_right->getSolverInstance()) {
        detail = "E_S1_RETREAT_IK_SOLVER_UNAVAILABLE side=right model=live";
        return false;
    }

    auto get_scene_client = ctx_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!get_scene_client->wait_for_service(5s)) {
        detail = "E_S1_RETREAT_SCENE_SERVICE_UNAVAILABLE";
        return false;
    }
    auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    scene_request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto scene_future = get_scene_client->async_send_request(scene_request);
    if (scene_future.wait_for(5s) != std::future_status::ready) {
        detail = "E_S1_RETREAT_SCENE_TIMEOUT";
        return false;
    }
    const auto scene_response = scene_future.get();
    bool scene_has_world_rod = false;
    for (const auto& world_object : scene_response->scene.world.collision_objects) {
        if (world_object.id == "aluminum_rod" && !world_object.primitive_poses.empty()) {
            scene_has_world_rod = true;
            break;
        }
    }
    for (const auto& attached : scene_response->scene.robot_state.attached_collision_objects) {
        if (attached.object.id == "aluminum_rod") scene_has_world_rod = false;
    }
    if (!scene_has_world_rod) {
        detail = "E_S1_RETREAT_WORLD_ROD_NOT_DETACHED";
        return false;
    }

    planning_scene::PlanningScene live_scene(live_model);
    live_scene.setPlanningSceneMsg(scene_response->scene);
    auto criterion_scene = std::make_shared<planning_scene::PlanningScene>(criterion_model);
    criterion_scene->setPlanningSceneMsg(scene_response->scene);

    moveit::core::RobotState retreat_start_criterion(criterion_model);
    moveit::core::RobotState retreat_start_live_measured(*retreat_start_live);
    const std::set<std::string> criterion_variables(
        criterion_model->getVariableNames().begin(), criterion_model->getVariableNames().end());
    const std::set<std::string> live_variables(
        live_model->getVariableNames().begin(), live_model->getVariableNames().end());
    for (const auto& name : retreat_start_live->getVariableNames()) {
        if (criterion_variables.count(name) != 0) {
            retreat_start_criterion.setVariablePosition(name, retreat_start_live->getVariablePosition(name));
        }
    }
    const auto& finger_names = a1v::fingerJointNames();
    for (std::size_t i = 0; i < finger_names.size(); ++i) {
        retreat_start_criterion.setVariablePosition(finger_names[i], retreat_fingers.positions[i]);
        retreat_start_live_measured.setVariablePosition(finger_names[i], retreat_fingers.positions[i]);
    }
    retreat_start_criterion.update();
    retreat_start_live_measured.update();
    if (!retreat_start_criterion.satisfiesBounds(criterion_dual, 1e-4)) {
        detail = "E_S1_RETREAT_START_BOUNDS";
        return false;
    }

    collision_detection::AllowedCollisionMatrix retreat_acm = a1_verify_.base_acm;
    retreat_acm.setEntry("mj_left_hand", "aluminum_rod", false);
    retreat_acm.setEntry("mj_right_hand", "aluminum_rod", false);
    retreat_acm.setEntry("mj_left_leftfinger", "aluminum_rod", false);
    retreat_acm.setEntry("mj_left_rightfinger", "aluminum_rod", false);
    retreat_acm.setEntry("mj_right_leftfinger", "aluminum_rod", false);
    retreat_acm.setEntry("mj_right_rightfinger", "aluminum_rod", false);
    double retreat_support_clearance =
        released_immediate.position.z - ctx_->rod_half_height_ - ctx_->p2a_support_z_;
    if (ctx_->p2a_support_plane_ && retreat_support_clearance >= -1e-4 &&
        s1::supportHeightEligible(released_immediate.position.z, ctx_->rod_half_height_,
            ctx_->p2a_support_z_, ctx_->p2a_resting_z_tol_)) {
        retreat_acm.setEntry("aluminum_rod", a1_verify_.support_id, true);
    }
    collision_detection::CollisionRequest retreat_collision_request;
    retreat_collision_request.group_name = ctx_->dual_arm_group_;
    retreat_collision_request.contacts = true;
    retreat_collision_request.max_contacts = 12;
    auto firstContact = [](const collision_detection::CollisionResult& result) {
        return result.contacts.empty() ? std::string("none") :
            result.contacts.begin()->first.first + "<->" + result.contacts.begin()->first.second;
    };
    collision_detection::CollisionResult live_start_collision;
    live_scene.checkCollision(retreat_collision_request, live_start_collision,
                              retreat_start_live_measured, retreat_acm);
    collision_detection::CollisionResult criterion_start_collision;
    criterion_scene->checkCollision(retreat_collision_request, criterion_start_collision,
                                    retreat_start_criterion, retreat_acm);
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_RETREAT_START_COMPARE] source=live_current_state_with_measured_fingers rod_source=post_release_MuJoCo "
        "criterion=mujoco_hand_aabb fingers_seq=(%llu,%llu) q=(%.6f,%.6f,%.6f,%.6f) "
        "rod_xyz=(%.6f,%.6f,%.6f) support_clearance=%.3fmm "
        "live_collision=%s live_pair=%s criterion_collision=%s criterion_pair=%s",
        static_cast<unsigned long long>(retreat_fingers.source_seq[0]),
        static_cast<unsigned long long>(retreat_fingers.source_seq[1]),
        retreat_fingers.positions[0], retreat_fingers.positions[1],
        retreat_fingers.positions[2], retreat_fingers.positions[3],
        released_immediate.position.x, released_immediate.position.y, released_immediate.position.z,
        retreat_support_clearance * 1000.0,
        live_start_collision.collision ? "true" : "false", firstContact(live_start_collision).c_str(),
        criterion_start_collision.collision ? "true" : "false",
        firstContact(criterion_start_collision).c_str());
    if (criterion_start_collision.collision) {
        detail = "E_S1_RETREAT_CRITERION_START_COLLISION: " + firstContact(criterion_start_collision);
        return false;
    }

    auto poseFromEigen = [](const Eigen::Isometry3d& tf) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.translation().x(); pose.position.y = tf.translation().y();
        pose.position.z = tf.translation().z();
        const Eigen::Quaterniond q(tf.linear());
        pose.orientation.x = q.x(); pose.orientation.y = q.y();
        pose.orientation.z = q.z(); pose.orientation.w = q.w();
        return pose;
    };
    auto criterionArmValues = [&](const moveit::core::RobotState& state) {
        std::vector<double> values;
        state.copyJointGroupPositions(criterion_dual, values);
        return values;
    };
    auto toCriterionState = [&](const moveit::core::RobotState& live_state) {
        moveit::core::RobotState mapped(criterion_model);
        for (const auto& name : live_state.getVariableNames()) {
            if (criterion_variables.count(name) != 0) {
                mapped.setVariablePosition(name, live_state.getVariablePosition(name));
            }
        }
        mapped.update();
        return mapped;
    };
    const Eigen::Isometry3d left_start_pose =
        retreat_start_criterion.getGlobalLinkTransform(a1_verify_.left_eef);
    const Eigen::Isometry3d right_start_pose =
        retreat_start_criterion.getGlobalLinkTransform(a1_verify_.right_eef);
    Eigen::Isometry3d left_goal_pose = left_start_pose;
    Eigen::Isometry3d right_goal_pose = right_start_pose;
    left_goal_pose.translation().z() += 0.10;
    right_goal_pose.translation().z() += 0.10;
    std::vector<moveit::core::RobotState> retreat_states;
    std::vector<moveit::core::RobotState> retreat_live_states;
    std::vector<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> retreat_targets;
    retreat_states.reserve(21);
    retreat_live_states.reserve(21);
    retreat_targets.reserve(21);
    retreat_states.push_back(retreat_start_criterion);
    retreat_live_states.push_back(retreat_start_live_measured);
    retreat_targets.emplace_back(left_start_pose, right_start_pose);
    auto checkRetreatCriterionState = [&](const moveit::core::RobotState& state,
                                          const Eigen::Isometry3d& expected_left,
                                          const Eigen::Isometry3d& expected_right,
                                          const std::size_t index) {
        const Eigen::Isometry3d actual_left = state.getGlobalLinkTransform(a1_verify_.left_eef);
        const Eigen::Isometry3d actual_right = state.getGlobalLinkTransform(a1_verify_.right_eef);
        const double left_pos = (actual_left.translation() - expected_left.translation()).norm();
        const double right_pos = (actual_right.translation() - expected_right.translation()).norm();
        const double left_ang = Eigen::AngleAxisd(
            expected_left.linear().transpose() * actual_left.linear()).angle();
        const double right_ang = Eigen::AngleAxisd(
            expected_right.linear().transpose() * actual_right.linear()).angle();
        if (!std::isfinite(left_pos) || !std::isfinite(right_pos) ||
            left_pos > 0.005 || right_pos > 0.005 ||
            left_ang > 2.0 * M_PI / 180.0 || right_ang > 2.0 * M_PI / 180.0) {
            detail = "E_S1_RETREAT_HAND_POSE_RESIDUAL at sample " + std::to_string(index);
            return false;
        }
        if (!state.satisfiesBounds(criterion_dual, 1e-4)) {
            detail = "E_S1_RETREAT_JOINT_BOUNDS at sample " + std::to_string(index);
            return false;
        }
        collision_detection::AllowedCollisionMatrix sample_acm = retreat_acm;
        collision_detection::CollisionResult collision;
        criterion_scene->checkCollision(retreat_collision_request, collision, state, sample_acm);
        if (collision.collision) {
            detail = "E_S1_RETREAT_COLLISION at sample " + std::to_string(index) + ": " +
                firstContact(collision);
            return false;
        }
        return true;
    };
    if (!checkRetreatCriterionState(retreat_states.front(), left_start_pose, right_start_pose, 0)) {
        return false;
    }
    constexpr std::size_t kRetreatSteps = 20;
    for (std::size_t i = 1; i <= kRetreatSteps; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(kRetreatSteps);
        Eigen::Isometry3d desired_left = left_start_pose;
        Eigen::Isometry3d desired_right = right_start_pose;
        desired_left.translation().z() += 0.10 * alpha;
        desired_right.translation().z() += 0.10 * alpha;
        moveit::core::RobotState candidate_live(retreat_live_states.back());
        const bool left_ik = candidate_live.setFromIK(
            live_left, poseFromEigen(desired_left), 0.5);
        if (!left_ik) {
            detail = "E_S1_RETREAT_IK_NO_SOLUTION side=left waypoint=" + std::to_string(i);
            return false;
        }
        const bool right_ik = candidate_live.setFromIK(
            live_right, poseFromEigen(desired_right), 0.5);
        if (!right_ik) {
            detail = "E_S1_RETREAT_IK_NO_SOLUTION side=right waypoint=" + std::to_string(i);
            return false;
        }
        candidate_live.update();
        moveit::core::RobotState candidate = toCriterionState(candidate_live);
        if (!checkRetreatCriterionState(candidate, desired_left, desired_right, i)) return false;
        const auto& previous = retreat_states.back();
        const std::size_t subdivisions = a1v::subdivisionsFor(
            a1v::maxAbsJointDelta(criterionArmValues(previous), criterionArmValues(candidate)), 0.01);
        for (std::size_t si = 1; si < subdivisions; ++si) {
            const double segment_alpha = static_cast<double>(si) / static_cast<double>(subdivisions);
            const auto q = a1v::interpolateJoints(
                criterionArmValues(previous), criterionArmValues(candidate), segment_alpha);
            moveit::core::RobotState interpolated(previous);
            interpolated.setJointGroupPositions(criterion_dual, q);
            interpolated.update();
            const Eigen::Isometry3d expected_left = interpolateObject(
                retreat_targets.back().first, desired_left, segment_alpha);
            const Eigen::Isometry3d expected_right = interpolateObject(
                retreat_targets.back().second, desired_right, segment_alpha);
            if (!checkRetreatCriterionState(interpolated, expected_left, expected_right,
                                            i * subdivisions + si)) return false;
        }
        retreat_states.push_back(candidate);
        retreat_live_states.push_back(candidate_live);
        retreat_targets.emplace_back(desired_left, desired_right);
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_RETREAT_GEOMETRY_VALIDATED] model=mujoco_hand_aabb waypoints=%zu max_step=5.0mm "
        "height=0.100m fixed_hand_orientation=true fixed_co_grasp_constraint=false",
        retreat_states.size());

    auto toLiveState = [&](const moveit::core::RobotState& criterion_state) {
        moveit::core::RobotState mapped(*retreat_start_live);
        for (const auto& name : criterion_state.getVariableNames()) {
            if (live_variables.count(name) != 0) {
                mapped.setVariablePosition(name, criterion_state.getVariablePosition(name));
            }
        }
        mapped.update();
        return mapped;
    };
    robot_trajectory::RobotTrajectory retreat_trajectory(live_model, ctx_->dual_arm_group_);
    for (const auto& state : retreat_states) retreat_trajectory.addSuffixWayPoint(toLiveState(state), 0.0);
    trajectory_processing::IterativeParabolicTimeParameterization retreat_iptp;
    if (!retreat_iptp.computeTimeStamps(retreat_trajectory, 0.10, 0.08)) {
        detail = "E_S1_RETREAT_IPTP_FAILED";
        return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    retreat_trajectory.getRobotTrajectoryMsg(retreat_plan.trajectory_);
    moveit::core::robotStateToRobotStateMsg(*retreat_start_live, retreat_plan.start_state_);
    auto& retreat_points = retreat_plan.trajectory_.joint_trajectory.points;
    const auto& retreat_joint_names = retreat_plan.trajectory_.joint_trajectory.joint_names;
    std::vector<s1::DirectionalJointLimits> retreat_limits;
    retreat_limits.reserve(retreat_joint_names.size());
    for (const auto& name : retreat_joint_names) {
        const auto& bounds = live_model->getVariableBounds(name);
        retreat_limits.push_back({bounds.min_velocity_, bounds.max_velocity_,
            bounds.min_acceleration_, bounds.max_acceleration_,
            bounds.velocity_bounded_, bounds.acceleration_bounded_});
    }
    std::vector<s1::TimedJointPoint> retreat_timed_points;
    for (const auto& point : retreat_points) {
        retreat_timed_points.push_back({rclcpp::Duration(point.time_from_start).seconds(),
            point.positions, point.velocities, point.accelerations});
    }
    s1::TimeScalingReport retreat_timing_report;
    std::string retreat_timing_error;
    if (!s1::scaleTimedTrajectory(retreat_timed_points, retreat_limits, 0.10, 0.08, 0.01,
                                  retreat_timing_report, retreat_timing_error)) {
        detail = "E_S1_RETREAT_TIMING_INVALID: " + retreat_timing_error;
        return false;
    }
    for (std::size_t i = 0; i < retreat_points.size(); ++i) {
        retreat_points[i].time_from_start = static_cast<builtin_interfaces::msg::Duration>(
            rclcpp::Duration::from_seconds(retreat_timed_points[i].time));
        retreat_points[i].velocities = retreat_timed_points[i].velocities;
        retreat_points[i].accelerations = retreat_timed_points[i].accelerations;
    }
    auto execution_start = ctx_->dual_arm_->getCurrentState(2.0);
    if (!execution_start) {
        detail = "E_S1_RETREAT_EXECUTION_START_UNAVAILABLE";
        return false;
    }
    double max_retreat_start_delta = 0.0;
    for (std::size_t j = 0; j < retreat_joint_names.size(); ++j) {
        max_retreat_start_delta = std::max(max_retreat_start_delta, std::abs(
            retreat_points.front().positions[j] - execution_start->getVariablePosition(retreat_joint_names[j])));
    }
    if (max_retreat_start_delta > 0.02) {
        detail = "E_S1_RETREAT_START_MISMATCH delta=" + std::to_string(max_retreat_start_delta);
        return false;
    }

    moveit::core::RobotState retreat_validation_base(retreat_start_criterion);
    auto retreatStateAt = [&](const std::vector<double>& positions) {
        moveit::core::RobotState state(retreat_validation_base);
        state.setVariablePositions(retreat_joint_names, positions);
        state.update();
        return state;
    };
    std::size_t retreat_checked = 0;
    auto validateRetreatMessageState = [&](const std::vector<double>& positions,
                                           const std::size_t point_index,
                                           const double alpha) {
        auto state = retreatStateAt(positions);
        const Eigen::Isometry3d expected_left = interpolateObject(
            retreat_targets[point_index].first,
            point_index + 1 < retreat_targets.size() ? retreat_targets[point_index + 1].first :
                                                       retreat_targets[point_index].first,
            alpha);
        const Eigen::Isometry3d expected_right = interpolateObject(
            retreat_targets[point_index].second,
            point_index + 1 < retreat_targets.size() ? retreat_targets[point_index + 1].second :
                                                       retreat_targets[point_index].second,
            alpha);
        if (!checkRetreatCriterionState(state, expected_left, expected_right, retreat_checked)) return false;
        ++retreat_checked;
        return true;
    };
    if (retreat_points.size() != retreat_targets.size()) {
        detail = "E_S1_RETREAT_FINAL_POINT_COUNT_MISMATCH";
        return false;
    }
    for (std::size_t i = 0; i < retreat_points.size(); ++i) {
        if (!validateRetreatMessageState(retreat_points[i].positions, i, 0.0)) return false;
        if (i + 1 < retreat_points.size()) {
            const std::size_t subdivisions = a1v::subdivisionsFor(
                a1v::maxAbsJointDelta(retreat_points[i].positions, retreat_points[i + 1].positions), 0.01);
            for (std::size_t si = 1; si < subdivisions; ++si) {
                const double alpha = static_cast<double>(si) / static_cast<double>(subdivisions);
                const auto q = a1v::interpolateJoints(
                    retreat_points[i].positions, retreat_points[i + 1].positions, alpha);
                if (!validateRetreatMessageState(q, i, alpha)) return false;
            }
        }
    }
    const double retreat_duration = rclcpp::Duration(
        retreat_points.back().time_from_start).seconds();
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_RETREAT_PREPLAN_VALIDATED] model=mujoco_hand_aabb points=%zu checked=%zu "
        "start_delta=%.6frad duration=%.6fs scale=%.9f Rv=%.6f Ra=%.6f",
        retreat_points.size(), retreat_checked, max_retreat_start_delta, retreat_duration,
        retreat_timing_report.scale, retreat_timing_report.velocity_ratio_after,
        retreat_timing_report.acceleration_ratio_after);
    auto dispatched_retreat = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(retreat_plan);
    const auto retreat_result = a1v::runBoundedExec(
        [iface, dispatched_retreat]() { return iface->execute(*dispatched_retreat).val; },
        [iface]() { iface->stop(); }, std::max(15.0, 3.0 * retreat_duration + 5.0), 0.2, 0.05);
    retreat_ok = retreat_result.status == a1v::BoundedExecOutcome::Status::COMPLETED &&
                 retreat_result.value == moveit::core::MoveItErrorCode::SUCCESS;
    if (retreat_ok) {
        auto arrived = ctx_->dual_arm_->getCurrentState(2.0);
        if (!arrived) retreat_ok = false;
        else {
            const auto left_arrived = arrived->getGlobalLinkTransform(a1_verify_.left_eef);
            const auto right_arrived = arrived->getGlobalLinkTransform(a1_verify_.right_eef);
            const double left_error = (left_arrived.translation() - left_goal_pose.translation()).norm();
            const double right_error = (right_arrived.translation() - right_goal_pose.translation()).norm();
            const double left_angle = Eigen::AngleAxisd(
                left_goal_pose.linear().transpose() * left_arrived.linear()).angle();
            const double right_angle = Eigen::AngleAxisd(
                right_goal_pose.linear().transpose() * right_arrived.linear()).angle();
            retreat_ok = left_error <= 0.005 && right_error <= 0.005 &&
                left_angle <= 2.0 * M_PI / 180.0 && right_angle <= 2.0 * M_PI / 180.0;
            RCLCPP_INFO(ctx_->get_logger(),
                "[S1_RETREAT_ARRIVED] left_pos=%.3fmm left_angle=%.3fdeg "
                "right_pos=%.3fmm right_angle=%.3fdeg",
                left_error * 1000.0, left_angle * 180.0 / M_PI,
                right_error * 1000.0, right_angle * 180.0 / M_PI);
        }
    }
    RCLCPP_INFO(ctx_->get_logger(), "[S1_RETREAT] planned=%.3fs execute=%.3fs result=%s",
        retreat_duration, retreat_result.elapsed_sec, retreat_ok ? "SUCCESS" : "FAILED");
    if (!retreat_ok) {
        detail = "E_S1_RETREAT_FAILED";
        return false;
    }
    MujocoRodTruth released;
    std::string release_wait_detail;
    if (!awaitStableRodTruth("released_open_retreat", released, release_wait_detail)) {
        detail = "E_S1_RELEASE_STABILITY_FAILED: " + release_wait_detail;
        return false;
    }
    const bool weld_off_confirmed = ctx_->weld_state_received_.load(std::memory_order_acquire) &&
        !ctx_->weld_state_active_.load(std::memory_order_acquire);
    const bool attach_removed = psi.getAttachedObjects({"aluminum_rod"}).count("aluminum_rod") == 0;
    double released_support_clearance = 0.0;
    const bool released_support_ok = s1::supportHeightEligible(
        released.position.z, ctx_->rod_half_height_, ctx_->p2a_support_z_,
        ctx_->p2a_resting_z_tol_, &released_support_clearance);
    const double final_pos_err = std::sqrt(std::pow(released.position.x - target_pos.x, 2) +
        std::pow(released.position.y - target_pos.y, 2) + std::pow(released.position.z - target_pos.z, 2));
    const double final_ang_err = a1t::rotationDistanceRad(released.orientation, target_quat);
    const double final_q_norm = std::sqrt(released.orientation.x * released.orientation.x +
        released.orientation.y * released.orientation.y + released.orientation.z * released.orientation.z +
        released.orientation.w * released.orientation.w);
    RCLCPP_INFO(ctx_->get_logger(),
        "[S1_RELEASED_TRUTH] final_xyz=(%.6f,%.6f,%.6f) final_quat_xyzw=(%.7f,%.7f,%.7f,%.7f) "
        "target_xyz=(%.6f,%.6f,%.6f) target_quat_xyzw=(%.7f,%.7f,%.7f,%.7f) "
        "position_error=%.3fmm angle_error=%.3fdeg q_norm=%.7f support_clearance=%.3fmm "
        "support_ok=%s weld_off=%s attach_removed=%s open=%s retreat=SUCCESS",
        released.position.x, released.position.y, released.position.z,
        released.orientation.x, released.orientation.y, released.orientation.z, released.orientation.w,
        target_pos.x, target_pos.y, target_pos.z,
        target_q.x(), target_q.y(), target_q.z(), target_q.w(),
        final_pos_err * 1000.0, final_ang_err * 180.0 / M_PI, final_q_norm,
        released_support_clearance * 1000.0, released_support_ok ? "true" : "false",
        weld_off_confirmed ? "true" : "false", attach_removed ? "true" : "false",
        open_ok ? "SUCCESS" : "FAILED");
    if (!weld_off_confirmed || !attach_removed || !released_support_ok) {
        detail = "E_S1_RELEASE_STATE_INCOMPLETE: weld/attach/support confirmation failed after retreat";
        return false;
    }
    if (final_pos_err > 0.01 || final_ang_err > 5.0 * M_PI / 180.0) {
        detail = "E_S1_RELEASE_DRIFT: rod moved outside 10 mm / 5 deg after opening and retreat";
        return false;
    }
    detail = "ideal_weld baseline complete; rigid ideal grasp only, friction grip/slip unverified";
    return true;
}

bool DualArmController::runGraspDescent(const a1t::TruthPayload& truth, std::string& detail)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(ctx_->get_logger(),
                "\n----- [P3A-2B] 从预抓取位姿下降到抓取位姿（到位即停，不闭合/不 weld/不 attach）-----");

    // ---- 0. Gates. The descent is only allowed on top of a pre-grasp this
    //         process actually reached, using this round's verified context.
    //         `descent_start` is deliberately NOT checked here: it is an OUTPUT
    //         of buildAndVerifyGraspDescent, not a precondition of it.
    if (!a1_verify_.valid || !a1_verify_.model || !a1_verify_.scene) {
        detail = "E_2B_NO_VERIFY_CONTEXT: the P2a criterion model/scene is unavailable";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    if (!a1_verify_.pregrasp_reached_this_round) {
        detail = "E_2B_NO_PREGRASP: the pre-grasp pose was not reached in this process, so "
                 "there is nothing to descend from";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    if (!checkControllerStatesEnabled(detail)) {
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }

    // ---- 1. Open both grippers first, on real feedback, BEFORE descending.
    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-2B] 下降前先打开双爪（单指位移 %.4f ⇒ 目标开口 %.4f m），"
                "并要求真实反馈到位",
                ctx_->gripper_open_position_, 2.0 * ctx_->gripper_open_position_);
    std::array<std::uint64_t, 2> finger_seq_before_open {{0, 0}};
    DualArmCarryTask::GripperContactFeedback finger_before_open;
    if (ctx_->latestGripperContact("mj_left", finger_before_open)) {
        finger_seq_before_open[0] = finger_before_open.seq;
    }
    if (ctx_->latestGripperContact("mj_right", finger_before_open)) {
        finger_seq_before_open[1] = finger_before_open.seq;
    }
    if (!ctx_->grippers_->controlDualGrippers(ctx_->gripper_open_position_)) {
        detail = "E_2B_GRIPPER_OPEN_FAILED: the grippers did not open on real feedback (or an "
                 "earlier goal's outcome is still unknown); refusing to descend";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(), "  ✓ [P3A-2B] 双爪打开并实测到位，允许下降");
    const double opened_after_steady_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    a1v::FingerJointSnapshot fingers;
    if (!awaitFreshOpenFingerSnapshot(opened_after_steady_sec, finger_seq_before_open,
                                      fingers, detail)) {
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s; refusing to plan/descend", detail.c_str());
        return false;
    }

    // ---- 2. Physical rod truth BEFORE the descent, from MuJoCo.
    //
    // The descent is aimed at a grasp pose derived from this round's FROZEN
    // reference. If the physical rod has moved since, that pose no longer
    // applies -- so the reference is checked, not merely printed. A measurement
    // that cannot be taken is a failure, not a warning: "unavailable" must never
    // mean "assume it is fine".
    const std::string truth_service =
        ctx_->descent_negative_truth_ == "unavailable" ? "/a1_nonexistent_get_body_state"
                                                       : ctx_->mujoco_get_body_state_service_;
    const MujocoRodTruth before_phys = readMujocoRodTruth(truth_service);
    a1v::Vec3 scene_before_pos;
    a1v::Quat scene_before_quat;
    const bool scene_before_ok =
        readSceneRodPose(a1_verify_.rod_id, scene_before_pos, scene_before_quat);
    {
        a1v::Vec3 frozen{truth.position.x, truth.position.y, truth.position.z};
        a1v::Quat frozen_q{truth.orientation.x, truth.orientation.y, truth.orientation.z,
                           truth.orientation.w};
        if (ctx_->descent_negative_truth_ == "offset") {
            // Test-only: perturb the reference the comparison uses, so the
            // "the frozen reference no longer applies" path can be shown to fire.
            frozen.z += 0.05;
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [P3A-2B][负例-注入] 已把比较用的冻结参考抬高 50 mm（仅验证用）");
        }
        if (!before_phys.ok) {
            detail = "E_2B_TRUTH_UNAVAILABLE: the physical rod could not be measured before the "
                     "descent (" + before_phys.detail + "); refusing to descend";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        const double dpos = std::sqrt(std::pow(before_phys.position.x - frozen.x, 2) +
                                      std::pow(before_phys.position.y - frozen.y, 2) +
                                      std::pow(before_phys.position.z - frozen.z, 2));
        const double dang = a1t::rotationDistanceRad(frozen_q, before_phys.orientation);
        const double ang_tol = ctx_->descent_truth_angle_tol_deg_ * M_PI / 180.0;
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 下降前 **MuJoCo 物理真值** 杆位姿: (%.6f, %.6f, %.6f) q=(%.5f, "
                    "%.5f, %.5f, %.5f)",
                    before_phys.position.x, before_phys.position.y, before_phys.position.z,
                    before_phys.orientation.x, before_phys.orientation.y,
                    before_phys.orientation.z, before_phys.orientation.w);
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 冻结参考仍适用性核对（物理真值 vs 本轮冻结真值）: 位置差 "
                    "%.4f mm（容差 %.1f mm），角度差 %.4f°（容差 %.1f°）",
                    dpos * 1000.0, ctx_->descent_truth_pos_tol_m_ * 1000.0,
                    dang * 180.0 / M_PI, ctx_->descent_truth_angle_tol_deg_);
        if (dpos > ctx_->descent_truth_pos_tol_m_ || dang > ang_tol) {
            detail = "E_2B_TRUTH_REFERENCE_STALE: the physical rod is no longer where this round "
                     "froze it (position " + std::to_string(dpos * 1000.0) + " mm, angle " +
                     std::to_string(dang * 180.0 / M_PI) + " deg); the descent target does not "
                     "apply, refusing to descend";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
    }

    // ---- 1b. Build and verify the descent, from where the robot ACTUALLY is
    //          right now. Nothing is dispatched by this call.
    if (!buildAndVerifyGraspDescent(truth, fingers, detail)) {
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] 下降路径未通过校验，不下发运动：%s",
                     detail.c_str());
        return false;
    }
    if (!a1_verify_.descent_plan_valid) {
        detail = "E_2B_NO_VERIFIED_DESCENT: the descent was not verified";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }

    // ---- 3. Dispatch the verified descent.
    const auto& traj = a1_verify_.descent_plan.trajectory_.joint_trajectory;
    const double planned_sec =
        traj.points.empty() ? 0.0 : rclcpp::Duration(traj.points.back().time_from_start).seconds();

    // The robot must still be where the path starts, and that start must be the
    // path's own first point.
    {
        const moveit::core::RobotStatePtr cur = ctx_->dual_arm_->getCurrentState();
        double worst = 0.0;
        for (std::size_t i = 0; i < traj.joint_names.size(); ++i) {
            worst = std::max(worst, std::fabs(cur->getVariablePosition(traj.joint_names[i]) -
                                              traj.points.front().positions[i]));
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 当前关节状态与下降轨迹首点最大差 %.6f rad（阈值 0.020）", worst);
        if (worst > 0.02) {
            detail = "E_2B_START_MISMATCH: the robot is no longer at the descent path's first "
                     "point (max delta " + std::to_string(worst) + " rad)";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
            return false;
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> iface = ctx_->dual_arm_;
    moveit::planning_interface::MoveGroupInterface::Plan dispatched = a1_verify_.descent_plan;

    RCLCPP_INFO(ctx_->get_logger(),
                "  [P3A-2B] 下发下降轨迹: %zu 点，计划时长 %.3f s，速度缩放 %.3f / 加速度缩放 %.3f",
                traj.points.size(), planned_sec, ctx_->pregrasp_vel_scale_,
                ctx_->pregrasp_acc_scale_);

    const a1v::BoundedExecOutcome exec = a1v::runBoundedExec(
        [iface, dispatched]() mutable { return iface->execute(dispatched).val; },
        [iface]() { iface->stop(); }, ctx_->pregrasp_exec_timeout_sec_, 4.0, 0.05);

    if (exec.status == a1v::BoundedExecOutcome::Status::TIMEOUT) {
        RCLCPP_ERROR(ctx_->get_logger(),
                     "  ✗ [P3A-2B] E_2B_EXEC_TIMEOUT: 执行在 %.1f s 内未结束（计划时长 %.3f s）；"
                     "迟到结果不参与判定，本轮以失败终止，不重试",
                     exec.elapsed_sec, planned_sec);
        detail = "E_2B_EXEC_TIMEOUT: execution did not finish within " +
                 std::to_string(ctx_->pregrasp_exec_timeout_sec_) + " s";
        return false;
    }
    if (exec.status == a1v::BoundedExecOutcome::Status::EXCEPTION) {
        detail = "E_2B_EXEC_EXCEPTION: the execution call raised an exception";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    RCLCPP_INFO(ctx_->get_logger(), "  [P3A-2B] 执行返回码 %d，用时 %.3f s（计划时长 %.3f s）",
                exec.value, exec.elapsed_sec, planned_sec);
    if (exec.value != moveit::core::MoveItErrorCode::SUCCESS) {
        detail = "E_2B_EXEC_FAILED: the controller reported error code " +
                 std::to_string(exec.value);
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }
    if (exec.elapsed_sec > std::max(3.0 * planned_sec, 5.0)) {
        detail = "E_2B_EXEC_OVERRUN: execution took " + std::to_string(exec.elapsed_sec) +
                 " s for a " + std::to_string(planned_sec) + " s plan";
        RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
        return false;
    }

    // ---- 4. Arrival: a short window of PROVABLY FRESH feedback. ------------
    //
    // Every sample must carry a joint_states sequence number not seen before, so
    // repeatedly reading one cached state cannot fill the window. The settle
    // test is the whole window's per-joint spread, and the end-effector error is
    // read from the newest fresh sample -- not from a separate, later read.
    {
        const FreshArrival arrival = awaitFreshArrival(
            traj.joint_names, ctx_->pregrasp_stable_window_sec_, ctx_->pregrasp_stable_tol_rad_);
        if (!arrival.ok) {
            detail = "E_2B_" + arrival.reason_code + ": " + arrival.detail;
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] 到位窗口（**新鲜反馈**）: %zu 个样本 / %.3f s（序号 %llu），"
                    "窗口内最大单关节变化 %.6f rad（%s，阈值 %.6f）",
                    arrival.samples, arrival.window_sec, arrival.last_seq,
                    arrival.worst_joint_change_rad,
                    arrival.worst_joint.empty() ? "-" : arrival.worst_joint.c_str(),
                    ctx_->pregrasp_stable_tol_rad_);

        moveit::core::RobotState arrived(ctx_->dual_arm_->getRobotModel());
        if (!freshStateFor(arrival.last_positions, arrived)) {
            detail = "E_2B_ARRIVAL_STATE_INVALID: the fresh sample could not build a robot state";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ [P3A-2B] %s", detail.c_str());
            return false;
        }
        const a1v::ArmTarget got_left =
            fromEigenPose(arrived.getGlobalLinkTransform(a1_verify_.left_eef));
        const a1v::ArmTarget got_right =
            fromEigenPose(arrived.getGlobalLinkTransform(a1_verify_.right_eef));
        const a1v::PoseError el = a1v::poseError(a1_verify_.grasp_target.left, got_left);
        const a1v::PoseError er = a1v::poseError(a1_verify_.grasp_target.right, got_right);

        RCLCPP_INFO(ctx_->get_logger(),
                    "     左臂 位置误差 %.4f mm，姿态误差 %.4f°  |  右臂 位置误差 %.4f mm，"
                    "姿态误差 %.4f°",
                    el.pos_m * 1000.0, el.angle_rad * 180.0 / M_PI, er.pos_m * 1000.0,
                    er.angle_rad * 180.0 / M_PI);

        // Straightness of the PLANNED waypoints, measured on the verified plan.
        // This is NOT a measurement of the executed trajectory: no control-rate
        // trace of the motion is collected, and the record must say so.
        std::vector<a1v::ArmTarget> planned_left;
        std::vector<a1v::ArmTarget> planned_right;
        for (const auto& wp : a1_verify_.descent_waypoints) {
            planned_left.push_back(fromEigenPose(wp->getGlobalLinkTransform(a1_verify_.left_eef)));
            planned_right.push_back(fromEigenPose(wp->getGlobalLinkTransform(a1_verify_.right_eef)));
        }
        const a1v::StraightDescentDeviation dl = a1v::straightDescentDeviation(planned_left);
        const a1v::StraightDescentDeviation dr = a1v::straightDescentDeviation(planned_right);
        RCLCPP_INFO(ctx_->get_logger(),
                    "  [P3A-2B] **规划航点**竖直性（对每臂首点的 XY 与姿态）: 左 max XY %.4f mm / "
                    "max 姿态 %.4f°；右 max XY %.4f mm / max 姿态 %.4f° —— 这是**规划航点的 FK "
                    "偏差**，不是执行轨迹的实测偏差",
                    dl.max_xy_m * 1000.0, dl.max_angle_rad * 180.0 / M_PI,
                    dr.max_xy_m * 1000.0, dr.max_angle_rad * 180.0 / M_PI);

        const double ang_tol = ctx_->pregrasp_arrival_angle_tol_deg_ * M_PI / 180.0;
        if (el.pos_m > ctx_->pregrasp_arrival_pos_tol_m_ ||
            er.pos_m > ctx_->pregrasp_arrival_pos_tol_m_ || el.angle_rad > ang_tol ||
            er.angle_rad > ang_tol) {
            detail = "E_2B_NOT_AT_GRASP_POSE: left " + std::to_string(el.pos_m * 1000.0) +
                     " mm/" + std::to_string(el.angle_rad * 180.0 / M_PI) + " deg, right " +
                     std::to_string(er.pos_m * 1000.0) + " mm/" +
                     std::to_string(er.angle_rad * 180.0 / M_PI) + " deg";
            RCLCPP_ERROR(ctx_->get_logger(), "  ✗ %s", detail.c_str());
            return false;
        }
    }

    // ---- 5. Physical rod truth AFTER, reported separately from the scene.
    {
        const MujocoRodTruth after_phys = readMujocoRodTruth();
        if (before_phys.ok && after_phys.ok) {
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P3A-2B] 物理杆变化（MuJoCo /get_body_state，前→后）: %s",
                        rodDeltaText(before_phys.position, before_phys.orientation,
                                     after_phys.position, after_phys.orientation)
                            .c_str());
        } else {
            // The "before" reading is now mandatory, so only the "after" reading
            // can be missing here. Say so plainly instead of implying the rod was
            // undisturbed.
            RCLCPP_WARN(ctx_->get_logger(),
                        "  [P3A-2B] 下降后的物理杆测量**失败**（%s）；本轮**未测得**杆是否被扰动，"
                        "不对此作任何声称。下降前已测得 = %s",
                        after_phys.detail.c_str(), before_phys.ok ? "是" : "否");
        }
        a1v::Vec3 scene_after_pos;
        a1v::Quat scene_after_quat;
        if (scene_before_ok &&
            readSceneRodPose(a1_verify_.rod_id, scene_after_pos, scene_after_quat)) {
            RCLCPP_INFO(ctx_->get_logger(),
                        "  [P3A-2B] 规划场景对象变化（/get_planning_scene，前→后）: %s "
                        "—— 这是**场景对象**，不是物理测量",
                        rodDeltaText(scene_before_pos, scene_before_quat, scene_after_pos,
                                     scene_after_quat)
                            .c_str());
        }
    }

    detail = "GRASP_POSE_REACHED";
    return true;
}
