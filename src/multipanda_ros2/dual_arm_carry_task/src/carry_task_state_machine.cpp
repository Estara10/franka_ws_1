#include "dual_arm_carry_task/carry_task_context.hpp"
#include "dual_arm_carry_task/gripper_controller.hpp"
#include "dual_arm_carry_task/dual_arm_controller.hpp"

namespace {

const char* to_stage_name(TaskState state)
{
    switch (state) {
        case TaskState::INIT: return "INIT";
        case TaskState::APPROACH: return "APPROACH";
        case TaskState::GRASP: return "GRASP";
        case TaskState::WAITING_PHYSICS: return "WAITING_PHYSICS";
        case TaskState::LIFT: return "LIFT";
        case TaskState::TRANSPORT: return "TRANSPORT";
        case TaskState::ROTATE: return "ROTATE";
        case TaskState::DESCEND: return "DESCEND";
        case TaskState::PLACE: return "PLACE";
        case TaskState::RETREAT: return "RETREAT";
        case TaskState::PREPARED: return "PREPARED";
        case TaskState::DONE: return "DONE";
        case TaskState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void publish_stage(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub, TaskState state)
{
    if (!pub) {
        return;
    }
    std_msgs::msg::String stage_msg;
    stage_msg.data = to_stage_name(state);
    pub->publish(stage_msg);
}

}  // namespace

    DualArmCarryTask::DualArmCarryTask() : rclcpp::Node("dual_arm_carry_task")
    {
        RCLCPP_INFO(this->get_logger(), "=== 双臂协作搬运任务节点初始化 ===");
        
        // 初始化参数
        this->declare_parameter<std::string>("left_arm_group", "mj_left_arm");
        this->declare_parameter<std::string>("right_arm_group", "mj_right_arm");
        this->declare_parameter<std::string>("dual_arm_group", "dual_panda");
        this->declare_parameter<std::string>("control_mode", "compliant_chomp");
        this->declare_parameter<double>("approach_height", 0.28);
        this->declare_parameter<double>("grasp_height", 0.13);
        this->declare_parameter<double>("lift_height", 0.40);  // 增加到40cm，便于后续运动规划
        this->declare_parameter<bool>("enable_rotate", true);   // true=执行ROTATE，false=直接放置不旋转
        this->declare_parameter<double>("gripper_close_position", 0.010);  // 物体宽0.04m，每指目标0.01m vs 实际0.02m → PD误差0.01m产生强力夹持，且不会超过关节下限

        // Mode B: vision-driven dynamic grasping parameters
        this->declare_parameter<bool>("use_vision", false);
        this->declare_parameter<double>("rod_target_x", 0.40);
        this->declare_parameter<double>("rod_target_y", -0.20);
        this->declare_parameter<double>("rod_target_yaw_deg", 90.0);

        // A1 P1b: rod pose source selection
        this->declare_parameter<std::string>("rod_pose_source", kRodPoseSourceStatic);
        this->declare_parameter<std::string>("expected_run_id", "");
        this->declare_parameter<bool>("prepare_only", false);
        this->declare_parameter<std::string>("a1_truth_topic", "/a1_rod_pose_truth");
        this->declare_parameter<double>("a1_truth_timeout_sec", 20.0);
        this->declare_parameter<double>("rod_truth_size_min", 0.005);
        this->declare_parameter<double>("rod_truth_size_max", 1.0);

        // A1 P2a: static validity checks (geometry / dual-arm IK / collision).
        // Off by default so the P1b prepare behaviour is unchanged.
        this->declare_parameter<bool>("p2a_check", false);
        this->declare_parameter<double>("p2a_workspace_x_min", 0.30);
        this->declare_parameter<double>("p2a_workspace_x_max", 0.70);
        this->declare_parameter<double>("p2a_workspace_y_min", -0.30);
        this->declare_parameter<double>("p2a_workspace_y_max", 0.30);
        this->declare_parameter<std::string>(
            "p2a_workspace_source",
            "config/a1_scene_sampling.yaml 采样范围按 15° 最大 footprint 外扩后加 80 mm 余量；"
            "MuJoCo 支撑面为无限 plane，非物理桌面边缘");
        this->declare_parameter<double>("p2a_support_z", 0.0);
        this->declare_parameter<bool>("p2a_support_plane", true);
        this->declare_parameter<std::string>("p2a_support_object_id", "a1_support_plane");
        this->declare_parameter<bool>("p2a_ground_negative", true);
        this->declare_parameter<bool>("p2a_hand_aabb_substitution", true);
        this->declare_parameter<bool>("p2a_hand_negative", true);
        this->declare_parameter<double>("p2a_support_depth_m", 2.0);
        this->declare_parameter<bool>("p2b_check", false);
        this->declare_parameter<std::string>("p2b_planner_id", "PTP");
        this->declare_parameter<double>("p2b_planning_time_sec", 10.0);
        this->declare_parameter<int>("p2b_planning_attempts", 5);
        this->declare_parameter<double>("p2b_max_joint_step_rad", 0.05);
        this->declare_parameter<bool>("p2b_midpath_negative", true);
        this->declare_parameter<double>("p2a_resting_z_tol", 0.003);
        this->declare_parameter<double>("p2a_max_tilt_deg", 2.0);
        this->declare_parameter<double>("p2a_grasp_offset", 0.15);
        this->declare_parameter<double>("p2a_eef_base_yaw_deg", 45.0);
        this->declare_parameter<int>("p2a_ik_attempts", 5);
        this->declare_parameter<double>("p2a_ik_timeout_sec", 0.05);
        this->declare_parameter<double>("p2a_ik_pos_tol_m", 0.001);
        this->declare_parameter<double>("p2a_ik_angle_tol_deg", 0.5);
        this->declare_parameter<double>("p2a_max_joint_jump_rad", 2.0);

        left_arm_group_ = this->get_parameter("left_arm_group").as_string();
        right_arm_group_ = this->get_parameter("right_arm_group").as_string();
        control_mode_ = this->get_parameter("control_mode").as_string();
        task_stage_pub_ = this->create_publisher<std_msgs::msg::String>("/task_stage", 20);
        task_status_pub_ = this->create_publisher<std_msgs::msg::String>("/task_status", 10);
        dual_arm_group_ = this->get_parameter("dual_arm_group").as_string();
        approach_height_ = this->get_parameter("approach_height").as_double();
        grasp_height_ = this->get_parameter("grasp_height").as_double();
        lift_height_ = this->get_parameter("lift_height").as_double();
        enable_rotate_ = this->get_parameter("enable_rotate").as_bool();
        gripper_close_pos_ = this->get_parameter("gripper_close_position").as_double();

        use_vision_ = this->get_parameter("use_vision").as_bool();
        rod_target_x_ = this->get_parameter("rod_target_x").as_double();
        rod_target_y_ = this->get_parameter("rod_target_y").as_double();
        rod_target_yaw_rad_ = this->get_parameter("rod_target_yaw_deg").as_double() * M_PI / 180.0;

        rod_pose_source_ = this->get_parameter("rod_pose_source").as_string();
        expected_run_id_ = this->get_parameter("expected_run_id").as_string();
        prepare_only_ = this->get_parameter("prepare_only").as_bool();
        a1_truth_topic_ = this->get_parameter("a1_truth_topic").as_string();
        a1_truth_timeout_sec_ = this->get_parameter("a1_truth_timeout_sec").as_double();
        rod_truth_size_min_ = this->get_parameter("rod_truth_size_min").as_double();
        rod_truth_size_max_ = this->get_parameter("rod_truth_size_max").as_double();

        p2a_check_ = this->get_parameter("p2a_check").as_bool();
        p2a_workspace_x_min_ = this->get_parameter("p2a_workspace_x_min").as_double();
        p2a_workspace_x_max_ = this->get_parameter("p2a_workspace_x_max").as_double();
        p2a_workspace_y_min_ = this->get_parameter("p2a_workspace_y_min").as_double();
        p2a_workspace_y_max_ = this->get_parameter("p2a_workspace_y_max").as_double();
        p2a_workspace_source_ = this->get_parameter("p2a_workspace_source").as_string();
        p2a_support_z_ = this->get_parameter("p2a_support_z").as_double();
        p2a_support_plane_ = this->get_parameter("p2a_support_plane").as_bool();
        p2a_support_object_id_ = this->get_parameter("p2a_support_object_id").as_string();
        p2a_ground_negative_ = this->get_parameter("p2a_ground_negative").as_bool();
        p2a_hand_aabb_substitution_ = this->get_parameter("p2a_hand_aabb_substitution").as_bool();
        p2a_hand_negative_ = this->get_parameter("p2a_hand_negative").as_bool();
        p2a_support_depth_m_ = this->get_parameter("p2a_support_depth_m").as_double();
        p2b_check_ = this->get_parameter("p2b_check").as_bool();
        p2b_planner_id_ = this->get_parameter("p2b_planner_id").as_string();
        p2b_planning_time_sec_ = this->get_parameter("p2b_planning_time_sec").as_double();
        p2b_planning_attempts_ = this->get_parameter("p2b_planning_attempts").as_int();
        p2b_max_joint_step_rad_ = this->get_parameter("p2b_max_joint_step_rad").as_double();
        p2b_midpath_negative_ = this->get_parameter("p2b_midpath_negative").as_bool();
        p2a_resting_z_tol_ = this->get_parameter("p2a_resting_z_tol").as_double();
        p2a_max_tilt_deg_ = this->get_parameter("p2a_max_tilt_deg").as_double();
        p2a_grasp_offset_m_ = this->get_parameter("p2a_grasp_offset").as_double();
        p2a_eef_base_yaw_deg_ = this->get_parameter("p2a_eef_base_yaw_deg").as_double();
        p2a_ik_attempts_ = this->get_parameter("p2a_ik_attempts").as_int();
        p2a_ik_timeout_sec_ = this->get_parameter("p2a_ik_timeout_sec").as_double();
        p2a_ik_pos_tol_m_ = this->get_parameter("p2a_ik_pos_tol_m").as_double();
        p2a_ik_angle_tol_deg_ = this->get_parameter("p2a_ik_angle_tol_deg").as_double();
        p2a_max_joint_jump_rad_ = this->get_parameter("p2a_max_joint_jump_rad").as_double();

        // Source resolution. The vision entry keeps working unchanged: leaving
        // rod_pose_source at its default while use_vision is true means vision.
        if (use_vision_ && rod_pose_source_ == kRodPoseSourceStatic) {
            rod_pose_source_ = kRodPoseSourceVision;
            RCLCPP_INFO(this->get_logger(),
                        "  rod_pose_source 未显式设置且 use_vision=true -> 按 vision 处理（兼容原视觉入口）");
        }
        if (use_vision_ && rod_pose_source_ == kRodPoseSourceA1Truth) {
            RCLCPP_ERROR(this->get_logger(),
                         "  ✗ 参数冲突: use_vision=true 与 rod_pose_source=a1_truth 不能同时使用");
            RCLCPP_ERROR(this->get_logger(),
                         "    真值输入与视觉输入必须分开，不会静默切换来源");
            setCurrentState(TaskState::ERROR);
            publish_stage(task_stage_pub_, getCurrentState());
            return;
        }
        if (rod_pose_source_ != kRodPoseSourceStatic && rod_pose_source_ != kRodPoseSourceVision &&
            rod_pose_source_ != kRodPoseSourceA1Truth) {
            RCLCPP_ERROR(this->get_logger(), "  ✗ 未知 rod_pose_source: %s",
                         rod_pose_source_.c_str());
            setCurrentState(TaskState::ERROR);
            publish_stage(task_stage_pub_, getCurrentState());
            return;
        }

        // 创建MuJoCo weld约束控制发布器
        weld_pub_ = this->create_publisher<std_msgs::msg::Bool>("/grasp_weld_active", 10);

        // Mode B: subscribe to detected rod pose from vision node
        if (rod_pose_source_ == kRodPoseSourceVision) {
            rod_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/detected_rod_pose", 10,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    const bool first_detection = !hasDetectedRodPose();
                    setDetectedRodPose(msg->pose);
                    if (first_detection) {
                        RCLCPP_INFO(this->get_logger(),
                            "Vision: rod detected at (%.3f, %.3f, %.3f)",
                            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                    }
                });
            RCLCPP_INFO(this->get_logger(), "Vision mode enabled: subscribing to /detected_rod_pose");
        }

        // A1 P1b: subscribe to the independent truth channel. transient_local so
        // a latched pose published before this node started is still delivered;
        // the callback only offers the payload to the latch, which freezes on
        // the first valid message and never updates afterwards.
        if (rod_pose_source_ == kRodPoseSourceA1Truth) {
            rclcpp::QoS truth_qos(1);
            truth_qos.transient_local().reliable();
            a1_truth_sub_ = this->create_subscription<dual_arm_a1_msgs::msg::RodPoseTruth>(
                a1_truth_topic_, truth_qos,
                [this](const dual_arm_a1_msgs::msg::RodPoseTruth::SharedPtr msg) {
                    a1t::TruthPayload payload;
                    payload.run_id = msg->run_id;
                    payload.frame_id = msg->header.frame_id;
                    payload.position = {msg->pose.position.x, msg->pose.position.y,
                                        msg->pose.position.z};
                    payload.orientation = {msg->pose.orientation.x, msg->pose.orientation.y,
                                           msg->pose.orientation.z, msg->pose.orientation.w};
                    payload.size = {msg->rod_size.x, msg->rod_size.y, msg->rod_size.z};
                    payload.mass_kg = msg->mass_kg;
                    payload.has_header_stamp = true;
                    payload.header_stamp_is_zero =
                        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);

                    const a1t::Verdict v = a1_truth_latch_.offer(payload, expected_run_id_,
                                                                 rod_truth_size_min_,
                                                                 rod_truth_size_max_);
                    if (v.ok) {
                        RCLCPP_INFO(this->get_logger(),
                                    "  A1 真值已冻结: run_id=%s frame=%s pos=(%.6f, %.6f, %.6f)",
                                    payload.run_id.c_str(), payload.frame_id.c_str(),
                                    payload.position.x, payload.position.y, payload.position.z);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "  A1 真值被拒: %s (%s)",
                                    v.reason_code.c_str(), v.detail.c_str());
                    }
                });
            RCLCPP_INFO(this->get_logger(),
                        "A1 truth mode enabled: subscribing to %s (expected_run_id=%s)",
                        a1_truth_topic_.c_str(),
                        expected_run_id_.empty() ? "<UNSET>" : expected_run_id_.c_str());
        }

        setCurrentState(TaskState::INIT);
        publish_stage(task_stage_pub_, getCurrentState());
        grippers_ = std::make_shared<GripperController>(this);
        arms_ = std::make_shared<DualArmController>(this);

        RCLCPP_INFO(this->get_logger(), "\n配置参数:");
        RCLCPP_INFO(this->get_logger(), "  左臂组: %s", left_arm_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "  右臂组: %s", right_arm_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "  双臂组: %s", dual_arm_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "  控制模式: %s", control_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "  旋转阶段: %s", enable_rotate_ ? "启用" : "禁用（直接放置）");
        RCLCPP_INFO(this->get_logger(), "  杆位姿来源: %s", rod_pose_source_.c_str());
        if (rod_pose_source_ == kRodPoseSourceA1Truth) {
            RCLCPP_INFO(this->get_logger(), "  准备模式(prepare_only): %s",
                        prepare_only_ ? "启用（本轮 A1 不运动）" : "禁用");
            RCLCPP_INFO(this->get_logger(), "  P2a 静态检查(p2a_check): %s",
                        p2a_check_ ? "启用（几何/双臂 IK/组合碰撞，仍不运动）" : "禁用");
        }
        if (rod_pose_source_ == kRodPoseSourceVision) {
            RCLCPP_INFO(this->get_logger(), "  视觉模式: 启用 (目标: %.3f, %.3f, yaw=%.1f°)",
                        rod_target_x_, rod_target_y_, rod_target_yaw_rad_ * 180.0 / M_PI);
        }
    }
    


void DualArmCarryTask::executeTask()
    {
        RCLCPP_INFO(get_logger(), "\n========================================");
        RCLCPP_INFO(get_logger(), "    开始执行双臂协作搬运任务");
        RCLCPP_INFO(get_logger(), "========================================\n");
        
        TaskState last_published_state = getCurrentState();
        publish_stage(task_stage_pub_, last_published_state);

        while (rclcpp::ok()) {
            const TaskState state = getCurrentState();
            if (state == TaskState::DONE || state == TaskState::ERROR ||
                state == TaskState::PREPARED) {
                break;
            }

            if (state != last_published_state) {
                publish_stage(task_stage_pub_, state);
                last_published_state = state;
            }

            switch (state) {
                case TaskState::INIT:
                    arms_->executeInit();
                    break;
                case TaskState::APPROACH:
                    arms_->executeApproach();
                    break;
                case TaskState::GRASP:
                    arms_->executeGrasp();
                    break;
                case TaskState::WAITING_PHYSICS:
                    // 异步等待物理引擎稳定，由 Timer 触发状态流转
                    break;
                case TaskState::LIFT:
                    arms_->executeLift();
                    break;
                case TaskState::TRANSPORT:
                    arms_->executeTransport();
                    break;
                case TaskState::ROTATE:
                    arms_->executeRotate();
                    break;
                case TaskState::DESCEND:
                    arms_->executeDescend();
                    break;
                case TaskState::PLACE:
                    arms_->executePlace();
                    break;
                case TaskState::RETREAT:
                    arms_->executeRetreat();
                    break;
                default:
                    break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        const TaskState final_state = getCurrentState();
        if (final_state == TaskState::PREPARED) {
            // A1 P1b: preparation finished. This is deliberately NOT reported as
            // DONE -- nothing has been carried. It only means the truth input was
            // frozen and the planning scene was synchronised and verified.
            publish_stage(task_stage_pub_, TaskState::PREPARED);
            RCLCPP_INFO(get_logger(), "\n========================================");
            RCLCPP_INFO(get_logger(), "    ✓ 准备完成（PREPARED）—— 未执行任何搬运运动");
            std_msgs::msg::String msg;
            msg.data = "PREPARED";
            task_status_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "已发布准备完成状态(PREPARED)，不是任务 DONE。");
            RCLCPP_INFO(get_logger(), "========================================\n");
        } else if (final_state == TaskState::DONE) {
            publish_stage(task_stage_pub_, TaskState::DONE);
            RCLCPP_INFO(get_logger(), "\n========================================");
            RCLCPP_INFO(get_logger(), "    ✓ 任务完成！");
            std_msgs::msg::String msg;
            msg.data = "DONE";
            task_status_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "已发布任务完成状态(DONE)。");

            RCLCPP_INFO(get_logger(), "========================================\n");
        } else if (final_state == TaskState::ERROR) {
            publish_stage(task_stage_pub_, TaskState::ERROR);
            RCLCPP_ERROR(get_logger(), "\n========================================");
            RCLCPP_ERROR(get_logger(), "    ✗ 任务失败！");
            std_msgs::msg::String msg;
            msg.data = "ERROR";
            task_status_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "已发布任务失败状态(ERROR)。");
            RCLCPP_ERROR(get_logger(), "========================================\n");
        }
    }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DualArmCarryTask>();

    // A1 P1b 局部修复：来源冲突或未知来源时构造函数已置 ERROR 并提前返回，
    // 此时 arms_/grippers_ 尚未创建。用简单状态检查明确非零退出，不初始化
    // MoveIt、不访问未创建的控制器，也不创建 executor 线程。
    if (node->getCurrentState() == TaskState::ERROR) {
        RCLCPP_ERROR(node->get_logger(),
                     "  ✗ 参数无效（rod_pose_source 冲突或未知），未初始化控制器，非零退出");
        node.reset();
        rclcpp::shutdown();
        return 1;
    }

    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // 在单独线程中运行执行器
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    
    // 等待节点初始化
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 初始化 MoveIt
    node->arms_->initialize();
    
    // 执行任务
    node->executeTask();
    
    // 任务完成后继续spin等待，不退出
    RCLCPP_INFO(node->get_logger(), "任务执行完成，节点继续运行...");
    RCLCPP_INFO(node->get_logger(), "按 Ctrl+C 停止节点");
    
    // 阻塞主线程，让executor线程继续运行
    executor_thread.join();
    
    return 0;
}
