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
        
        // 创建MuJoCo weld约束控制发布器
        weld_pub_ = this->create_publisher<std_msgs::msg::Bool>("/grasp_weld_active", 10);
        
        current_state_ = TaskState::INIT;
        publish_stage(task_stage_pub_, current_state_);
        grippers_ = std::make_shared<GripperController>(this);
        arms_ = std::make_shared<DualArmController>(this);
        
        RCLCPP_INFO(this->get_logger(), "\n配置参数:");
        RCLCPP_INFO(this->get_logger(), "  左臂组: %s", left_arm_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "  右臂组: %s", right_arm_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "  双臂组: %s", dual_arm_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "  控制模式: %s", control_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "  旋转阶段: %s", enable_rotate_ ? "启用" : "禁用（直接放置）");
    }
    


void DualArmCarryTask::executeTask()
    {
        RCLCPP_INFO(get_logger(), "\n========================================");
        RCLCPP_INFO(get_logger(), "    开始执行双臂协作搬运任务");
        RCLCPP_INFO(get_logger(), "========================================\n");
        
        TaskState last_published_state = current_state_;
        publish_stage(task_stage_pub_, last_published_state);

        while (rclcpp::ok() && current_state_ != TaskState::DONE && current_state_ != TaskState::ERROR) {
            if (current_state_ != last_published_state) {
                publish_stage(task_stage_pub_, current_state_);
                last_published_state = current_state_;
            }

            switch (current_state_) {
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
        
        if (current_state_ == TaskState::DONE) {
            publish_stage(task_stage_pub_, TaskState::DONE);
            RCLCPP_INFO(get_logger(), "\n========================================");
            RCLCPP_INFO(get_logger(), "    ✓ 任务完成！");
            std_msgs::msg::String msg;
            msg.data = "DONE";
            task_status_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "已发布任务完成状态(DONE)。");

            RCLCPP_INFO(get_logger(), "========================================\n");
        } else if (current_state_ == TaskState::ERROR) {
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
