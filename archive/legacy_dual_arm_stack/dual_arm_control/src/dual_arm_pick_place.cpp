#include <rclcpp/rclcpp.hpp>
#include "dual_arm_control/dual_arm_gripper_controller.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 辅助函数：欧拉角转四元数
geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

// [修正] 不再继承 rclcpp::Node，而是持有一个 Node 指针
class DualArmPickPlace
{
public:
    DualArmPickPlace(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        // 1. 初始化 MoveGroup (使用传入的 safety node pointer)
        // 注意：MoveGroupInterface 接受 const std::shared_ptr<rclcpp::Node>&
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "dual_arm");
        
        // 2. 初始化辅助类
        gripper_controller_ = std::make_shared<DualArmGripperController>(node_);

        // 3. 设置 MoveIt 参数
        move_group_->setMaxVelocityScalingFactor(0.3); 
        move_group_->setMaxAccelerationScalingFactor(0.3);
        move_group_->setPlanningTime(10.0);

        RCLCPP_INFO(node_->get_logger(), "Initialization complete.");
    }

    void run()
    {
        // ==========================================
        // Step 1: 准备工作 (Reset)
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 1: Moving to HOME pose...");
        move_group_->setNamedTarget("home");
        
        // [Fix] 增加由于仿真误差导致的容差允许范围
        move_group_->setGoalTolerance(0.05);
        move_group_->setPlanningTime(10.0);
        
        if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to reach Home. Is the controller running?");
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Step 1: Opening grippers...");
        gripper_controller_->open_grippers();
        gripper_controller_->detach_object(); 

        // ==========================================
        // Step 2: 生成场景物体
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 2: Adding collision object to scene...");
        gripper_controller_->add_collision_object();
        std::this_thread::sleep_for(std::chrono::seconds(2)); 

        // ==========================================
        // Step 3: 预接近 (Pre-Approach / Observation Pose)
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 3: Planning to Pre-grasp position...");
        
        // [关键修正] 使用关节空间规划，而不是笛卡尔空间规划
        // 对于 dual_arm 组，直接设置双末端位姿容易导致 IK 失败
        // 改用 setJointValueTarget 更可靠
        
        // 方案：使用关节角度直接控制，避免复杂的双臂 IK
        std::map<std::string, double> target_joints;
        
        // Panda 1 (右臂) - 向右侧展开，末端朝下
        target_joints["panda_1_joint1"] = 0.5;      // 底座稍微向外转
        target_joints["panda_1_joint2"] = -0.3;     // 肩膀抬高
        target_joints["panda_1_joint3"] = 0.0;
        target_joints["panda_1_joint4"] = -2.0;     // 肘部弯曲
        target_joints["panda_1_joint5"] = 0.0;
        target_joints["panda_1_joint6"] = 1.8;      // 腕部调整
        target_joints["panda_1_joint7"] = 0.785;
        
        // Panda 2 (左臂) - 向左侧展开，对称
        target_joints["panda_2_joint1"] = -0.5;     // 底座向外转（负方向）
        target_joints["panda_2_joint2"] = -0.3;
        target_joints["panda_2_joint3"] = 0.0;
        target_joints["panda_2_joint4"] = -2.0;
        target_joints["panda_2_joint5"] = 0.0;
        target_joints["panda_2_joint6"] = 1.8;
        target_joints["panda_2_joint7"] = 0.785;

        move_group_->setJointValueTarget(target_joints);
        move_group_->setPlanningTime(10.0);
        move_group_->setNumPlanningAttempts(10);

        if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(), "Step 3: Joint-space plan failed, trying with relaxed constraints...");
            
            // 放宽约束再试
            move_group_->setGoalTolerance(0.1);
            move_group_->setPlanningTime(20.0);
            
            if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(node_->get_logger(), "Step 3: Failed to reach Pre-Approach pose!");
                return;
            }
        }
        
        RCLCPP_INFO(node_->get_logger(), "Step 3: Pre-Approach position reached!");

        // ==========================================
        // Step 4: 垂直抓取 (Vertical Grasp) - 使用关节空间规划
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 4: Moving to grasp position...");
        
        // 物体位置: x=0.5, y=0, z=0.2
        // 物体尺寸: 0.4(X方向长) x 0.04 x 0.04
        // 两个夹爪需要从 Y 方向的两侧夹住物体
        // panda_1 在 Y=-0.4, panda_2 在 Y=+0.4
        // 物体两端在 Y=±0.2，所以夹爪需要到达 Y≈±0.2 的位置
        
        std::map<std::string, double> grasp_joints;
        
        // Panda 1 (Y负方向的臂，在 Y=-0.4) - 需要向 Y=0 方向伸展
        // 增大 joint1 使末端向中间移动，调整其他关节使末端朝向物体
        grasp_joints["panda_1_joint1"] = 1.2;       // 大幅向中间转
        grasp_joints["panda_1_joint2"] = 0.4;       // 肩膀前倾
        grasp_joints["panda_1_joint3"] = 0.0;
        grasp_joints["panda_1_joint4"] = -1.5;      // 肘部弯曲
        grasp_joints["panda_1_joint5"] = 0.0;
        grasp_joints["panda_1_joint6"] = 1.9;       // 腕部
        grasp_joints["panda_1_joint7"] = 0.785;     // 末端旋转
        
        // Panda 2 (Y正方向的臂，在 Y=+0.4) - 对称姿态
        grasp_joints["panda_2_joint1"] = -1.2;      // 大幅向中间转（负方向）
        grasp_joints["panda_2_joint2"] = 0.4;
        grasp_joints["panda_2_joint3"] = 0.0;
        grasp_joints["panda_2_joint4"] = -1.5;
        grasp_joints["panda_2_joint5"] = 0.0;
        grasp_joints["panda_2_joint6"] = 1.9;
        grasp_joints["panda_2_joint7"] = 0.785;

        move_group_->setJointValueTarget(grasp_joints);
        move_group_->setPlanningTime(15.0);
        move_group_->setNumPlanningAttempts(10);
        
        if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
           RCLCPP_ERROR(node_->get_logger(), "Step 4: Grasp position failed!"); 
        } else {
           RCLCPP_INFO(node_->get_logger(), "Step 4: Grasp position reached!");
        }

        // 闭合夹爪
        gripper_controller_->close_grippers();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 附着物体（在夹爪闭合后）
        gripper_controller_->attach_object_to_hands();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        // ==========================================
        // Step 5: 抬升 (Lift) - 使用关节空间规划
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 5: Lifting...");
        
        std::map<std::string, double> lift_joints;
        
        // 抬升：减小 joint2 使手臂抬高，保持双臂夹持姿态
        lift_joints["panda_1_joint1"] = 1.2;
        lift_joints["panda_1_joint2"] = -0.3;       // 肩膀抬高（负值）
        lift_joints["panda_1_joint3"] = 0.0;
        lift_joints["panda_1_joint4"] = -2.0;       // 肘部调整
        lift_joints["panda_1_joint5"] = 0.0;
        lift_joints["panda_1_joint6"] = 1.7;
        lift_joints["panda_1_joint7"] = 0.785;
        
        lift_joints["panda_2_joint1"] = -1.2;
        lift_joints["panda_2_joint2"] = -0.3;
        lift_joints["panda_2_joint3"] = 0.0;
        lift_joints["panda_2_joint4"] = -2.0;
        lift_joints["panda_2_joint5"] = 0.0;
        lift_joints["panda_2_joint6"] = 1.7;
        lift_joints["panda_2_joint7"] = 0.785;

        move_group_->setJointValueTarget(lift_joints);
        
        if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(), "Step 5: Lift failed!");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Step 5: Lift complete!");
        }

        // ==========================================
        // Step 6: 平移搬运 (Transport) - 向后（-X方向）移动较大距离
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 6: Transporting to new location...");
        
        std::map<std::string, double> transport_joints;
        
        // 通过减小 joint1 使物体向后（-X方向）移动
        // 同时调整其他关节保持抓取姿态
        transport_joints["panda_1_joint1"] = 0.3;   // 向后转（减小角度）
        transport_joints["panda_1_joint2"] = -0.5;  // 保持抬高
        transport_joints["panda_1_joint3"] = 0.0;
        transport_joints["panda_1_joint4"] = -2.2;
        transport_joints["panda_1_joint5"] = 0.0;
        transport_joints["panda_1_joint6"] = 1.7;
        transport_joints["panda_1_joint7"] = 0.785;
        
        transport_joints["panda_2_joint1"] = -0.3;  // 对称
        transport_joints["panda_2_joint2"] = -0.5;
        transport_joints["panda_2_joint3"] = 0.0;
        transport_joints["panda_2_joint4"] = -2.2;
        transport_joints["panda_2_joint5"] = 0.0;
        transport_joints["panda_2_joint6"] = 1.7;
        transport_joints["panda_2_joint7"] = 0.785;

        move_group_->setJointValueTarget(transport_joints);
        
        if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(), "Step 6: Transport failed!");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Step 6: Transport complete!");
        }

        // ==========================================
        // Step 7: 放置 (Place) - 降低到放置位置
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 7: Lowering to place...");
        
        std::map<std::string, double> place_joints;
        
        // 降低：增大 joint2 使手臂下降
        place_joints["panda_1_joint1"] = 0.3;
        place_joints["panda_1_joint2"] = 0.2;       // 肩膀下压
        place_joints["panda_1_joint3"] = 0.0;
        place_joints["panda_1_joint4"] = -1.6;
        place_joints["panda_1_joint5"] = 0.0;
        place_joints["panda_1_joint6"] = 1.8;
        place_joints["panda_1_joint7"] = 0.785;
        
        place_joints["panda_2_joint1"] = -0.3;
        place_joints["panda_2_joint2"] = 0.2;
        place_joints["panda_2_joint3"] = 0.0;
        place_joints["panda_2_joint4"] = -1.6;
        place_joints["panda_2_joint5"] = 0.0;
        place_joints["panda_2_joint6"] = 1.8;
        place_joints["panda_2_joint7"] = 0.785;

        move_group_->setJointValueTarget(place_joints);
        
        if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(), "Step 7: Place failed!");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Step 7: Place complete!");
        }

        // 松开夹爪并分离物体
        gripper_controller_->detach_object();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        gripper_controller_->open_grippers();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // ==========================================
        // Step 8: 撤退 (Retreat) - 回到安全位置
        // ==========================================
        RCLCPP_INFO(node_->get_logger(), "Step 8: Retreating...");
        
        std::map<std::string, double> retreat_joints;
        
        // 撤退到接近home的安全位置
        retreat_joints["panda_1_joint1"] = 0.0;
        retreat_joints["panda_1_joint2"] = -0.785;
        retreat_joints["panda_1_joint3"] = 0.0;
        retreat_joints["panda_1_joint4"] = -2.356;
        retreat_joints["panda_1_joint5"] = 0.0;
        retreat_joints["panda_1_joint6"] = 1.571;
        retreat_joints["panda_1_joint7"] = 0.785;
        
        retreat_joints["panda_2_joint1"] = 0.0;
        retreat_joints["panda_2_joint2"] = -0.785;
        retreat_joints["panda_2_joint3"] = 0.0;
        retreat_joints["panda_2_joint4"] = -2.356;
        retreat_joints["panda_2_joint5"] = 0.0;
        retreat_joints["panda_2_joint6"] = 1.571;
        retreat_joints["panda_2_joint7"] = 0.785;

        move_group_->setJointValueTarget(retreat_joints);
        
        if(move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(), "Step 8: Retreat failed!");
        } else {
            RCLCPP_INFO(node_->get_logger(), "Step 8: Retreat complete!");
        }
        
        RCLCPP_INFO(node_->get_logger(), "Mission Complete!");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<DualArmGripperController> gripper_controller_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({{"use_sim_time", true}});

    // [核心] 单独创建 Node
    auto node = std::make_shared<rclcpp::Node>("dual_arm_pick_place", node_options);
    
    // [核心] 将 Node 传给业务类
    auto app = std::make_shared<DualArmPickPlace>(node);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    app->run();
    
    rclcpp::shutdown();
    return 0;
}
