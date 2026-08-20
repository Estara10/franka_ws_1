#include "dual_arm_control/dual_arm_gripper_controller.hpp"

DualArmGripperController::DualArmGripperController(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node)
{
    // Ensure we are using simulation time if parameter is set
    // (Managed by Node passed in)

    // Initialize MoveGroups for grippers
    // Group names must match those in dual_panda.srdf
    // We use a separate thread or async spinner usually, but here we assume the main node handles spinning
    // or we use standard blocking MoveGroup calls.
    
    try {
        hand_1_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand_1");
        hand_2_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand_2");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize MoveGroupInterface for grippers: %s", e.what());
    }
}

bool DualArmGripperController::open_grippers()
{
    RCLCPP_INFO(node_->get_logger(), "Opening grippers...");
    
    if (!hand_1_group_ || !hand_2_group_) {
         RCLCPP_ERROR(node_->get_logger(), "Gripper groups not initialized");
         return false;
    }

    bool success_1 = hand_1_group_->setNamedTarget("open");
    bool success_2 = hand_2_group_->setNamedTarget("open");
    
    if (success_1 && success_2) {
        // Move synchronously for simplicity
        // Note: For true implementation, one might want to check if move() succeeds
        auto res1 = hand_1_group_->move();
        auto res2 = hand_2_group_->move();
        return (res1 == moveit::core::MoveItErrorCode::SUCCESS && res2 == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set open target for hands");
        return false;
    }
}

bool DualArmGripperController::close_grippers()
{
    RCLCPP_INFO(node_->get_logger(), "Closing grippers...");
    
    if (!hand_1_group_ || !hand_2_group_) return false;

    hand_1_group_->setNamedTarget("close");
    hand_2_group_->setNamedTarget("close");
    
    auto res1 = hand_1_group_->move();
    auto res2 = hand_2_group_->move();
    
    return (res1 == moveit::core::MoveItErrorCode::SUCCESS && res2 == moveit::core::MoveItErrorCode::SUCCESS);
}

void DualArmGripperController::add_collision_object()
{
    RCLCPP_INFO(node_->get_logger(), "Adding collision object: %s", OBJECT_ID.c_str());
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world"; 
    collision_object.id = OBJECT_ID;
    
    // Define position (matches gazebo spawn)
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.2;
    pose.orientation.w = 1.0;
    
    // Define shape (matches URDF box)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;
    primitive.dimensions[1] = 0.04;
    primitive.dimensions[2] = 0.04;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;
    
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    
    // Apply
    planning_scene_interface_.applyCollisionObjects(collision_objects);
}

void DualArmGripperController::attach_object_to_hands()
{
    RCLCPP_INFO(node_->get_logger(), "Attaching object to panda_1_link8 (and allowing collision with both hands)...");
    
    // 方法1：使用 MoveGroupInterface::attachObject（更可靠）
    // 这个方法会自动处理从 world 到 attached 的转换，保持物体当前位置
    
    // 定义 touch_links - 允许这些 link 与物体接触而不报碰撞
    std::vector<std::string> touch_links = {
        "panda_1_hand", "panda_1_leftfinger", "panda_1_rightfinger", "panda_1_link8",
        "panda_2_hand", "panda_2_leftfinger", "panda_2_rightfinger", "panda_2_link8"
    };
    
    // 使用 hand_1_group_ 来附着物体（它可以访问 panda_1_link8）
    if (hand_1_group_) {
        bool success = hand_1_group_->attachObject(OBJECT_ID, "panda_1_link8", touch_links);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Successfully attached %s to panda_1_link8", OBJECT_ID.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to attach %s", OBJECT_ID.c_str());
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "hand_1_group_ not initialized, cannot attach object");
    }
    
    // 等待规划场景更新
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void DualArmGripperController::detach_object()
{
    RCLCPP_INFO(node_->get_logger(), "Detaching object...");
    
    // 使用 MoveGroupInterface::detachObject（与 attachObject 配套）
    if (hand_1_group_) {
        bool success = hand_1_group_->detachObject(OBJECT_ID);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Successfully detached %s", OBJECT_ID.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to detach %s (may already be detached)", OBJECT_ID.c_str());
        }
    }
    
    // 等待规划场景更新
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
