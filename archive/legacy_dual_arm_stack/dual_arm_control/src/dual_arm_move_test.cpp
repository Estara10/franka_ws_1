#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>
#include <sensor_msgs/msg/joint_state.hpp>

// 辅助类：用于检测 joint_states 是否活跃
class JointStateListener : public rclcpp::Node
{
public:
  JointStateListener() : Node("joint_state_check_node")
  {
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!received_) {
           RCLCPP_INFO(this->get_logger(), "📥 成功联通物理引擎！监测到 %zu 个关节", msg->name.size());
        }
        received_ = true;
      });
  }
  bool hasReceived() const { return received_; }
private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  bool received_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // 1. 强制使用仿真时间
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides({{"use_sim_time", true}});

  auto node = std::make_shared<rclcpp::Node>("dual_arm_dance_test", node_options);
  
  // 后台处理回调
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  RCLCPP_INFO(node->get_logger(), "🚀 启动双臂大幅度运动测试...");

  // 2. 握手检查
  auto check_node = std::make_shared<JointStateListener>();
  rclcpp::executors::SingleThreadedExecutor check_executor;
  check_executor.add_node(check_node);
  
  int wait_sec = 0;
  while (rclcpp::ok() && !check_node->hasReceived()) {
    check_executor.spin_some();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(node->get_logger(), "⏳ 等待机械臂数据... (%ds)", ++wait_sec);
    if (wait_sec > 30) return 1;
  }

  // 3. 设置 MoveIt
  static const std::string PLANNING_GROUP = "dual_arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // 重要：提高速度，让动作肉眼可见
  move_group.setMaxVelocityScalingFactor(0.5); // 50% 速度
  move_group.setMaxAccelerationScalingFactor(0.5);

  // 4. 获取当前位姿
  std::vector<double> current_joints = move_group.getCurrentJointValues();
  std::vector<double> target_joints = current_joints;

  // 5. 设置大幅度目标 (单位：弧度)
  // Panda 有 7 个自由度。dual_arm 组通常前7个是 Panda_1, 后7个是 Panda_2
  if (target_joints.size() >= 14) {
      // 左臂 (向左转 90度，抬起)
      target_joints[0] += 1.57; // Joint 1: Pan
      target_joints[1] = 0.5;   // Joint 2: Lift

      // 右臂 (向右转 90度，抬起)
      target_joints[7] -= 1.57; // Joint 1: Pan
      target_joints[8] = 0.5;   // Joint 2: Lift
      
      RCLCPP_INFO(node->get_logger(), "🎯 目标设定：双臂展开 (Base旋转 +/- 90度)");
  }

  move_group.setJointValueTarget(target_joints);
  move_group.setPlanningTime(10.0);

  // 6. 规划与执行
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "✅ 规划成功！正在向物理引擎发送指令...");
    // execute 是阻塞函数，直到动作完成才会返回
    move_group.execute(my_plan); 
    RCLCPP_INFO(node->get_logger(), "🏁 动作执行完毕。请检查 Gazebo 画面。");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "❌ 路径规划失败");
  }

  rclcpp::shutdown();
  return 0;
}