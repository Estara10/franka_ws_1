#pragma once

#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <multi_mode_controller/base/switch_engine_interface.h>
#include <multi_mode_controller/utils/get_robot_state_msg.h>
#include <franka/robot_state.h>
#include <chrono>
#include <sstream>
// franka_msgs::FrankaState getRobotStateMsg(const franka::RobotState& rs);


namespace switch_engine {

  using multi_mode_control_msgs::srv::GetRobotStates;
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

class SwitchEngine : public SwitchEngineInterface {
 private:
  
//   ros::ServiceServer get_robot_states_;
  virtual void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      panda_controllers::PandaControllerInterface* control) override {
    return control->computeTau(tau);
  }
  // Service not working FIX:
  // if arg is shared pointers, they have to be both const
  // or you get an operator = error 
  // and making them just references don't work either.
  bool getRobotStatesCallback(
        const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const GetRobotStates::Request::SharedPtr& /*req*/,
        const GetRobotStates::Response::SharedPtr& res) {
    const auto wall_now = std::chrono::system_clock::now().time_since_epoch();
    const auto stamp_sec = std::chrono::duration_cast<std::chrono::seconds>(wall_now);
    const auto stamp_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_now - stamp_sec);
    for (std::size_t index = 0; index < robot_data_.size(); ++index) {
      auto state = getRobotStateMsg(robot_data_[index]->state());
      state.header.frame_id = index < arm_ids_.size() ? arm_ids_[index] + "_link0" : "unknown_link0";
      state.header.stamp.sec = static_cast<std::int32_t>(stamp_sec.count());
      state.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_nsec.count());
      res->states.push_back(std::move(state));
    }
    return true;
  }
  virtual bool initImpl(
      const std::vector<panda_controllers::RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string /*name*/, std::string resource) override {
    robot_data_ = robot_data;
    std::string topic_prefix = resource;
    std::size_t pos = topic_prefix.find("&");
    while (pos != std::string::npos) {
      topic_prefix.replace(pos, 1, "_and_");
      pos = topic_prefix.find("&");
    }
    std::stringstream resource_stream(resource);
    std::string arm_id;
    arm_ids_.clear();
    while (std::getline(resource_stream, arm_id, '&')) {
      if (!arm_id.empty()) arm_ids_.push_back(arm_id);
    }
    if (arm_ids_.size() != robot_data_.size()) return false;
    
    get_robot_states_server_ = node->create_service<GetRobotStates>(
        topic_prefix + "/get_robot_states",
        std::bind(&SwitchEngine::getRobotStatesCallback, this, _1, _2, _3));

    return true;
  }
  virtual void startImpl(
      panda_controllers::PandaControllerInterface* control) override {
    return control->start();
  }
  virtual void stopImpl(
      panda_controllers::PandaControllerInterface* control) override {
    return control->stop();
  }

  rclcpp::Service<GetRobotStates>::SharedPtr get_robot_states_server_;
  std::vector<panda_controllers::RobotData*> robot_data_;
  std::vector<std::string> arm_ids_;
};

}
