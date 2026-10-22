#include <multi_mode_controller/controllers/dual_cartesian_impedance_controller.h>

#include <multi_mode_controller/utils/controller_factory.h>

using namespace panda_controllers;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = DualCartesianImpedanceControllerPose;
using Parameters = DualCartesianImpedanceControllerParams;
using GoalMsg = multi_mode_control_msgs::msg::DualCartesianImpedanceGoal;
using ConfigRequest = multi_mode_control_msgs::srv::SetCartesianImpedance::Request;
using ConfigResponse = multi_mode_control_msgs::srv::SetCartesianImpedance::Response;
using Controller = DualCartesianImpedanceController;


static auto registration = ControllerFactory::registerClass<Controller>(
    "dual_cartesian_impedance_controller");


bool Controller::desiredPoseCallbackImpl(Pose& p_d, 
                                         const Pose& p,
                                         const GoalMsg& msg) {
  p_d.poses[0].position = Vector3d(msg.l_pose.position.x, msg.l_pose.position.y,
                                    msg.l_pose.position.z);
  p_d.poses[1].position = Vector3d(msg.r_pose.position.x, msg.r_pose.position.y,
                                    msg.r_pose.position.z);
  // p_d.poses[0].position = Vector3d(msg.l_pose.position.x, msg.l_pose.position.y,
  //                         msg.l_pose.position.z);
  // p_d.poses[1].position = Vector3d(msg.r_pose.position.x, msg.r_pose.position.y,
  //                         msg.r_pose.position.z);

  if ((p_d.poses[0].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[0].position-p.poses[0].position).norm() > 0.1 ||
      (p_d.poses[1].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[1].position-p.poses[1].position).norm() > 0.1) {
    auto& clk = *PandaControllerBase<Parameters, Pose>::node_->get_clock();
    const auto& logger = PandaControllerBase<Parameters, Pose>::node_->get_logger();
    RCLCPP_WARN_THROTTLE(logger, clk, 1000, "dual_cartesian_impedance_controller: Discarding "
        "target pose that is too far away from current pose (left: %f m, right: %f m, allowed "
        "maximum is 0.1 m).",
        (p_d.poses[0].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[0].position-p.poses[0].position).norm(),
        (p_d.poses[1].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[1].position-p.poses[1].position).norm());
    return false;
  }
  p_d.poses[0].orientation = Quaterniond(msg.l_pose.orientation.w, msg.l_pose.orientation.x,
                                          msg.l_pose.orientation.y, msg.l_pose.orientation.z);
  p_d.poses[1].orientation = Quaterniond(msg.r_pose.orientation.w, msg.r_pose.orientation.x,
                                           msg.r_pose.orientation.y, msg.r_pose.orientation.z);
  // p_d.poses[0].orientation = Quaterniond(msg.l_pose.orientation.w, msg.l_pose.orientation.x,
  //                               msg.l_pose.orientation.y, msg.l_pose.orientation.z);
  // p_d.poses[1].orientation = Quaterniond(msg.r_pose.orientation.w, msg.r_pose.orientation.x,
                                // msg.r_pose.orientation.y, msg.r_pose.orientation.z);
                                
  if (p.poses[0].orientation.angularDistance(p_d.poses[0].orientation) > 0.15 ||
      p.poses[1].orientation.angularDistance(p_d.poses[1].orientation) > 0.15) {
    auto& clk = *PandaControllerBase<Parameters, Pose>::node_->get_clock();
    const auto& logger = PandaControllerBase<Parameters, Pose>::node_->get_logger();
    RCLCPP_WARN_THROTTLE(logger, clk, 1000, "dual_cartesian_impedance_controller: Discarding "
        "target pose that rotates too far away from current pose (left: %f rad, right: %f rad, "
        "allowed maximum is 0.15 rad).",
        p.poses[0].orientation.angularDistance(p_d.poses[0].orientation),
        p.poses[1].orientation.angularDistance(p_d.poses[1].orientation));
    return false;
  }
  p_d.poses[0].q_n = Eigen::Map<const Vector7d>(msg.l_q_n.data());
  p_d.poses[1].q_n = Eigen::Map<const Vector7d>(msg.r_q_n.data());
  p_d.poses[0].twist << msg.l_twist.linear.x, msg.l_twist.linear.y, msg.l_twist.linear.z,
                        msg.l_twist.angular.x, msg.l_twist.angular.y, msg.l_twist.angular.z;
  p_d.poses[1].twist << msg.r_twist.linear.x, msg.r_twist.linear.y, msg.r_twist.linear.z,
                        msg.r_twist.angular.x, msg.r_twist.angular.y, msg.r_twist.angular.z;
  // Explicit payload feedforward, already expressed in each arm's link0
  // (zero-Jacobian) frame. Zero is the disabled/default branch.
  p_d.poses[0].ff_force = Vector3d(msg.l_ff_force.x, msg.l_ff_force.y, msg.l_ff_force.z);
  p_d.poses[1].ff_force = Vector3d(msg.r_ff_force.x, msg.r_ff_force.y, msg.r_ff_force.z);
  p_d.poses[0].use_cooperative_wrench = msg.use_cooperative_wrench;
  p_d.poses[1].use_cooperative_wrench = msg.use_cooperative_wrench;
  p_d.poses[0].cooperative_wrench <<
      msg.l_cooperative_wrench.force.x, msg.l_cooperative_wrench.force.y,
      msg.l_cooperative_wrench.force.z, msg.l_cooperative_wrench.torque.x,
      msg.l_cooperative_wrench.torque.y, msg.l_cooperative_wrench.torque.z;
  p_d.poses[1].cooperative_wrench <<
      msg.r_cooperative_wrench.force.x, msg.r_cooperative_wrench.force.y,
      msg.r_cooperative_wrench.force.z, msg.r_cooperative_wrench.torque.x,
      msg.r_cooperative_wrench.torque.y, msg.r_cooperative_wrench.torque.z;
  if (!p_d.poses[0].twist.allFinite() || !p_d.poses[1].twist.allFinite() ||
      !p_d.poses[0].ff_force.allFinite() || !p_d.poses[1].ff_force.allFinite() ||
      !p_d.poses[0].cooperative_wrench.allFinite() || !p_d.poses[1].cooperative_wrench.allFinite()) {
    auto& clk = *PandaControllerBase<Parameters, Pose>::node_->get_clock();
    const auto& logger = PandaControllerBase<Parameters, Pose>::node_->get_logger();
    RCLCPP_WARN_THROTTLE(logger, clk, 1000,
        "dual_cartesian_impedance_controller: Discarding goal with a non-finite twist or wrench.");
    return false;
  }
  return true;
}

bool Controller::setParametersCallbackImpl(Parameters& p_d, const Parameters& p,
            const std::shared_ptr<ConfigRequest>& request, const std::shared_ptr<ConfigResponse>& response) {
  (void)p;
  (void)response;
  for(int i=0; i<2; i++){
    p_d.params[i].stiffness = Matrix6d(request->stiffness.data());
    p_d.params[i].damping_ratio = Vector6d(request->damping_ratio.data());
    p_d.params[i].nullspace_stiffness = request->nullspace_stiffness;
  }
  return true;
}
