#include <multi_mode_controller/controllers/comless_dual_cartesian_impedance_controller.h>
#include <multi_mode_controller/utils/controller_factory.h>
#include <multi_mode_controller/utils/damping_design.h>
#include <multi_mode_controller/utils/nullspace_projection.h>
#include <multi_mode_controller/utils/panda_limits.h>
// SEEMS LIKE A POINTER ISSUE.
// WHEN TWO CLASSES DERIVE FROM THE SAME PARENT, IT DOESN'T SEEM TO WORK
using namespace panda_controllers;
using Eigen::Vector3d;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Eigen::Quaterniond;
using Pose = DualCartesianImpedanceControllerPose;
using Params = DualCartesianImpedanceControllerParams;
using Controller = ComlessDualCartesianImpedanceController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_dual_cartesian_impedance_controller");

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired_poses, const Params& p) {
  const double now = get_wall_time();
  const double dt = have_previous_time_ ? (now - previous_time_) : 0.0;
  previous_time_ = now;
  have_previous_time_ = true;
  // measurement setup
  // double start_time = get_wall_time();
  // std::vector<std::array<double,7>> taus_(2);
  // measurement setup

  const std::array<RobotDataSnapshot, 2> samples{{
      robot_data_[0]->snapshot(), robot_data_[1]->snapshot()}};
  Pose current_poses;
  for (int i = 0; i < 2; ++i) {
    const auto& state = samples[i].state;
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    current_poses.poses[i].position = transform.translation();
    current_poses.poses[i].orientation = Eigen::Quaterniond(transform.linear());
    current_poses.poses[i].q_n = Eigen::Map<const Vector7d>(state.q.data());
  }
  Eigen::VectorXd right_left_qs(17);
  right_left_qs << 0, 0, 0, current_poses.poses[1].q_n, current_poses.poses[0].q_n;
  // Vector14d Q = ... 
  bool isRight = false;
  for(int i = 0; i < 2; i++){
    auto& desired = desired_poses.poses[i];
    auto& current = current_poses.poses[i];
    current.position -= getOffset().poses[i].position;
    const auto& sample = samples[i];
    const Matrix7d& inertia = sample.mass;
    const Vector7d& coriolis = sample.coriolis;
    const Eigen::Matrix<double, 6, 7>& jacobian = sample.ee_zero_jacobian;
    const Vector7d qD = Eigen::Map<const Vector7d>(sample.state.dq.data());
    Vector6d error;
    error.head(3) << current.position - desired.position;
    if (desired.orientation.coeffs().dot(current.orientation.coeffs()) < 0.0) {
      current.orientation.coeffs() << -current.orientation.coeffs();
    }
    Vector7d tau_collision(7);
    tau_collision.setZero();
    if(bCollisionAvoidance){
      tau_collision << -redundancy_resolution::CollisionAvoidanceGradient(right_left_qs, isRight);
      // if(isRight){
      //   RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "CA tau: " <<  tau_collision.transpose());
      // }
    }
    Vector7d tau_manipulability(7);
    tau_manipulability.setZero();
    if(bManipulability){
      tau_manipulability << -redundancy_resolution::ManipulabilityGradient(right_left_qs, isRight);
    }
    isRight = !isRight;
    Eigen::Quaterniond rot_error(
        current.orientation * desired.orientation.inverse());
    Eigen::AngleAxisd rot_error_aa(rot_error);
    error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
    Vector7d tau_task, tau_nullspace, tau_d;
    Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia),
        p.params[i].stiffness, p.params[i].damping_ratio);
    // Task torque: the damping term acts on the difference between the
    // reference twist and the twist the arm actually has, measured in the same
    // frame as the error and the zero Jacobian. A moving reference is not
    // treated as if it were stationary.
    const Vector6d reference_twist = desired.twist;
    const Vector6d current_twist = jacobian * qD;
    if (desired.use_cooperative_wrench) {
      // The outer task computes one object wrench plus a zero-resultant
      // internal relative wrench. Do not superimpose the independent arm
      // spring/damper: doing so would count a second task controller.
      tau_task << jacobian.transpose() * desired.cooperative_wrench;
    } else {
      const Vector6d spring_force = -p.params[i].stiffness * error;
      const Vector6d damping_force = D * (reference_twist - current_twist);
      tau_task << jacobian.transpose() * (spring_force + damping_force);
    }
    // Explicit payload feedforward. `desired.ff_force` is in the same base
    // (zero-Jacobian) frame as the pose error, so this is J_linear^T F. It is
    // added to the raw torque BEFORE the shared magnitude/rate limits below.
    const Vector7d tau_payload_ff = jacobian.topRows<3>().transpose() * desired.ff_force;
    tau_nullspace <<
        getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
        (p.params[i].nullspace_stiffness * (desired.q_n - current.q_n) -
        (2.0 * std::sqrt(p.params[i].nullspace_stiffness)) * qD);
    // coriolis is the only feed-forward term: MuJoCo runs this model with
    // body gravcomp=1, so gravity is already cancelled by the simulator and
    // adding a gravity term here would compensate it twice.
    tau_d << tau_task + tau_payload_ff + tau_nullspace + coriolis + tau_collision + tau_manipulability;

    const TorqueLimitResult limited = applyTorqueLimits(
        tau_d, previous_tau_[i], dt, panda_limits::tau_max, panda_limits::tauD_max);
    if (limited.nonfinite_input) ++nonfinite_cycles_;
    if (limited.saturated) ++saturated_cycles_;
    if (limited.rate_limited) ++rate_limited_cycles_;
    previous_tau_[i] = limited.tau;
    for (size_t j = 0; j < 7; ++j) {
      (*tau[i])[j] = limited.tau[j];
    }
    last_raw_tau_[i] = tau_d;
    last_applied_tau_[i] = limited.tau;
  }
  if (++limit_log_cycle_ % 500 == 0) {
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "impedance torque protection: nonfinite=" << nonfinite_cycles_
        << " magnitude_limited=" << saturated_cycles_
        << " rate_limited=" << rate_limited_cycles_
        << " last_raw=" << last_raw_tau_[0].transpose()
        << " last_applied=" << previous_tau_[0].transpose()
        << " last_raw_right=" << last_raw_tau_[1].transpose()
        << " last_applied_right=" << previous_tau_[1].transpose());
  }
  /*Uncomment for logging */
  // if(getCounter() < max_count_){
  //   double end_time = get_wall_time();
  //   double diff = end_time - start_time;
  //   measureTime(diff);
  //   measureTau(taus_, end_time);
  //   increaseCounter();
  // }
  // else{
  //   if(!logged_){
  //     RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Max count reached: " << getCounter() << "/" << max_count_);
  //     std::string file_name{"dcic"};
  //     write_tau_to_file(file_name);
  //     write_time_to_file(file_name);
  //     RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Files written");
  //     logged_ = true;
  //   }
  // }
  /*Uncomment for logging */
}

Params Controller::defaultParameters() {
  Params p;
  for(int i = 0; i < 2; i++){
    p.params[i].stiffness.setIdentity();
    p.params[i].stiffness.topLeftCorner(3, 3) << 400 * Eigen::Matrix3d::Identity();
    p.params[i].stiffness.bottomRightCorner(3, 3) << 20 * Eigen::Matrix3d::Identity();
    p.params[i].damping_ratio = Vector6d::Constant(0.8);
    p.params[i].nullspace_stiffness = 10;
  }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Params set");
  
  return p;
}

Pose Controller::getCurrentPoseImpl() {
  Pose p;
  for(int i = 0; i < 2; i++){
    const auto robot_state = robot_data_[i]->state();
    Eigen::Map<const Vector7d> q(robot_state.q.data());
    Eigen::Affine3d transform(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    p.poses[i].position = transform.translation();
    p.poses[i].orientation = Eigen::Quaterniond(transform.linear());
    p.poses[i].q_n = q;
  }
  // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "getCurrentPoseImpl: " << p.position.transpose() << " " << p.orientation);
  return p;
}

void Controller::startImpl() {
  // A fresh activation starts from rest with no carried-over rate state.
  for (int i = 0; i < 2; ++i) {
    previous_tau_[i].setZero();
    last_raw_tau_[i].setZero();
    last_applied_tau_[i].setZero();
  }
  previous_time_ = 0.0;
  have_previous_time_ = false;
  nonfinite_cycles_ = 0;
  saturated_cycles_ = 0;
  rate_limited_cycles_ = 0;
}

bool Controller::hasOffsetImpl() {
  return getOffset().poses[0].orientation.norm() != 0 || getOffset().poses[1].position.norm() != 0;
}

void Controller::resetOffset() {
  Pose p;
  for(int i = 0; i<2; i++){
    p.poses[i].position = Vector3d::Zero();
    p.poses[i].orientation = Quaterniond(1, 0, 0, 0);
    p.poses[i].q_n = Vector7d::Zero();
  }
  setOffset(p);
}

