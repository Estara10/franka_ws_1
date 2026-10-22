#pragma once

#include <Eigen/Dense>
#include <multi_mode_controller/base/panda_controller_base.h>
#include <multi_mode_controller/utils/torque_limits.h>
#include <multi_mode_controller/utils/redundancy_resolution.h>

namespace panda_controllers {

struct PandaCartesianImpedanceControllerPose {
  // Explicit defaults: these objects are buffered and copied before every
  // field is written, so an uninitialised Eigen member would be read as a
  // trajectory target.
  Eigen::Matrix<double, 7, 1> q_n = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  // Reference twist [v; w] in the same frame as the pose error and the
  // zero Jacobian (the arm base frame). Zero means "hold".
  Eigen::Matrix<double, 6, 1> twist = Eigen::Matrix<double, 6, 1>::Zero();
  // Explicit payload feedforward force [N] in the same base frame as the
  // zero Jacobian. It is mapped through J_linear^T and added before the
  // shared magnitude/rate limits. Zero keeps the original law.
  Eigen::Vector3d ff_force = Eigen::Vector3d::Zero();
  bool use_cooperative_wrench = false;
  Eigen::Matrix<double, 6, 1> cooperative_wrench = Eigen::Matrix<double, 6, 1>::Zero();
};

struct PandaCartesianImpedanceControllerParams {
  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 1> damping_ratio;
  double nullspace_stiffness;
};

struct DualCartesianImpedanceControllerPose {
  DualCartesianImpedanceControllerPose() : poses(2) {};
  std::vector<PandaCartesianImpedanceControllerPose> poses;
};

struct DualCartesianImpedanceControllerParams {
  DualCartesianImpedanceControllerParams() : params(2) {};
  std::vector<PandaCartesianImpedanceControllerParams> params;
};  

class ComlessDualCartesianImpedanceController :
    public virtual PandaControllerBase<DualCartesianImpedanceControllerParams,
                                       DualCartesianImpedanceControllerPose> {
 public:
  virtual ~ComlessDualCartesianImpedanceController() = default;
 protected:
  bool bCollisionAvoidance = true;
  bool bManipulability = false;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const DualCartesianImpedanceControllerPose& desired,
      const DualCartesianImpedanceControllerParams& p) override final;
  DualCartesianImpedanceControllerParams defaultParameters() override final;
  DualCartesianImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
  void startImpl() override;

  // Torque protection state. The rate limit needs the torque that was actually
  // applied last cycle and the measured control period.
  Eigen::Matrix<double, 7, 1> previous_tau_[2];
  double previous_time_{0.0};
  bool have_previous_time_{false};
  unsigned long long nonfinite_cycles_{0};
  unsigned long long saturated_cycles_{0};
  unsigned long long rate_limited_cycles_{0};
  unsigned long long limit_log_cycle_{0};
  Eigen::Matrix<double, 7, 1> last_raw_tau_[2];
  Eigen::Matrix<double, 7, 1> last_applied_tau_[2];
};

}
