#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>

#include <multi_mode_controller/base/panda_controller_interface.h>
#include <multi_mode_controller/utils/buffered.h>

namespace panda_controllers {

template <typename Parameters, typename Pose>
class PandaControllerBase : public PandaControllerInterface {
 public:
  virtual ~PandaControllerBase() = default;
  Pose getCurrentPose() {
    std::lock_guard<std::recursive_mutex> lock(current_pose_mutex_);
    return current_;
  }
  Pose getDesiredPose() {
    return desired_.getContent();
  }
  Pose getDesiredPoseBuffered() {
    return desired_.get();
  }
  Pose getLastDesiredPose() {
    return last_desired_;
  }
  Pose getOffset() {
    return offset_;
  }
  Parameters getParameters() {
    return p_.getContent();
  }
  Parameters getParametersBuffered() {
    return p_.get();
  }
  void setDesiredPoseBuffered(const Pose& p) {
    desired_.setBuffer(p);
  }
  void setError(const bool& error) {
    error_.store(error, std::memory_order_release);
  }
  void setOffset(const Pose& p) {
    offset_ = p;
  }
  void setParametersBuffered(const Parameters& p) {
    p_.setBuffer(p);
  }
  void updateCurrentPose() {
    std::lock_guard<std::recursive_mutex> lock(current_pose_mutex_);
    current_ = getCurrentPoseImpl();
  }
 protected:
  virtual bool init_(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) override {
    p_.setBuffer(defaultParameters());
    return initImpl(robot_data, node, name, resource);
  }
  void start_() override {
    setError(false);
    move_timeout_ = 3;
    updateCurrentPose();
    resetOffset();
    has_offset_ = false;
    desired_.setBuffer(current_);
    desired_.update();
    last_reference_ns_.store(nowSteadyNs(), std::memory_order_release);
    reference_received_.store(false, std::memory_order_release);
    reference_fault_logged_.store(false, std::memory_order_release);
    startImpl();
  }
  void stop_() override {
    setError(false);
    move_timeout_ = 3;
    stopImpl();
  }
  // Called only after a goal has passed the controller-specific validation.
  // The timestamp is intentionally independent of ROS time so paused or
  // simulated clocks cannot disable the watchdog.
  void markReferenceReceived() {
    last_reference_ns_.store(nowSteadyNs(), std::memory_order_release);
    reference_received_.store(true, std::memory_order_release);
  }

  virtual bool referenceWatchdogEnabled() const { return false; }
  virtual double referenceTimeoutSec() const { return 0.5; }
  virtual double referenceStartupTimeoutSec() const { return 2.0; }
 private:
  std::recursive_mutex current_pose_mutex_;
  Buffered<Parameters> p_;
  Buffered<Pose> desired_;
  Pose current_, last_desired_, offset_;
  int move_timeout_;
  bool has_offset_;
  void computeTau_(const std::vector<std::array<double, 7>*>& tau)
      override final {
    updateCurrentPose();
    p_.update();
    ++move_timeout_;
    has_offset_ = hasOffsetImpl();
    if (desired_.updatePending()) {
      last_desired_ = desired_.getContent();
      desired_.update();
      move_timeout_ = 0;
    }
    const Pose desired_snapshot = desired_.getContent();
    const Parameters parameters_snapshot = p_.getContent();
    if (referenceWatchdogEnabled()) {
      const auto now_ns = nowSteadyNs();
      const double age = static_cast<double>(now_ns -
          last_reference_ns_.load(std::memory_order_acquire)) * 1e-9;
      const double timeout = reference_received_.load(std::memory_order_acquire) ?
          referenceTimeoutSec() : referenceStartupTimeoutSec();
      if (age > timeout) {
        if (!reference_fault_logged_.exchange(true, std::memory_order_acq_rel)) {
          RCLCPP_ERROR(node_->get_logger(),
                       "[REFERENCE_WATCHDOG] reference timeout: age=%.3f s timeout=%.3f s; "
                       "holding last valid target and latching controller fault",
                       age, timeout);
        }
        setError(true);
      }
    }
    // On a watchdog fault, keep applying the last valid target.  The
    // controller manager observes hasError() and propagates the fault; this
    // avoids the unsafe alternative of abruptly writing zero torque while a
    // load may be suspended.
    computeTauImpl(tau, desired_snapshot, parameters_snapshot);
  }
  virtual void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const Pose& desired, const Parameters& p) = 0;
  virtual Parameters defaultParameters() = 0;
  virtual Pose getCurrentPoseImpl() = 0;
  bool hasOffset_() override final {
    return has_offset_;
  }
  virtual bool hasOffsetImpl() {
    return false;
  }
  virtual bool initImpl(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) {
    (void)robot_data;
    (void)node;
    (void)name;
    (void)resource;
    return true;
  }
  virtual bool moving_() override final {
    return move_timeout_ < 3 || has_offset_;
  }
  virtual void resetOffset() {}
  virtual void startImpl() {}
  virtual void stopImpl() {}

  static std::int64_t nowSteadyNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::atomic<std::int64_t> last_reference_ns_{0};
  std::atomic<bool> reference_received_{false};
  std::atomic<bool> reference_fault_logged_{false};
};

}
