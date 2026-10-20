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
  void setError(const bool& error, const std::string& source = "controller_error") {
    if (error) {
      std::lock_guard<std::mutex> lock(fault_metadata_mutex_);
      if (!error_.load(std::memory_order_relaxed)) {
        fault_source_ = source;
        fault_steady_time_ns_.store(static_cast<std::uint64_t>(nowSteadyNs()), std::memory_order_release);
        fault_sequence_.fetch_add(1, std::memory_order_acq_rel);
        error_.store(true, std::memory_order_release);
      }
    } else {
      std::lock_guard<std::mutex> lock(fault_metadata_mutex_);
      error_.store(false, std::memory_order_release);
      fault_source_.clear();
      fault_steady_time_ns_.store(0, std::memory_order_release);
    }
  }
  std::string faultSource() const override {
    std::lock_guard<std::mutex> lock(fault_metadata_mutex_);
    return hasError() ? fault_source_ : "";
  }
  std::uint64_t faultSequence() const override {
    return hasError() ? fault_sequence_.load(std::memory_order_acquire) : 0;
  }
  std::uint64_t faultSteadyTimeNs() const override {
    return hasError() ? fault_steady_time_ns_.load(std::memory_order_acquire) : 0;
  }
  std::uint64_t referenceSequence() const override {
    return reference_sequence_.load(std::memory_order_acquire);
  }
  std::uint64_t referenceSteadyTimeNs() const override {
    return reference_steady_time_ns_.load(std::memory_order_acquire);
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
    reference_sequence_.store(0, std::memory_order_release);
    reference_steady_time_ns_.store(0, std::memory_order_release);
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
    const auto received_ns = nowSteadyNs();
    last_reference_ns_.store(received_ns, std::memory_order_release);
    reference_steady_time_ns_.store(static_cast<std::uint64_t>(received_ns), std::memory_order_release);
    reference_sequence_.fetch_add(1, std::memory_order_acq_rel);
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
        setError(true, "reference_watchdog");
      }
    }
    // On a watchdog fault the closed-loop law continues to use the last valid
    // target. This is not a claim that the resulting torque is a safe hold.
    // MultiModeController publishes the latched fault through its status
    // service; orchestration must stop sending references and report failure.
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
  std::atomic<std::uint64_t> reference_sequence_{0};
  std::atomic<std::uint64_t> reference_steady_time_ns_{0};
  std::atomic<bool> reference_fault_logged_{false};
  std::string fault_source_;
  mutable std::mutex fault_metadata_mutex_;
  std::atomic<std::uint64_t> fault_sequence_{0};
  std::atomic<std::uint64_t> fault_steady_time_ns_{0};
};

}
