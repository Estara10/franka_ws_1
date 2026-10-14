#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace a1g {

/// Fresh, post-action joint feedback window. Values are the ordered 14 arm
/// joints; cumulative displacement is intentionally measured elsewhere.
class JointStabilityWindow {
 public:
  JointStabilityWindow(double duration_sec, double max_joint_spread_rad)
      : duration_sec_(std::max(0.0, duration_sec)),
        max_joint_spread_rad_(std::max(0.0, max_joint_spread_rad)) {}

  bool update(unsigned long long seq, double recv_steady_sec,
              const std::vector<double>& positions) {
    if (seq == 0 || !std::isfinite(recv_steady_sec) || positions.empty() ||
        !std::all_of(positions.begin(), positions.end(),
                     [](double value) { return std::isfinite(value); })) {
      return false;
    }
    if (seq <= last_seq_ || (!samples_.empty() && recv_steady_sec <= samples_.back().first)) {
      return false;
    }
    if (samples_.empty()) {
      last_seq_ = seq;
      samples_.push_back({recv_steady_sec, positions});
      min_ = positions;
      max_ = positions;
      return true;
    }
    if (positions.size() != min_.size()) {
      reset();
      return false;
    }
    last_seq_ = seq;
    samples_.push_back({recv_steady_sec, positions});
    for (std::size_t i = 0; i < positions.size(); ++i) {
      min_[i] = std::min(min_[i], positions[i]);
      max_[i] = std::max(max_[i], positions[i]);
    }
    return true;
  }

  bool stable() const {
    return samples_.size() >= 2 && duration() >= duration_sec_ &&
           maxSpread() <= max_joint_spread_rad_;
  }
  std::size_t samples() const { return samples_.size(); }
  double duration() const {
    return samples_.size() < 2 ? 0.0 : samples_.back().first - samples_.front().first;
  }
  double maxSpread() const {
    if (min_.empty() || min_.size() != max_.size()) return 0.0;
    double spread = 0.0;
    for (std::size_t i = 0; i < min_.size(); ++i) spread = std::max(spread, max_[i] - min_[i]);
    return spread;
  }
  const std::vector<double>& latestPositions() const { return samples_.back().second; }
  unsigned long long lastSeq() const { return last_seq_; }
  double lastTime() const { return samples_.empty() ? 0.0 : samples_.back().first; }

 private:
  void reset() {
    samples_.clear(); min_.clear(); max_.clear(); last_seq_ = 0;
  }
  double duration_sec_;
  double max_joint_spread_rad_;
  unsigned long long last_seq_ {0};
  std::vector<std::pair<double, std::vector<double>>> samples_;
  std::vector<double> min_;
  std::vector<double> max_;
};

inline bool poseWithinLimits(double position_error_m, double angle_error_rad,
                             double position_limit_m, double angle_limit_rad) {
  return std::isfinite(position_error_m) && std::isfinite(angle_error_rad) &&
         position_error_m >= 0.0 && angle_error_rad >= 0.0 &&
         position_error_m <= position_limit_m && angle_error_rad <= angle_limit_rad;
}

}  // namespace a1g
