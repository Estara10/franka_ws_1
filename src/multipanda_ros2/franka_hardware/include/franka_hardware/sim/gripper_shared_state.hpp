// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>
#include <mutex>
#include <vector>

namespace franka_gripper {

inline constexpr double kMaxContactForcePerFingerN = 50.0;
// The requested contact hold maps to 90% of the 2*per-finger split-tendon
// actuator output (50 N/finger -> nominal -90 N actuator output).
inline constexpr double kContactHoldForceCommandFraction = 0.90;
// The fixed-arm direct-force trace produced 0.67/0.77 mm peak-to-peak gap
// motion while keeping each pad in contact on >=18/20 samples. A 0.5 mm
// window rejected that observed stable hold; 1.0 mm remains below the prior
// position-hold scatter (0.86/1.00 mm) and is paired with >=16/20 per finger.
inline constexpr double kContactForceStableGapSpreadM = 0.001;
inline constexpr int kContactForceStableOccupancySamples = 16;

inline bool contactForceWindowAccepted(std::size_t sample_count, double duration_s,
                                       int left_contact_samples, int right_contact_samples,
                                       double gap_spread_m) {
  return sample_count == 20 && std::isfinite(duration_s) && duration_s >= 0.18 &&
      left_contact_samples >= kContactForceStableOccupancySamples &&
      right_contact_samples >= kContactForceStableOccupancySamples &&
      std::isfinite(gap_spread_m) && gap_spread_m <= kContactForceStableGapSpreadM;
}
// During explicit contact hold, the symmetric tendon mode has k=0.5*1200=600
// N/m per finger coordinate and effective finger mass 0.015 kg + 0.1 kg
// armature. Critical damping is 2*sqrt(600*0.115)=16.61 N s/m. The model's
// default is 1 N s/m, so add 15.6 only while force hold is active.
inline constexpr double kContactHoldAdditionalFingerDampingNsPerM = 15.6;

inline double contactHoldFingerDamping(double model_damping, bool force_hold_active) {
  if (!std::isfinite(model_damping) || model_damping < 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  return model_damping + (force_hold_active ? kContactHoldAdditionalFingerDampingNsPerM : 0.0);
}

/// Per-gripper handoff between the action-server callbacks and the MuJoCo
/// physics thread.
///
/// Ownership rule, and the reason this is a single guarded struct rather than
/// several bare doubles:
///   * the PHYSICS THREAD is the only writer of `finger_q1` / `finger_q2` and of
///     `feedback_seq`, and the only caller that touches MuJoCo `qpos`/`ctrl`;
///   * the ACTION SERVER is the only writer of `command_per_finger`, and it
///     never writes MuJoCo state;
///   * every access takes `mutex`, and the lock is never held across a sleep or
///     across an action wait.
///
/// `feedback_seq` is what makes "N fresh samples" checkable: a reader that
/// re-reads the same physics value twice sees the same sequence number and
/// therefore cannot count it twice.
struct GripperSharedState {
  mutable std::mutex mutex;

  /// Target for the tendon position actuator, in metres of PER-FINGER
  /// displacement. Written only by the action server.
  double command_per_finger {0.0};
  // Commands remain owned by the action server; measurements by physics.
  double closing_force_limit_per_finger_n {0.0};
  double hold_force_per_finger_n {0.0};  // 0 = ordinary position mode
  double hold_reference_tendon_length {0.0};
  bool force_hold_supported {false};
  bool force_hold_fault {false};
  bool direct_force_mode_active {false};
  bool stop_at_current_position {false};
  // `left` and `right` are this gripper's two finger bodies. Each magnitude
  // sums its pad/rod contact points; rod_force_xyz is the net world force on the
  // rod from both fingers of this gripper. The task sums the two arm vectors.
  double normal_left_n {0.0}, normal_right_n {0.0};
  double tangent_left_n {0.0}, tangent_right_n {0.0};
  double rod_force_x_n {0.0}, rod_force_y_n {0.0}, rod_force_z_n {0.0};
  double simulation_time {0.0};
  std::uint32_t rod_pad_contact_count {0};
  bool weld_left_active {false}, weld_right_active {false};
  bool rod_constraint_active {false};


  /// Real MuJoCo finger joint positions in metres. Written only by the physics
  /// thread.
  double finger_q1 {0.0};
  double finger_q2 {0.0};

  /// Bumped by the physics thread on every feedback update.
  unsigned long long feedback_seq {0};
  bool left_finger_rod_contact {false};
  bool right_finger_rod_contact {false};
};

/// One consistent copy of the shared state, taken under the lock.
struct GripperFeedback {
  double command_per_finger {0.0};
  // Commands remain owned by the action server; measurements by physics.
  double closing_force_limit_per_finger_n {0.0};
  double hold_force_per_finger_n {0.0};  // 0 = ordinary position mode
  double hold_reference_tendon_length {0.0};
  bool force_hold_supported {false};
  bool force_hold_fault {false};
  bool direct_force_mode_active {false};
  bool stop_at_current_position {false};
  double normal_left_n {0.0}, normal_right_n {0.0};
  double tangent_left_n {0.0}, tangent_right_n {0.0};
  double rod_force_x_n {0.0}, rod_force_y_n {0.0}, rod_force_z_n {0.0};
  double simulation_time {0.0};
  std::uint32_t rod_pad_contact_count {0};
  bool weld_left_active {false}, weld_right_active {false};
  bool rod_constraint_active {false};

  double finger_q1 {0.0};
  double finger_q2 {0.0};
  unsigned long long feedback_seq {0};
  bool left_finger_rod_contact {false};
  bool right_finger_rod_contact {false};
  /// False when any of the three numbers above is not finite, or when no
  /// physics feedback has been published yet.
  bool valid {false};

  /// The real total opening. Both finger joints are read separately and summed:
  /// this is not one finger doubled.
  double totalGap() const { return finger_q1 + finger_q2; }
};

/// Reads one consistent snapshot. Never blocks on anything but the mutex.
inline GripperFeedback readFeedback(const std::shared_ptr<GripperSharedState>& state) {
  GripperFeedback out;
  if (!state) {
    return out;
  }
  std::lock_guard<std::mutex> lock(state->mutex);
  out.command_per_finger = state->command_per_finger;
  out.closing_force_limit_per_finger_n = state->closing_force_limit_per_finger_n;
  out.finger_q1 = state->finger_q1;
  out.finger_q2 = state->finger_q2;
  out.hold_force_per_finger_n = state->hold_force_per_finger_n;
  out.hold_reference_tendon_length = state->hold_reference_tendon_length;
  out.force_hold_supported = state->force_hold_supported;
  out.force_hold_fault = state->force_hold_fault;
  out.direct_force_mode_active = state->direct_force_mode_active;
  out.stop_at_current_position = state->stop_at_current_position;
  out.normal_left_n = state->normal_left_n; out.normal_right_n = state->normal_right_n;
  out.tangent_left_n = state->tangent_left_n; out.tangent_right_n = state->tangent_right_n;
  out.rod_force_x_n = state->rod_force_x_n;
  out.rod_force_y_n = state->rod_force_y_n;
  out.rod_force_z_n = state->rod_force_z_n;
  out.simulation_time = state->simulation_time;
  out.rod_pad_contact_count = state->rod_pad_contact_count;
  out.weld_left_active = state->weld_left_active; out.weld_right_active = state->weld_right_active;
  out.rod_constraint_active = state->rod_constraint_active;
  out.feedback_seq = state->feedback_seq;
  out.left_finger_rod_contact = state->left_finger_rod_contact;
  out.right_finger_rod_contact = state->right_finger_rod_contact;
  out.valid = state->feedback_seq > 0ULL && std::isfinite(out.finger_q1) &&
              std::isfinite(out.finger_q2) && std::isfinite(out.command_per_finger);
  return out;
}

/// Writes the actuator target. Returns false (and writes nothing) for a
/// non-finite request, so a NaN can never be turned into a full close.
inline bool writeCommand(const std::shared_ptr<GripperSharedState>& state,
                         double per_finger_m, double closing_force_limit_n = 0.0) {
  if (!state || !std::isfinite(per_finger_m) || !std::isfinite(closing_force_limit_n) ||
      closing_force_limit_n < 0.0 || closing_force_limit_n > kMaxContactForcePerFingerN) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state->mutex);
  if (closing_force_limit_n > 0.0 && !state->force_hold_supported) return false;
  state->closing_force_limit_per_finger_n = closing_force_limit_n;
  state->hold_force_per_finger_n = 0.0;
  state->hold_reference_tendon_length = 0.0;
  state->force_hold_fault = false;
  state->stop_at_current_position = false;
  state->command_per_finger = per_finger_m;
  return true;
}

// A positive request is supported only by the checked 0.5/0.5 position tendon.
// The reference length is latched at first bilateral contact. The physics
// thread keeps the native tendon position servo and temporarily limits its
// actuator force to 2F (one half per finger); ordinary position commands do
// not enter this contact-only hold mode. The position target must fit ctrlrange.
inline bool writeForceHold(const std::shared_ptr<GripperSharedState>& state, double force_n,
                           double reference_tendon_length) {
  if (!state || !std::isfinite(force_n) || force_n <= 0.0 ||
      force_n > kMaxContactForcePerFingerN ||
      !std::isfinite(reference_tendon_length) || reference_tendon_length < 0.0 ||
      reference_tendon_length > 0.08) return false;
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->force_hold_supported || state->force_hold_fault) return false;
  state->closing_force_limit_per_finger_n = force_n;
  state->hold_force_per_finger_n = force_n;
  state->hold_reference_tendon_length = reference_tendon_length;
  state->stop_at_current_position = false;
  return true;
}

inline void requestForceHoldStopAtCurrent(const std::shared_ptr<GripperSharedState>& state,
                                          bool actuator_fault = false) {
  if (!state) return;
  std::lock_guard<std::mutex> lock(state->mutex);
  state->closing_force_limit_per_finger_n = 0.0;
  state->hold_force_per_finger_n = 0.0;
  state->hold_reference_tendon_length = 0.0;
  state->stop_at_current_position = true;
  state->force_hold_fault = state->force_hold_fault || actuator_fault;
}

inline void publishDirectForceModeActive(const std::shared_ptr<GripperSharedState>& state,
                                         bool active) {
  if (!state) return;
  std::lock_guard<std::mutex> lock(state->mutex);
  state->direct_force_mode_active = active;
}

inline void publishStoppedTendonPosition(const std::shared_ptr<GripperSharedState>& state,
                                          double tendon_position_m) {
  if (!state || !std::isfinite(tendon_position_m)) return;
  std::lock_guard<std::mutex> lock(state->mutex);
  state->command_per_finger = tendon_position_m;
  state->stop_at_current_position = false;
}

inline void failForceHoldAtCurrent(const std::shared_ptr<GripperSharedState>& state,
                                   double tendon_position_m) {
  requestForceHoldStopAtCurrent(state, true);
  publishStoppedTendonPosition(state, tendon_position_m);
}
inline double boundedForceLimitPositionTarget(double length, double reference_length,
                                              double kp, double force_n,
                                              double min_ctrl, double max_ctrl) {
  if (!std::isfinite(length) || !std::isfinite(reference_length) ||
      reference_length < 0.0 || reference_length > 0.08 ||
      !std::isfinite(kp) || kp <= 0.0 ||
      !std::isfinite(force_n) || force_n <= 0.0 ||
      force_n > kMaxContactForcePerFingerN ||
      !std::isfinite(min_ctrl) || !std::isfinite(max_ctrl) || min_ctrl > max_ctrl)
    return std::numeric_limits<double>::quiet_NaN();
  const double max_tendon_force = 2.0 * force_n;
  const double desired_tendon_force = std::clamp(
      kp * (reference_length - length) - max_tendon_force,
      -max_tendon_force, max_tendon_force);
  return std::clamp(length + desired_tendon_force / kp, min_ctrl, max_ctrl);
}

inline bool forceHoldActuatorForceRange(double force_n, double& lower, double& upper) {
  if (!std::isfinite(force_n) || force_n <= 0.0 ||
      force_n > kMaxContactForcePerFingerN) return false;
  const double tendon_limit = 2.0 * force_n;
  lower = -tendon_limit;
  upper = tendon_limit;
  return true;
}

/// EXPERIMENTAL, NOT A VALIDATED BASELINE: fixed position target for the
/// explicit contact hold. The target is latched from the first bilateral-
/// contact tendon length; the native position servo then supplies a restoring
/// response while its actuator force is capped by forceHoldActuatorForceRange().
/// This variant failed the isolated physical reproduction and is retained only
/// for traceability; `command_fraction` preserves the previous direct-force
/// hold's 90% operating point.
inline double forceHoldPositionTarget(double reference_tendon_length,
                                     double force_per_finger_n,
                                     double kp,
                                     double command_fraction,
                                     double ctrl_min, double ctrl_max) {
  if (!std::isfinite(reference_tendon_length) || reference_tendon_length < 0.0 ||
      reference_tendon_length > 0.08 || !std::isfinite(force_per_finger_n) ||
      force_per_finger_n <= 0.0 || force_per_finger_n > kMaxContactForcePerFingerN ||
      !std::isfinite(kp) || kp <= 0.0 || !std::isfinite(command_fraction) ||
      command_fraction <= 0.0 || command_fraction > 1.0 ||
      !std::isfinite(ctrl_min) || !std::isfinite(ctrl_max) || ctrl_min > ctrl_max) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Each split-tendon branch has coefficient 0.5, so one actuator-force unit
  // contributes half a unit to each finger. Keep the existing force level and
  // sign; changing only to a fixed position target adds common-mode restoring
  // stiffness that the direct-force command did not have.
  const double actuator_force = -2.0 * force_per_finger_n * command_fraction;
  const double target = reference_tendon_length + actuator_force / kp;
  // Do not silently turn an unavailable force into a smaller one by clipping
  // the position target. The physics thread treats NaN as an actuator fault.
  if (target < ctrl_min || target > ctrl_max)
    return std::numeric_limits<double>::quiet_NaN();
  return target;
}

/// Maps a positive per-finger closing-force request to the signed force on the
/// split-tendon actuator. Each tendon branch uses `finger_tendon_coefficient`
/// and the actuator multiplies tendon force by `actuator_gear`. Positive finger
/// q opens the hand, so a closing command is negative.
inline double directClosingTendonForceCommand(double force_per_finger_n,
                                              double finger_tendon_coefficient,
                                              double actuator_gear,
                                              double command_fraction,
                                              double ctrl_min, double ctrl_max) {
  if (!std::isfinite(force_per_finger_n) || force_per_finger_n <= 0.0 ||
      force_per_finger_n > kMaxContactForcePerFingerN ||
      !std::isfinite(finger_tendon_coefficient) || finger_tendon_coefficient <= 0.0 ||
      !std::isfinite(actuator_gear) || actuator_gear <= 0.0 ||
      !std::isfinite(command_fraction) || command_fraction <= 0.0 || command_fraction > 1.0 ||
      !std::isfinite(ctrl_min) || !std::isfinite(ctrl_max) || ctrl_min > ctrl_max)
    return std::numeric_limits<double>::quiet_NaN();
  const double actuator_force = -force_per_finger_n * command_fraction /
      (finger_tendon_coefficient * actuator_gear);
  if (actuator_force < ctrl_min || actuator_force > ctrl_max)
    return std::numeric_limits<double>::quiet_NaN();
  return actuator_force;
}

/// Publishes one physics sample. Called from the physics thread only.
inline void publishFeedback(const std::shared_ptr<GripperSharedState>& state,
                            double q1, double q2,
                            bool left_finger_rod_contact = false,
                            bool right_finger_rod_contact = false) {
  if (!state) {
    return;
  }
  std::lock_guard<std::mutex> lock(state->mutex);
  state->finger_q1 = q1;
  state->finger_q2 = q2;
  state->left_finger_rod_contact = left_finger_rod_contact;
  state->right_finger_rod_contact = right_finger_rod_contact;
  ++state->feedback_seq;
}

/// Decides whether the gripper has actually settled at a target.
///
/// Three properties matter here and are the reason this is not just a counter:
///
///  * a sample counts only if its sequence number is strictly newer than the
///    last counted one, so re-reading the same physics value can never build up
///    a "10 fresh samples" window;
///  * the threshold applies to the WHOLE window (`max - min` over its samples),
///    not to the difference between two adjacent samples, so a slow monotone
///    drift that moves less than the step threshold each tick still fails;
///  * the window is the LAST `required_` samples — it slides. Accumulating
///    min/max from the start of the move instead would keep the settling
///    transient in view forever: an observed run that held a dead-flat
///    0.08000 m for its last 200 cycles was still failing on the residual
///    spread of its own approach.
///
/// A sample outside `tol_m` of the target empties the window, so by
/// construction every sample in a satisfied window is inside tolerance.
class StabilityWindow {
 public:
  StabilityWindow(int required_samples, double tol_m, double max_delta_m)
      : required_(required_samples > 1 ? required_samples : 1),
        tol_(tol_m > 0.0 ? tol_m : 0.0),
        max_delta_(max_delta_m > 0.0 ? max_delta_m : 0.0),
        gaps_(static_cast<std::size_t>(required_samples > 1 ? required_samples : 1), 0.0) {}

  void reset() {
    have_last_seq_ = false;
    last_seq_ = 0ULL;
    filled_ = 0;
    next_ = 0;
  }

  /// Feeds one feedback snapshot taken at the requested `target_m`.
  /// Returns true only when the last `required_` fresh samples are all inside
  /// tolerance and their whole-window spread is within the threshold.
  /// `snapshot()` then holds the sample that satisfied it.
  bool update(const GripperFeedback& fb, double target_m) {
    if (!fb.valid || !std::isfinite(target_m)) {
      reset();
      return false;
    }
    if (have_last_seq_ && fb.feedback_seq == last_seq_) {
      // No new physics feedback: the same value must not be counted twice.
      return false;
    }
    have_last_seq_ = true;
    last_seq_ = fb.feedback_seq;

    const double gap = fb.totalGap();
    if (std::abs(gap - target_m) > tol_) {
      filled_ = 0;
      next_ = 0;
      return false;
    }
    gaps_[next_] = gap;
    next_ = (next_ + 1) % gaps_.size();
    if (filled_ < static_cast<int>(gaps_.size())) {
      ++filled_;
    }
    if (filled_ < required_) {
      return false;
    }
    if (spread() > max_delta_) {
      return false;
    }
    snapshot_ = fb;
    return true;
  }

  /// The feedback that satisfied the window. Only meaningful after update()
  /// returned true.
  const GripperFeedback& snapshot() const { return snapshot_; }
  /// How many of the window's samples are filled, at most `required()`.
  int count() const { return filled_; }
  int required() const { return required_; }
  /// `max - min` over the samples currently in the window.
  double spread() const {
    if (filled_ == 0) {
      return 0.0;
    }
    double lo = gaps_[0];
    double hi = gaps_[0];
    for (double g : gaps_) {
      lo = std::min(lo, g);
      hi = std::max(hi, g);
    }
    return hi - lo;
  }

 private:
  int required_;
  double tol_;
  double max_delta_;
  bool have_last_seq_ {false};
  unsigned long long last_seq_ {0};
  int filled_ {0};
  std::size_t next_ {0};
  std::vector<double> gaps_;
  GripperFeedback snapshot_;
};

/// Explicit object-contact command state used by the simulator Grasp action.
///
/// Before bilateral contact this advances toward the requested gap by at most
/// max_step_m. On the first fresh bilateral-contact sample it latches the
/// measured gap exactly once and fixes the target 0.5 mm inside it (total gap).
/// The transition remains rate limited. Contact loss clears the acceptance
/// window and freezes the command until bilateral contact returns.
class BilateralContactHold {
 public:
  enum class Stage { APPROACH, HOLD };

  struct Update {
    double command_gap_m {0.0};
    double hold_target_gap_m {0.0};
    bool fresh_sample {false};
    bool entered_hold {false};
    bool contact_lost {false};
    bool confirmed {false};
    int window_samples {0};
    double window_spread_m {0.0};
    Stage stage {Stage::APPROACH};
  };

  BilateralContactHold(int required_samples, double max_gap_spread_m,
                       double preload_m = 0.0005)
      : required_(required_samples > 1 ? required_samples : 1),
        max_spread_(std::max(0.0, max_gap_spread_m)),
        preload_m_(std::max(0.0, preload_m)) {}

  Update update(const GripperFeedback& fb,
                double current_command_gap_m,
                double approach_gap_m,
                double max_step_m) {
    Update out;
    out.command_gap_m = current_command_gap_m;
    out.hold_target_gap_m = hold_target_gap_m_;
    out.stage = stage_;
    if (!fb.valid || !std::isfinite(current_command_gap_m) ||
        !std::isfinite(approach_gap_m) || !std::isfinite(max_step_m) || max_step_m < 0.0) {
      resetWindow();
      return out;
    }
    if (have_seq_ && fb.feedback_seq <= last_seq_) {
      // Duplicate or out-of-order feedback is never evidence and cannot move
      // the command. In particular, it cannot fill a contact window.
      return out;
    }
    have_seq_ = true;
    last_seq_ = fb.feedback_seq;
    out.fresh_sample = true;

    const bool bilateral = fb.left_finger_rod_contact && fb.right_finger_rod_contact;
    if (stage_ == Stage::APPROACH && bilateral) {
      stage_ = Stage::HOLD;
      latched_contact_gap_m_ = std::clamp(fb.totalGap(), 0.0, 0.08);
      hold_target_gap_m_ = std::max(0.0, latched_contact_gap_m_ - preload_m_);
      out.entered_hold = true;
      resetWindow();
    } else if (stage_ == Stage::HOLD && !bilateral && was_bilateral_) {
      out.contact_lost = true;
      resetWindow();
    }

    if (stage_ == Stage::APPROACH) {
      const double delta = approach_gap_m - current_command_gap_m;
      if (std::abs(delta) > max_step_m) {
        out.command_gap_m = current_command_gap_m + std::copysign(max_step_m, delta);
      } else {
        out.command_gap_m = approach_gap_m;
      }
    } else if (!bilateral) {
      // Do not continue inward during a contact interruption. Keep the last
      // bounded command; a later fresh bilateral sample may resume toward the
      // same latched target, but can never add preload.
      out.command_gap_m = current_command_gap_m;
    } else {
      const double delta = hold_target_gap_m_ - current_command_gap_m;
      if (std::abs(delta) > max_step_m) {
        out.command_gap_m = current_command_gap_m + std::copysign(max_step_m, delta);
      } else {
        out.command_gap_m = hold_target_gap_m_;
      }
      if (bilateral) {
        gaps_.push_back(fb.totalGap());
        if (gaps_.size() > static_cast<std::size_t>(required_)) gaps_.erase(gaps_.begin());
        out.window_samples = static_cast<int>(gaps_.size());
        out.window_spread_m = spread();
        out.confirmed = gaps_.size() == static_cast<std::size_t>(required_) &&
                        out.window_spread_m <= max_spread_;
      }
    }
    was_bilateral_ = bilateral;
    out.hold_target_gap_m = hold_target_gap_m_;
    out.stage = stage_;
    if (stage_ != Stage::HOLD) {
      out.window_samples = 0;
      out.window_spread_m = 0.0;
    }
    return out;
  }

  Stage stage() const { return stage_; }
  double holdTargetGap() const { return hold_target_gap_m_; }
  double latchedContactGap() const { return latched_contact_gap_m_; }
  int windowSamples() const { return static_cast<int>(gaps_.size()); }
  double windowSpread() const { return spread(); }

 private:
  void resetWindow() { gaps_.clear(); }
  double spread() const {
    if (gaps_.empty()) return 0.0;
    const auto bounds = std::minmax_element(gaps_.begin(), gaps_.end());
    return *bounds.second - *bounds.first;
  }

  const int required_;
  const double max_spread_;
  const double preload_m_;
  Stage stage_ {Stage::APPROACH};
  bool have_seq_ {false};
  bool was_bilateral_ {false};
  unsigned long long last_seq_ {0ULL};
  double latched_contact_gap_m_ {0.0};
  double hold_target_gap_m_ {0.0};
  std::vector<double> gaps_;
};

/// A stable contact window is only an action success while the goal is still
/// live and inside its deadline. Shared with the server test to exercise the
/// exact final acceptance gate.
inline bool contactSuccessAllowed(bool stable_window_confirmed,
                                  bool canceled,
                                  bool deadline_expired) {
  return stable_window_confirmed && !canceled && !deadline_expired;
}

}  // namespace franka_gripper
