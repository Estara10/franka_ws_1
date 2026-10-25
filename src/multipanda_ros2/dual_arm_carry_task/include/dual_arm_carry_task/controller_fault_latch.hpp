// Copyright (c) 2026 The Franka Dual Arm Project
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>

namespace dual_arm_carry_task
{

// A controller-health query is fail-closed: a latched fault, a controller-state
// conflict, or a sustained loss of complete health observations permanently
// stops this orchestration run. It is deliberately not a controller-recovery
// policy.
//
// A single request that does not answer is not a fault by itself. It only
// becomes one when no complete observation has been confirmed for longer than
// kMaxStatusAgeNs. That budget is five health-query intervals (50 ms), 25
// reference periods (10 ms), and half of the 500 ms MMC reference watchdog.
class ControllerFaultLatch
{
public:
  bool observe(bool service_ready, bool response_ready, bool mmc_active, bool faulted,
               const std::string& source, std::uint64_t fault_sequence,
               std::uint64_t fault_time_ns, std::uint64_t status_time_ns,
               std::uint64_t now_ns, std::uint64_t status_age_budget_ns = kMaxStatusAgeNs)
  {
    if (!healthy_) return false;
    if (!service_ready || !response_ready) {
      latch("controller fault status unavailable");
    } else if (!mmc_active) {
      latch("MMC is not active");
    } else if (status_time_ns == 0 || now_ns < status_time_ns ||
               now_ns - status_time_ns > status_age_budget_ns) {
      latch("controller fault status stale or clock invalid");
    } else if (faulted) {
      const std::string detail = source.empty() ? "unknown" : source;
      latch("controller fault latched: " + detail + " sequence=" +
            std::to_string(fault_sequence) + " time_ns=" + std::to_string(fault_time_ns));
    }
    if (healthy_) last_good_ns_ = now_ns;
    return healthy_;
  }

  // Called when a query round did not produce a complete observation. It is
  // tolerated while a recent complete observation still exists. The caller may
  // use a longer budget outside the reference-publishing phase (for example,
  // while the bridge is armed and waiting for the lift gate).
  bool observeQueryMiss(std::uint64_t now_ns, std::uint64_t budget_ns = kMaxStatusAgeNs)
  {
    if (!healthy_) return false;
    const std::string suffix = " " + std::to_string(budget_ns / 1000000ULL) +
        " ms freshness budget";
    if (last_good_ns_ == 0) {
      if (first_unobserved_ns_ == 0) first_unobserved_ns_ = now_ns;
      if (now_ns > first_unobserved_ns_ && now_ns - first_unobserved_ns_ > budget_ns) {
        latch("controller health unavailable beyond" + suffix);
      }
      return healthy_;
    }
    if (now_ns > last_good_ns_ && now_ns - last_good_ns_ > budget_ns) {
      latch("controller health status unavailable beyond" + suffix);
    }
    return healthy_;
  }

  // An already-confirmed active-controller set that no longer matches the
  // expected exclusive state is an immediate failure, not a transient miss.
  bool latchControllerStateConflict(const std::string& detail)
  {
    if (!healthy_) return false;
    latch("controller state conflict: " + detail);
    return healthy_;
  }

  bool healthy() const { return healthy_; }
  const std::string& failure() const { return failure_; }
  std::uint64_t lastGoodNs() const { return last_good_ns_; }
  static constexpr std::uint64_t kMaxStatusAgeNs = 250000000ULL;

private:
  void latch(const std::string& reason)
  {
    healthy_ = false;
    failure_ = reason;
  }

  bool healthy_{true};
  std::string failure_;
  std::uint64_t last_good_ns_{0};
  std::uint64_t first_unobserved_ns_{0};
};

}  // namespace dual_arm_carry_task
