#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace dual_arm_carry_task {

enum class ServiceDiscoveryResult { READY, DEADLINE_EXPIRED };

// The caller owns one steady-clock deadline for the complete preflight. The
// wait callback must not sleep beyond the supplied next wake time.
template<typename Ready, typename WaitUntil>
ServiceDiscoveryResult waitForServiceUntil(
    std::chrono::steady_clock::time_point deadline,
    Ready&& ready,
    WaitUntil&& wait_until)
{
  while (std::chrono::steady_clock::now() < deadline) {
    if (ready()) return ServiceDiscoveryResult::READY;
    const auto next = std::min(
        deadline, std::chrono::steady_clock::now() + std::chrono::milliseconds(10));
    wait_until(next);
  }
  return ServiceDiscoveryResult::DEADLINE_EXPIRED;
}

enum class ControllerStateResult { MATCH, MISSING_JTC, MISSING_MMC, UNEXPECTED_STATE };

struct ControllerStateObservation {
  std::string jtc_state;
  std::string mmc_state;
};

inline ControllerStateResult evaluateControllerStates(
    const std::vector<std::pair<std::string, std::string>>& controllers,
    const std::string& jtc_name,
    const std::string& mmc_name,
    bool expect_mmc_active,
    ControllerStateObservation& observation)
{
  bool jtc_seen = false;
  bool mmc_seen = false;
  for (const auto& controller : controllers) {
    if (controller.first == jtc_name) {
      jtc_seen = true;
      observation.jtc_state = controller.second;
    }
    if (controller.first == mmc_name) {
      mmc_seen = true;
      observation.mmc_state = controller.second;
    }
  }
  if (!jtc_seen) return ControllerStateResult::MISSING_JTC;
  if (!mmc_seen) return ControllerStateResult::MISSING_MMC;
  const bool jtc_active = observation.jtc_state == "active";
  const bool mmc_active = observation.mmc_state == "active";
  return (mmc_active == expect_mmc_active && jtc_active != expect_mmc_active)
      ? ControllerStateResult::MATCH : ControllerStateResult::UNEXPECTED_STATE;
}

}  // namespace dual_arm_carry_task
