#pragma once

#include <chrono>
#include <cstdint>

namespace dual_arm_carry_task
{
enum class MmcActivationResult
{
  ACCEPTED,
  SWITCH_FAILED,
  CONTROLLER_STATE_INVALID,
  STATUS_UNAVAILABLE,
  PROTECTION_FAILED,
  CONTROLLER_UNHEALTHY,
  FIRST_REFERENCE_TIMEOUT
};

// Shared production/test ordering contract. `deadline` covers the strict
// switch, active-state confirmation, subscription discovery and the first
// callback-validated goal acknowledgement.
template<class Hooks>
MmcActivationResult activateAndPrimeMmc(
    Hooks& hooks, std::chrono::steady_clock::time_point deadline)
{
  auto fail = [&hooks](MmcActivationResult result) {
    hooks.stopPublishing();
    return result;
  };
  if (!hooks.switchStrict(deadline)) return fail(MmcActivationResult::SWITCH_FAILED);
  if (!hooks.controllersMutuallyExclusive(deadline))
    return fail(MmcActivationResult::CONTROLLER_STATE_INVALID);

  std::uint64_t baseline_sequence = 0;
  if (!hooks.referenceSequence(deadline, baseline_sequence))
    return fail(MmcActivationResult::STATUS_UNAVAILABLE);

  while (std::chrono::steady_clock::now() < deadline) {
    if (!hooks.protectionHealthy()) return fail(MmcActivationResult::PROTECTION_FAILED);
    if (!hooks.controllerHealthy()) return fail(MmcActivationResult::CONTROLLER_UNHEALTHY);
    if (hooks.goalSubscriberReady()) {
      hooks.publishInitialReference();
      std::uint64_t accepted_sequence = baseline_sequence;
      if (!hooks.referenceSequence(deadline, accepted_sequence))
        return fail(MmcActivationResult::STATUS_UNAVAILABLE);
      if (accepted_sequence > baseline_sequence) return MmcActivationResult::ACCEPTED;
    }
    hooks.waitTick(deadline);
  }
  return fail(MmcActivationResult::FIRST_REFERENCE_TIMEOUT);
}
}  // namespace dual_arm_carry_task
