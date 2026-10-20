#pragma once

#include <chrono>
#include <limits>
#include <string>

namespace dual_arm_carry_task
{
// Small production guard for the single outstanding MuJoCo truth request used
// by the hover protection loop. A late response cannot clear a latched fault.
class AsyncTruthGuard
{
public:
  using Clock = std::chrono::steady_clock;
  explicit AsyncTruthGuard(std::chrono::duration<double> freshness_budget)
  : budget_(std::chrono::duration_cast<Clock::duration>(freshness_budget)) {}

  void seedSample(Clock::time_point received) { sample_received_ = received; has_sample_ = true; }
  bool canStartRequest() const { return !faulted_ && !in_flight_; }
  // Discards an outstanding request without latching a fault. Needed when the
  // caller deliberately leaves the monitoring loop for a long, bounded
  // planning phase and will start a fresh request afterwards.
  void abandonRequest() { in_flight_ = false; }
  void beginRequest(Clock::time_point started)
  {
    if (canStartRequest()) { in_flight_ = true; request_started_ = started; }
  }
  bool observe(Clock::time_point now)
  {
    if (faulted_) return false;
    if (in_flight_ && now - request_started_ >= budget_) return fail("request_timeout");
    if (!has_sample_ || now - sample_received_ >= budget_) return fail("sample_stale");
    return true;
  }
  bool completeRequest(Clock::time_point received)
  {
    if (faulted_ || !in_flight_) return false;
    if (received - request_started_ >= budget_) return fail("request_timeout");
    in_flight_ = false;
    sample_received_ = received;
    has_sample_ = true;
    return true;
  }
  bool fail(const std::string& reason)
  {
    if (!faulted_) { faulted_ = true; failure_ = reason; }
    in_flight_ = false;
    return false;
  }
  bool faulted() const { return faulted_; }
  bool inFlight() const { return in_flight_; }
  const std::string& failure() const { return failure_; }
  double requestElapsed(Clock::time_point now) const
  {
    return in_flight_ ? std::chrono::duration<double>(now-request_started_).count() : 0.0;
  }
  double sampleAge(Clock::time_point now) const
  {
    return has_sample_ ? std::chrono::duration<double>(now-sample_received_).count() :
        std::numeric_limits<double>::infinity();
  }

private:
  Clock::duration budget_;
  Clock::time_point request_started_{};
  Clock::time_point sample_received_{};
  bool has_sample_{false};
  bool in_flight_{false};
  bool faulted_{false};
  std::string failure_;
};
}  // namespace dual_arm_carry_task
