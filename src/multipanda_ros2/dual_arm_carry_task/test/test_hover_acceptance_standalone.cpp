#include "dual_arm_carry_task/hover_acceptance.hpp"
#include <cassert>
#include <limits>

int main() {
  hover_experiment::HeightMonitor h;
  h.update(0.044, true);
  assert(!h.rise_target_reached);
  h.update(0.046, true);
  assert(h.rise_target_reached);
  assert(!h.lifted);  // rise target is tracked, not the hover-entry gate
  assert(h.confirmSupportClearance(0.044));
  h.hold_start_rise = 0.046;
  h.update(0.0449, false);  // previous run's sub-mm threshold crossing
  assert(h.holdValid(0.044));
  assert(!h.holdValid(0.002));  // return to support is not suspended hold
  h.update(0.025, false);
  assert(!h.holdValid(0.025));  // latch does not waive drop protection
  assert(!h.holdValid(std::numeric_limits<double>::quiet_NaN()));
  hover_experiment::HeightMonitor clearance_entry;
  clearance_entry.update(0.043767, true);  // B_11 reached clearance below 45 mm rise
  assert(!clearance_entry.rise_target_reached);
  assert(clearance_entry.confirmSupportClearance(0.044499));
  clearance_entry.hold_start_rise = clearance_entry.last_rise;
  assert(clearance_entry.holdValid(0.044499));
  assert(!clearance_entry.confirmSupportClearance(0.0));
  hover_experiment::HeightMonitor unsupported;
  unsupported.update(0.050, true);
  assert(!unsupported.confirmSupportClearance(0.0));
  assert(!unsupported.lifted);
  hover_experiment::HeightMonitor never_lifted;
  never_lifted.update(0.047, false);  // HOLD cannot establish LIFT acceptance
  assert(!never_lifted.lifted);
  assert(!never_lifted.rise_target_reached);
}
