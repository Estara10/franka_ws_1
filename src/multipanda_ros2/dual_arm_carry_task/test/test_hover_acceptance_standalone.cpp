#include "dual_arm_carry_task/hover_acceptance.hpp"
#include <cassert>
#include <limits>

int main() {
  hover_experiment::HeightMonitor h;
  h.update(0.044, true);
  assert(!h.lifted);
  h.update(0.046, true);
  assert(h.lifted);
  h.hold_start_rise = 0.046;
  h.update(0.0449, false);  // previous run's sub-mm threshold crossing
  assert(h.holdValid(0.044));
  assert(!h.holdValid(0.002));  // return to support is not suspended hold
  h.update(0.025, false);
  assert(!h.holdValid(0.025));  // latch does not waive drop protection
  assert(!h.holdValid(std::numeric_limits<double>::quiet_NaN()));
  hover_experiment::HeightMonitor never_lifted;
  never_lifted.update(0.047, false);  // HOLD cannot establish LIFT acceptance
  assert(!never_lifted.lifted);
}
