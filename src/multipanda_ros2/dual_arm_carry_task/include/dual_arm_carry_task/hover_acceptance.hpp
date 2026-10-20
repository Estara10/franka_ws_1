#pragma once

#include <algorithm>
#include <cmath>

namespace hover_experiment {
// Hover-only acceptance: the 45 mm rise target is a tracking metric. Hover
// entry is confirmed from fresh physical rod clearance over the support.
// HOLD must still remain clear of support and may not fall by the existing
// 20 mm experimental slip window. Full transport acceptance is unchanged.
struct HeightMonitor {
  bool lifted{false};
  bool rise_target_reached{false};
  double peak_rise{0.0};
  double last_rise{0.0};
  double hold_start_rise{0.0};

  void update(double rise, bool lift_phase) {
    if (!std::isfinite(rise)) return;
    peak_rise = std::max(peak_rise, rise);
    last_rise = rise;
    if (lift_phase && rise >= 0.045) rise_target_reached = true;
  }
  bool confirmSupportClearance(double clearance) {
    if (!std::isfinite(clearance) || clearance <= 0.0) return false;
    lifted = true;
    return true;
  }
  bool holdValid(double clearance) const {
    return lifted && std::isfinite(clearance) && clearance > 0.003 &&
           last_rise >= hold_start_rise - 0.020;
  }
};
}  // namespace hover_experiment
