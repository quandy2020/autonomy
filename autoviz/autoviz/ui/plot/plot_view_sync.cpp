/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_view_sync.hpp"

#include <algorithm>

#include "autoviz/ui/plot/plot_panel.hpp"

namespace autoviz {
namespace plot {

PlotViewSync& PlotViewSync::instance() {
  static PlotViewSync sync;
  return sync;
}

void PlotViewSync::registerPanel(PlotPanel* panel) {
  if (panel == nullptr) {
    return;
  }
  if (std::find(panels_.begin(), panels_.end(), panel) == panels_.end()) {
    panels_.push_back(panel);
  }
}

void PlotViewSync::unregisterPanel(PlotPanel* panel) {
  panels_.erase(std::remove(panels_.begin(), panels_.end(), panel), panels_.end());
}

void PlotViewSync::publishXRange(PlotPanel* source, double min_x, double max_x) {
  if (publishing_ || source == nullptr) {
    return;
  }
  publishing_ = true;
  for (PlotPanel* panel : panels_) {
    if (panel == nullptr || panel == source) {
      continue;
    }
    panel->applySyncedXRange(min_x, max_x);
  }
  publishing_ = false;
}

}  // namespace plot
}  // namespace autoviz
