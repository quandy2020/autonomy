/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/sync/plot_view_sync.hpp"

#include <algorithm>

#include "autoviz/ui/plot/panel/plot_panel.hpp"

namespace autoviz {
namespace plot {
namespace {

bool SameGroup(const QString& a, const QString& b) {
  return !a.isEmpty() && a == b;
}

}  // namespace

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

void PlotViewSync::publishXRange(PlotPanel* source, const QString& group,
                                 double min_x, double max_x) {
  if (publishing_ || source == nullptr || group.isEmpty()) {
    return;
  }
  publishing_ = true;
  for (PlotPanel* panel : panels_) {
    if (panel == nullptr || panel == source) {
      continue;
    }
    if (!SameGroup(panel->effectiveSyncGroup(), group)) {
      continue;
    }
    panel->applySyncedXRange(min_x, max_x);
  }
  publishing_ = false;
}

void PlotViewSync::publishYRange(PlotPanel* source, const QString& group,
                                 double min_y, double max_y) {
  if (publishing_ || source == nullptr || group.isEmpty()) {
    return;
  }
  publishing_ = true;
  for (PlotPanel* panel : panels_) {
    if (panel == nullptr || panel == source) {
      continue;
    }
    if (!SameGroup(panel->effectiveSyncGroup(), group) ||
        !panel->syncYAxisEnabled()) {
      continue;
    }
    panel->applySyncedYRange(min_y, max_y);
  }
  publishing_ = false;
}

void PlotViewSync::publishCursorX(PlotPanel* source, const QString& group,
                                  bool active, double x) {
  if (publishing_ || source == nullptr || group.isEmpty()) {
    return;
  }
  publishing_ = true;
  for (PlotPanel* panel : panels_) {
    if (panel == nullptr || panel == source) {
      continue;
    }
    if (!SameGroup(panel->effectiveSyncGroup(), group) ||
        !panel->syncCursorEnabled()) {
      continue;
    }
    panel->applySyncedCursorX(active, x);
  }
  publishing_ = false;
}

}  // namespace plot
}  // namespace autoviz
