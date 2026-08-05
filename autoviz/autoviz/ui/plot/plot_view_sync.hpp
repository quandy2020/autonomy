/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

namespace autoviz {
namespace plot {

class PlotPanel;

/** Broadcasts timestamp plot x-axis viewport changes across synced panels. */
class PlotViewSync {
 public:
  static PlotViewSync& instance();

  void registerPanel(PlotPanel* panel);
  void unregisterPanel(PlotPanel* panel);
  void publishXRange(PlotPanel* source, double min_x, double max_x);

 private:
  PlotViewSync() = default;

  bool publishing_ = false;
  std::vector<PlotPanel*> panels_;
};

}  // namespace plot
}  // namespace autoviz
