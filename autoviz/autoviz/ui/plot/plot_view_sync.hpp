/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_view_sync.hpp
 * @brief Broadcast timestamp-plot X-axis viewport changes across synced panels.
 *
 * Singleton registry: panels with @c sync_with_other_plots call
 * @ref registerPanel() and publish range changes via @ref publishXRange(),
 * which fans out to @ref PlotPanel::applySyncedXRange().
 *
 * @see PlotPanel
 * @see PlotChartWidget::viewRangeChanged()
 */

#pragma once

#include <vector>

namespace autoviz {
namespace plot {

class PlotPanel;

/**
 * @class PlotViewSync
 * @brief Broadcasts timestamp plot x-axis viewport changes across synced panels.
 *
 * @note @c publishing_ prevents re-entrant publish loops when applying a
 *       synced range triggers another @c viewRangeChanged.
 */
class PlotViewSync {
 public:
  /**
   * @brief Returns the process singleton.
   *
   * @return Shared sync controller.
   */
  static PlotViewSync& instance();

  /**
   * @brief Registers a panel to receive synced X ranges.
   *
   * @param panel Non-owning panel pointer (must @ref unregisterPanel() on destroy).
   */
  void registerPanel(PlotPanel* panel);

  /**
   * @brief Removes a panel from the sync registry.
   *
   * @param panel Panel previously passed to @ref registerPanel().
   */
  void unregisterPanel(PlotPanel* panel);

  /**
   * @brief Publishes an X range from @p source to all other registered panels.
   *
   * Skips @p source and no-ops while already publishing.
   *
   * @param source Panel that initiated the change.
   * @param min_x Visible minimum X.
   * @param max_x Visible maximum X.
   */
  void publishXRange(PlotPanel* source, double min_x, double max_x);

 private:
  /** Private default constructor for the singleton. */
  PlotViewSync() = default;

  /** Re-entrancy guard during fan-out. */
  bool publishing_ = false;

  /** Registered panels (non-owning). */
  std::vector<PlotPanel*> panels_;
};

}  // namespace plot
}  // namespace autoviz
