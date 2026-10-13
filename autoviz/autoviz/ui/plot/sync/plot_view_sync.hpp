/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_view_sync.hpp
 * @brief Broadcast plot viewport / cursor changes across synced panels.
 *
 * Singleton registry: panels with a non-empty sync group publish range and
 * cursor changes, which fan out to other panels in the same group.
 *
 * @see PlotPanel
 * @see PlotChartWidget::viewRangeChanged()
 */

#pragma once

#include <QString>

#include <vector>

namespace autoviz {
namespace plot {

class PlotPanel;

/**
 * @class PlotViewSync
 * @brief Broadcasts plot x/y viewport and hover cursor across sync groups.
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
   * @brief Registers a panel to receive synced ranges / cursors.
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
   * @brief Publishes an X range from @p source to other panels in the same group.
   *
   * @param source Panel that initiated the change.
   * @param group Sync group name (empty = no-op).
   * @param min_x Visible minimum X.
   * @param max_x Visible maximum X.
   */
  void publishXRange(PlotPanel* source, const QString& group, double min_x,
                     double max_x);

  /**
   * @brief Publishes a left Y range from @p source to other panels in the group.
   *
   * @param source Panel that initiated the change.
   * @param group Sync group name (empty = no-op).
   * @param min_y Visible minimum Y.
   * @param max_y Visible maximum Y.
   */
  void publishYRange(PlotPanel* source, const QString& group, double min_y,
                     double max_y);

  /**
   * @brief Publishes a hover cursor X, or clears it when @p active is false.
   *
   * @param source Panel that initiated the change.
   * @param group Sync group name (empty = no-op).
   * @param active Whether a cursor X is valid.
   * @param x Cursor X when @p active is true.
   */
  void publishCursorX(PlotPanel* source, const QString& group, bool active,
                      double x);

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
