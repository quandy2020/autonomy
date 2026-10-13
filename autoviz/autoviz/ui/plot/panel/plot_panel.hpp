/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_panel.hpp
 * @brief Plot panel — multi-series chart, legend, settings, and view sync.
 *
 * Subscribes per-series channels, drains queues on a tick timer, extracts
 * samples via @ref PlotFieldExtractor, and drives @ref PlotChartWidget.
 * Registers with @ref PlotViewSync when X-axis sync is enabled.
 *
 * ## Data flow
 *
 * - **In:** subscriptions → queues → extract / modifiers → @c points deque
 * - **Out:** @ref configChanged(); chrome signals; sync X via @ref PlotViewSync
 *
 * @see PlotChartWidget
 * @see PlotSettingsWidget
 * @see PlotPanelConfig
 * @see PlotViewSync
 */

#pragma once

#include <QWidget>

#include <limits>
#include <memory>
#include <vector>

#include "autoviz/ui/plot/plot_types.hpp"

class QAction;
class QButtonGroup;
class QDragEnterEvent;
class QDropEvent;
class QFocusEvent;
class QWheelEvent;
class QTimer;
class QToolButton;
class QScrollArea;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}

namespace plot {

class PlotChartWidget;
class PlotLegendWidget;
class PlotSettingsWidget;

/**
 * @class PlotPanel
 * @brief Dockable multi-series plot with tools, legend, and settings.
 *
 * ## Layout
 *
 * @code
 * ┌────────────────────────────────────────────────────┐
 * │ [select|pan|zoom|inspect|brush] [legend] [settings] │
 * ├──────────────────────────────────┬─────────────────┤
 * │ PlotChartWidget (+ legend float) │ Settings        │
 * │                                  │                 │
 * └──────────────────────────────────┴─────────────────┘
 * @endcode
 *
 * @note Does not own @ref common::VisualizationManager.
 */
class PlotPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs chart, legend, settings, tick timer, and tool buttons.
   *
   * @param manager Non-owning visualization manager.
   * @param parent Qt parent widget.
   */
  explicit PlotPanel(common::VisualizationManager* manager,
                     QWidget* parent = nullptr);

  /**
   * @brief Unsubscribes series and unregisters from @ref PlotViewSync.
   */
  ~PlotPanel() override;

  /**
   * @brief Installs interaction / settings / expand tools on the dock title bar.
   *
   * @param dock Host dock widget.
   */
  void installTitleBarTools(PanelDockWidget* dock);

  /**
   * @brief Returns the current panel configuration.
   *
   * @return Copy of @c config_ (series configs without runtime buffers).
   */
  PlotPanelConfig config() const;

  /**
   * @brief Replaces config, rebuilds runtimes, and resubscribes.
   *
   * @param config Full plot panel configuration.
   */
  void setConfig(const PlotPanelConfig& config);

  /**
   * @brief Copy panel settings for split/duplicate; runtime buffers start empty.
   *
   * @param config Source configuration.
   */
  void cloneConfigFrom(const PlotPanelConfig& config);

  /**
   * @brief Apply property edits from the embedded settings panel to this plot.
   *
   * @param config Edited config from @ref PlotSettingsWidget.
   */
  void applySettings(const PlotPanelConfig& config);

  /**
   * @brief Appends a new empty / default series and opens editing as needed.
   */
  void addSeries();

  /**
   * @brief Appends a series bound to @p channel and @p field_path.
   *
   * @param channel Source channel.
   * @param field_path Y field path (may include modifiers).
   */
  void addSeriesFromTopic(const QString& channel, const QString& field_path);

  /**
   * @brief Removes the series at @p index from config and runtime.
   *
   * @param index Series index.
   */
  void removeSeries(int index);

  /**
   * @brief Applies a synced X range from @ref PlotViewSync.
   *
   * @param min_x Synced minimum X.
   * @param max_x Synced maximum X.
   */
  void applySyncedXRange(double min_x, double max_x);

  /**
   * @brief Applies a synced left Y range from @ref PlotViewSync.
   *
   * @param min_y Synced minimum Y.
   * @param max_y Synced maximum Y.
   */
  void applySyncedYRange(double min_y, double max_y);

  /**
   * @brief Applies or clears a shared hover cursor from @ref PlotViewSync.
   *
   * @param active Whether @p x is valid.
   * @param x Cursor X when @p active is true.
   */
  void applySyncedCursorX(bool active, double x);

  /**
   * @brief Effective sync group name (empty when sync is off).
   *
   * @return Group name, or empty.
   */
  QString effectiveSyncGroup() const;

  /** @return Whether this panel accepts synced Y ranges. */
  bool syncYAxisEnabled() const { return config_.sync_y_axis; }

  /** @return Whether this panel accepts synced cursors. */
  bool syncCursorEnabled() const { return config_.sync_cursor; }

  /**
   * @brief Pauses or resumes sampling into the chart.
   *
   * @param paused @c true to freeze new samples.
   */
  void setPaused(bool paused);

  /** @return Whether sampling is paused. */
  bool isPaused() const { return paused_; }

  /** Clears sampled points without changing series config. */
  void clearPlotData();

  /** Imports a plot XML document into this panel. */
  void importPlotXml();

  /** Exports this panel config as plot XML. */
  void exportPlotXml();

  /**
   * @brief Shows or hides the settings pane.
   *
   * @param visible @c true to show settings.
   */
  void setSettingsVisible(bool visible);

  /**
   * @brief Whether settings are currently visible.
   *
   * @return Settings visibility.
   */
  bool settingsVisible() const;

  /**
   * @brief Syncs the title-bar Settings button checked state.
   *
   * @param checked Desired checked state.
   */
  void setSettingsButtonChecked(bool checked);

  /**
   * @brief Syncs the title-bar Expand button checked state.
   *
   * @param checked Desired checked state.
   */
  void setExpandButtonChecked(bool checked);

  /**
   * @brief Refreshes channel lists in the settings widget.
   */
  void refreshSettingsChannels();

  /**
   * @brief Exports current series samples to a CSV file (file dialog).
   */
  void exportPlotDataAsCsv();

  /**
   * @brief Exports the chart widget as a PNG image chosen by the user.
   */
  void exportPlotAsPng();

  /**
   * @brief Clears sampled points after global variables change.
   */
  void invalidateSeriesData();

  /**
   * @brief Scroll area hosting settings; reparented into the shared sidebar
   *        inspector.
   *
   * @return Settings host widget.
   * @see recallSettingsWidget()
   */
  QWidget* settingsWidgetForInspector();

  /**
   * @brief Reclaims settings from the shared inspector sidebar.
   */
  void recallSettingsWidget();

 signals:
  /** Emitted when config should be persisted. */
  void configChanged();

  /**
   * @brief Emitted when this plot becomes the active panel (focus / click).
   */
  void activated();

  /**
   * @brief Emitted when the title-bar Settings button is toggled (legacy hook).
   *
   * @param visible New settings visibility.
   */
  void settingsToggled(bool visible);

  /**
   * @brief Request splitting this panel.
   *
   * @param orientation Split orientation.
   */
  void panelSplitRequested(Qt::Orientation orientation);

  /** Request removing this panel. */
  void panelRemoveRequested();

  /** Request expanding this panel. */
  void panelExpandRequested();

  /**
   * @brief Request changing panel type by object name.
   *
   * @param object_name Target panel object name.
   */
  void panelChangeRequested(const QString& object_name);

 protected:
  /**
   * @brief Emits @ref activated() on focus.
   *
   * @param event Focus-in event.
   */
  void focusInEvent(QFocusEvent* event) override;

  /**
   * @brief Forwards wheel zoom to the chart when appropriate.
   *
   * @param event Wheel event.
   */
  void wheelEvent(QWheelEvent* event) override;

  /**
   * @brief Filters child events (e.g. wheel on settings / legend).
   *
   * @param watched Object being filtered.
   * @param event Event.
   * @return @c true if the event was handled.
   */
  bool eventFilter(QObject* watched, QEvent* event) override;

  /**
   * @brief Accepts plot-series drag mime.
   *
   * @param event Drag-enter event.
   */
  void dragEnterEvent(QDragEnterEvent* event) override;

  /**
   * @brief Continues accepting plot-series drags.
   *
   * @param event Drag-move event.
   */
  void dragMoveEvent(QDragMoveEvent* event) override;

  /**
   * @brief Adds dropped series payloads.
   *
   * @param event Drop event.
   */
  void dropEvent(QDropEvent* event) override;

 private slots:
  /**
   * @brief Title-bar Settings toggle.
   *
   * @param visible Requested visibility.
   */
  void onToggleSettings(bool visible);

  /**
   * @brief Tick timer: drain queues, trim points, update legend / chart.
   */
  void onTick();

 private:
  /** Chart “+” / settings add-series request. */
  void onAddSeriesRequested();

  /**
   * @brief Settings remove-series request.
   *
   * @param index Series index to remove.
   */
  void onRemoveSeriesRequested(int index);

  /** Push config into chart, legend, tools, and sync registration. */
  void applyConfigToUi();

  /** Sync Settings button checked state. */
  void syncSettingsToolState();

  /** Mirror config into the settings form. */
  void syncSettingsWidgetFromConfig();

  /**
   * @brief Show/hide the floating legend.
   *
   * @param visible Legend visibility.
   */
  void setLegendVisible(bool visible);

  /** Sync legend tool button state. */
  void syncLegendToolState();

  /** Reposition the legend over the chart anchor rect. */
  void updateLegendGeometry();

  /** Unsubscribe all and subscribe enabled series from config/runtime. */
  void resubscribeAll();

  /** Ensure @c runtime_series_ matches @c config_.series. */
  void mergeRuntimeWithConfig();

  /**
   * @brief Unsubscribe one series runtime.
   *
   * @param runtime Series to unsubscribe (queue cleared as needed).
   */
  void unsubscribeSeries(PlotSeriesRuntime& runtime);

  /**
   * @brief Subscribe the series at @p index.
   *
   * @param index Runtime / config series index.
   */
  void subscribeSeries(int index);

  /**
   * @brief Resolve message type for a channel.
   *
   * @param channel Channel name.
   * @return Message type string, or empty.
   */
  std::string messageTypeForChannel(const std::string& channel) const;

  /**
   * @brief Next auto color from the series color palette.
   *
   * @return Color for a newly added series.
   */
  QColor nextSeriesColor() const;

  /** Refresh legend value rows from hover / playhead. */
  void updateLegendValues();

  /**
   * @brief Handle a dropped channel+field as a new series.
   *
   * @param channel Source channel.
   * @param field_path Y field path.
   */
  void handleSeriesDrop(const QString& channel, const QString& field_path);

  /**
   * @brief Sets chart interaction mode and syncs tool buttons.
   *
   * @param mode Interaction mode.
   */
  void setChartInteractionMode(PlotInteractionMode mode);

  /** Sync select/pan/zoom tool button checked states. */
  void syncInteractionToolState();

  /** Resolve channel_pattern against live channels and resubscribe as needed. */
  void resolveChannelPatterns();

  /** Sync pause tool button state. */
  void syncPauseToolState();

  /** Non-owning visualization manager. */
  common::VisualizationManager* manager_ = nullptr;

  /** Persisted panel configuration. */
  PlotPanelConfig config_;

  /** Per-series runtime (queues, points, modifier state). */
  std::vector<std::unique_ptr<PlotSeriesRuntime>> runtime_series_;

  /** Chart canvas. */
  PlotChartWidget* chart_ = nullptr;

  /** Floating legend overlay. */
  PlotLegendWidget* legend_widget_ = nullptr;

  /** Settings form. */
  PlotSettingsWidget* settings_widget_ = nullptr;

  /** Scroll host for settings. */
  QScrollArea* settings_scroll_ = nullptr;

  /** Settings layout container. */
  QWidget* settings_container_ = nullptr;

  /** Periodic drain / redraw timer. */
  QTimer* tick_timer_ = nullptr;

  /** Dock Expand button. */
  QToolButton* expand_button_ = nullptr;

  /** Dock Settings button. */
  QToolButton* settings_button_ = nullptr;

  /** Select interaction tool. */
  QToolButton* select_tool_button_ = nullptr;

  /** Pan interaction tool. */
  QToolButton* pan_tool_button_ = nullptr;

  /** Zoom interaction tool. */
  QToolButton* zoom_tool_button_ = nullptr;

  /** Inspect interaction tool (A/B points). */
  QToolButton* inspect_tool_button_ = nullptr;

  /** Brush interaction tool (X-range stats). */
  QToolButton* brush_tool_button_ = nullptr;

  /** Legend visibility toggle. */
  QToolButton* legend_tool_button_ = nullptr;

  /** Pause / run toggle (checked = paused). */
  QToolButton* pause_tool_button_ = nullptr;

  /** Clear sampled points. */
  QToolButton* clear_tool_button_ = nullptr;

  /** Mutually exclusive interaction tool group. */
  QButtonGroup* interaction_tool_group_ = nullptr;

  /** When true, onTick drains queues without appending samples. */
  bool paused_ = false;

  /** Cursor into the auto color palette for new series. */
  int color_cursor_ = 0;

  /** Last sim time seen in onTick (dirty replot for playback tracker). */
  double last_tick_sim_time_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace plot
}  // namespace autoviz
