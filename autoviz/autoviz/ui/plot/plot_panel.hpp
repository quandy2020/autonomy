/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <memory>
#include <vector>

#include "autoviz/ui/plot/plot_types.hpp"

class QAction;
class QActionGroup;
class QDragEnterEvent;
class QDropEvent;
class QFocusEvent;
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

class PlotPanel : public QWidget {
  Q_OBJECT

 public:
  explicit PlotPanel(common::VisualizationManager* manager,
                     QWidget* parent = nullptr);
  ~PlotPanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  PlotPanelConfig config() const;
  void setConfig(const PlotPanelConfig& config);
  /** Copy panel settings for split/duplicate; runtime buffers start empty. */
  void cloneConfigFrom(const PlotPanelConfig& config);
  /** Apply property edits from the embedded settings panel to this plot. */
  void applySettings(const PlotPanelConfig& config);
  void addSeries();
  void addSeriesFromTopic(const QString& channel, const QString& field_path);
  void removeSeries(int index);
  void applySyncedXRange(double min_x, double max_x);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  void refreshSettingsChannels();
  void exportPlotDataAsCsv();
  /** Clears sampled points after global variables change. */
  void invalidateSeriesData();
  /** Scroll area hosting settings; reparented into the shared sidebar inspector. */
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();

 signals:
  void configChanged();
  /** Emitted when this plot becomes the active panel (focus / click). */
  void activated();
  /** Emitted when the title-bar Settings button is toggled (legacy hook). */
  void settingsToggled(bool visible);
  void panelSplitRequested(Qt::Orientation orientation);
  void panelRemoveRequested();
  void panelExpandRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void onToggleSettings(bool visible);
  void onToggleLegend(bool visible);
  void onInteractionModeChanged(QAction* action);
  void onTick();

 private:
  void onAddSeriesRequested();
  void onRemoveSeriesRequested(int index);
  void applyConfigToUi();
  void syncSettingsToolState();
  void syncSettingsWidgetFromConfig();
  void setLegendVisible(bool visible);
  void syncLegendToolState();
  void updateLegendGeometry();
  void resubscribeAll();
  void mergeRuntimeWithConfig();
  void unsubscribeSeries(PlotSeriesRuntime& runtime);
  void subscribeSeries(int index);
  std::string messageTypeForChannel(const std::string& channel) const;
  QColor nextSeriesColor() const;
  void trimSeriesPoints(PlotSeriesRuntime& runtime, double latest_x);
  void updateLegendValues();
  void handleSeriesDrop(const QString& channel, const QString& field_path);
  double resolveTimestampSec(const PlotSeriesRuntime& runtime,
                             const std::string& message_type,
                             const std::string& payload, double receive_time,
                             double sim_time) const;

  common::VisualizationManager* manager_ = nullptr;
  PlotPanelConfig config_;
  std::vector<std::unique_ptr<PlotSeriesRuntime>> runtime_series_;
  PlotChartWidget* chart_ = nullptr;
  PlotLegendWidget* legend_widget_ = nullptr;
  PlotSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QTimer* tick_timer_ = nullptr;
  QToolButton* legend_button_ = nullptr;
  QToolButton* expand_button_ = nullptr;
  QToolButton* settings_button_ = nullptr;
  QActionGroup* interaction_group_ = nullptr;
  int color_cursor_ = 0;
};

}  // namespace plot
}  // namespace autoviz
