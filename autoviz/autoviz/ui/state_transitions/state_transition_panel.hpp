/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <memory>
#include <vector>

#include "autoviz/ui/state_transitions/state_transition_types.hpp"

class QDragEnterEvent;
class QDropEvent;
class QFocusEvent;
class QFrame;
class QScrollArea;
class QTimer;
class QToolButton;
class QLabel;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace indicator {
struct IndicatorFieldValue;
}

namespace state_transitions {

class StateTransitionSettingsWidget;
class StateTransitionViewWidget;

class StateTransitionPanel : public QWidget {
  Q_OBJECT

 public:
  explicit StateTransitionPanel(common::VisualizationManager* manager,
                                QWidget* parent = nullptr);
  ~StateTransitionPanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  StateTransitionPanelConfig config() const;
  void setConfig(const StateTransitionPanelConfig& config);
  void cloneConfigFrom(const StateTransitionPanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshSettingsChannels();
  void refreshFromVariables();
  void invalidateSeriesData();

  void handleSeriesDrop(const QString& channel, const QString& field_path);

 signals:
  void configChanged();
  void activated();
  void settingsToggled(bool visible);
  void panelSplitRequested(Qt::Orientation orientation);
  void panelExpandRequested();
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void onToggleSettings(bool visible);
  void onTick();

 private:
  void onAddSeriesRequested();
  void onRemoveSeriesRequested(int index);
  void applyConfigToUi();
  void syncSettingsWidgetFromConfig();
  void resubscribeAll();
  void unsubscribeSeries(StateTransitionSeriesRuntime& runtime);
  void subscribeSeries(int index);
  std::string messageTypeForChannel(const std::string& channel) const;
  double resolveTimestampSec(const StateTransitionSeriesRuntime& runtime,
                             const std::string& message_type,
                             const std::string& payload, double receive_time,
                             double sim_time) const;
  void ingestPayload(int series_index, const std::string& payload);
  void trimSeriesSegments(StateTransitionSeriesRuntime& runtime, double latest_time);
  void maybeInferMappingRule(int series_index,
                             const indicator::IndicatorFieldValue& value);
  void updateStatusBar();

  common::VisualizationManager* manager_ = nullptr;
  StateTransitionPanelConfig config_;
  std::vector<std::unique_ptr<StateTransitionSeriesRuntime>> runtime_series_;
  StateTransitionViewWidget* view_ = nullptr;
  StateTransitionSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QFrame* footer_ = nullptr;
  QLabel* status_label_ = nullptr;
  QTimer* tick_timer_ = nullptr;
  QToolButton* settings_button_ = nullptr;
  QToolButton* expand_button_ = nullptr;
  QToolButton* box_zoom_button_ = nullptr;
};

}  // namespace state_transitions
}  // namespace autoviz
