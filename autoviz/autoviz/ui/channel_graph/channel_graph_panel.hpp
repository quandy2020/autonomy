/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QCheckBox;
class QComboBox;
class QLineEdit;
class QLabel;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace channel_graph {

class ChannelGraphView;

struct ChannelGraphPanelConfig {
  bool show_services = true;
  bool show_channels = true;
  bool auto_refresh = true;
  QString filter;
  QString prefix_filter;
};

ChannelGraphPanelConfig DefaultChannelGraphPanelConfig();

class ChannelGraphPanel : public QWidget {
  Q_OBJECT

 public:
  explicit ChannelGraphPanel(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

  void installTitleBarTools(PanelDockWidget* dock);
  void setExpandButtonChecked(bool checked);

  ChannelGraphPanelConfig config() const;
  void setConfig(const ChannelGraphPanelConfig& config);
  void cloneConfigFrom(const ChannelGraphPanelConfig& config);

 public slots:
  void refreshGraph();

 signals:
  void activated();
  void configChanged();
  void panelSplitRequested(Qt::Orientation orientation);
  void panelExpandRequested();
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private slots:
  void onFilterChanged(const QString& text);
  void onPrefixChanged(int index);
  void onShowServicesToggled(bool enabled);
  void onShowChannelsToggled(bool enabled);
  void onAutoRefreshToggled(bool enabled);
  void onRefreshClicked();
  void onZoomFitClicked();
  void onGraphRendered(int vertex_count, int edge_count);

 private:
  void setupUi();
  void applyConfigToUi();
  void rebuildPrefixCombo();
  void updateStatusText(const QString& text);

  common::VisualizationManager* manager_ = nullptr;
  ChannelGraphPanelConfig config_;
  ChannelGraphView* graph_view_ = nullptr;
  QCheckBox* show_services_check_ = nullptr;
  QCheckBox* show_channels_check_ = nullptr;
  QCheckBox* auto_refresh_check_ = nullptr;
  QComboBox* prefix_combo_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QLabel* status_label_ = nullptr;
  QToolButton* expand_button_ = nullptr;
  QTimer* refresh_timer_ = nullptr;
  QTimer* filter_timer_ = nullptr;
  QString last_topology_hash_;
};

}  // namespace channel_graph
}  // namespace autoviz
