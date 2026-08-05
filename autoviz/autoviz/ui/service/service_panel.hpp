/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QPointer>

#include "autoviz/ui/service/service_types.hpp"

class QScrollArea;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace service_panel {

class ServiceEditorWidget;
class ServiceSettingsWidget;

class ServicePanel : public QWidget {
  Q_OBJECT

 public:
  explicit ServicePanel(common::VisualizationManager* manager,
                        QWidget* parent = nullptr);

  void installTitleBarTools(PanelDockWidget* dock);

  ServiceCallPanelConfig config() const;
  void setConfig(const ServiceCallPanelConfig& config);
  void cloneConfigFrom(const ServiceCallPanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshServices();

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

 private slots:
  void onToggleSettings(bool visible);
  void onEditorConfigChanged();
  void onSettingsConfigChanged();

 private:
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();
  void applyConfigToUi();

  common::VisualizationManager* manager_ = nullptr;
  ServiceCallPanelConfig config_;
  ServiceEditorWidget* editor_ = nullptr;
  ServiceSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
};

}  // namespace service_panel
}  // namespace autoviz
