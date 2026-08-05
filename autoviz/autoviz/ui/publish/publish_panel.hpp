/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QPointer>

#include "autoviz/ui/publish/publish_types.hpp"

class QScrollArea;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace publish_panel {

class PublishEditorWidget;
class PublishSettingsWidget;

class PublishPanel : public QWidget {
  Q_OBJECT

 public:
  explicit PublishPanel(common::VisualizationManager* manager, QWidget* parent = nullptr);

  void installTitleBarTools(PanelDockWidget* dock);

  PublishPanelConfig config() const;
  void setConfig(const PublishPanelConfig& config);
  void cloneConfigFrom(const PublishPanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshSettingsChannels();

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

 private:
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();
  void applyConfigToUi();

  common::VisualizationManager* manager_ = nullptr;
  PublishPanelConfig config_;
  PublishEditorWidget* editor_ = nullptr;
  PublishSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
};

}  // namespace publish_panel
}  // namespace autoviz
