/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QPointer>

#include "autoviz/ui/grpc/grpc_types.hpp"

class QScrollArea;
class QToolButton;

namespace autoviz {

class PanelDockWidget;

namespace grpc_panel {

class GrpcEditorWidget;
class GrpcSettingsWidget;

class GrpcPanel : public QWidget {
  Q_OBJECT

 public:
  explicit GrpcPanel(QWidget* parent = nullptr);

  void installTitleBarTools(PanelDockWidget* dock);

  GrpcPanelPersistConfig config() const;
  void setConfig(const GrpcPanelPersistConfig& config);
  void cloneConfigFrom(const GrpcPanelPersistConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();

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

  GrpcPanelPersistConfig config_;
  GrpcEditorWidget* editor_ = nullptr;
  GrpcSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
};

}  // namespace grpc_panel
}  // namespace autoviz
