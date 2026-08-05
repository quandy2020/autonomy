/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <optional>

#include <QPointer>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>

#include "autoviz/ui/teleop/teleop_types.hpp"

class QScrollArea;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace teleop {
class TeleopControlWidget;
class TeleopSettingsWidget;
}

namespace teleop {

class TeleopPanel : public QWidget {
  Q_OBJECT

 public:
  explicit TeleopPanel(common::VisualizationManager* manager,
                       QWidget* parent = nullptr);
  ~TeleopPanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  TeleopPanelConfig config() const;
  void setConfig(const TeleopPanelConfig& config);
  void cloneConfigFrom(const TeleopPanelConfig& config);
  void applySettings(const TeleopPanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();

 signals:
  void configChanged();
  void activated();
  void settingsToggled(bool visible);
  void panelSplitRequested(Qt::Orientation orientation);
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;

 private slots:
  void onToggleSettings(bool visible);
  void onPublishTick();
  void onLinearChanged(double x, double y);
  void onAngularChanged(double turn);
  void onLinearReleased();
  void onAngularReleased();
  void onStopClicked();

 private:
  void publishTwist(const automsgs::msgs::geometry_msgs::Twist& twist);
  void publishStop();
  void setActiveTwist(const automsgs::msgs::geometry_msgs::Twist& twist);
  void updatePublishTimer();
  automsgs::msgs::geometry_msgs::Twist composeTwist() const;
  void publishComposedTwist();
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();

  common::VisualizationManager* manager_ = nullptr;
  TeleopPanelConfig config_;
  TeleopControlWidget* control_ = nullptr;
  TeleopSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QTimer* publish_timer_ = nullptr;
  std::optional<automsgs::msgs::geometry_msgs::Twist> active_twist_;
  double linear_x_ = 0.0;
  double linear_y_ = 0.0;
  double angular_turn_ = 0.0;
  bool linear_active_ = false;
  bool angular_active_ = false;
};

}  // namespace teleop
}  // namespace autoviz
