/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <optional>

#include <QPointer>
#include <QWidget>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>

#include <autolink/node/reader.hpp>
#include <autolink/node/writer.hpp>
#include <automsgs/task/teleop.pb.h>
#include "autoviz/ui/teleop/teleop_types.hpp"

class QElapsedTimer;
class QFocusEvent;
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
  void publishTeleopVelocity(const automsgs::msgs::geometry_msgs::Twist& twist);
  void publishTeleopSessionStart();
  void publishTeleopSessionStop();
  void publishStop();
  void setActiveTwist(const automsgs::msgs::geometry_msgs::Twist& twist);
  void updatePublishTimer();
  automsgs::msgs::geometry_msgs::Twist composeTwist() const;
  void publishComposedTwist();
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();
  void ensureTeleopGoalWriter();
  void resetTeleopGoalWriter();
  void ensureTeleopFeedbackReader();
  void shutdownTeleopReaders();
  void updateSmartTeleopUiStatus();
  void startSessionConnectTimer();
  void stopSessionConnectTimer();
  void maybeResetTeleopGoalWriter();

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
  bool smart_teleop_session_active_ = false;
  bool teleop_start_logged_this_connect_ = false;
  QString smart_teleop_status_text_;
  std::shared_ptr<::autolink::Writer<::autonomy::task::proto::TeleopGoal>>
      teleop_goal_writer_;
  std::shared_ptr<::autolink::Reader<::autonomy::task::proto::TeleopFeedback>>
      teleop_feedback_reader_;
  QTimer* session_connect_timer_ = nullptr;
  QElapsedTimer* goal_writer_reset_timer_ = nullptr;
  int goal_write_fail_streak_ = 0;
};

}  // namespace teleop
}  // namespace autoviz
