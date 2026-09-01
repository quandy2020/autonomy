/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QElapsedTimer>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <glog/logging.h>

#include <unordered_map>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/time_utils.hpp>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/teleop_channels.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/teleop/teleop_control_widget.hpp"
#include "autoviz/ui/teleop/teleop_settings_widget.hpp"
#include "autoviz/ui/teleop/teleop_twist_utils.hpp"

namespace autoviz {
namespace teleop {
namespace {

constexpr int kGoalWriterResetCooldownMs = 5000;

namespace tp = ::autonomy::task::proto;

QString TeleopStatusLabel(tp::TeleopStatus status) {
  switch (status) {
    case tp::TELEOP_STATUS_ACTIVE:
      return QObject::tr("运行中");
    case tp::TELEOP_STATUS_TIMEOUT:
      return QObject::tr("超时");
    case tp::TELEOP_STATUS_REJECTED:
      return QObject::tr("被拒绝");
    case tp::TELEOP_STATUS_IDLE:
      return QObject::tr("空闲");
    default:
      return QObject::tr("未知");
  }
}

float TeleopWatchdogSec(double publish_rate_hz) {
  const double hz = std::max(publish_rate_hz, 0.1);
  return static_cast<float>(std::max(1.0, 2.5 / hz));
}

tp::TeleopGoal MakeTeleopGoal(tp::TeleopCommand command,
                             const automsgs::msgs::geometry_msgs::Twist& twist,
                             float max_linear, float max_angular,
                             float watchdog_sec) {
  tp::TeleopGoal goal;
  goal.set_command(command);
  goal.set_max_linear_speed(max_linear);
  goal.set_max_angular_speed(max_angular);
  goal.set_watchdog_timeout_sec(watchdog_sec);
  goal.set_disable_collision_checks(false);
  if (command == tp::TELEOP_CMD_VELOCITY || command == tp::TELEOP_CMD_START) {
    auto* velocity = goal.mutable_velocity();
    *velocity->mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    velocity->mutable_header()->set_frame_id("base_link");
    *velocity->mutable_twist() = twist;
  }
  return goal;
}

}  // namespace

TeleopPanel::TeleopPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultTeleopPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  ApplyPanelShell(this);
  setObjectName(QStringLiteral("TeleopPanelContent"));
  setStyleSheet(QStringLiteral(
      "TeleopPanel, QWidget#TeleopPanelContent {"
      "  background: #f8f9fb; color: #1e293b;"
      "}"));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  settings_container_ = new QWidget(this);
  settings_container_->hide();
  auto* settings_layout = new QVBoxLayout(settings_container_);
  settings_layout->setContentsMargins(0, 0, 0, 0);
  settings_scroll_ = new QScrollArea(settings_container_);
  settings_scroll_->setWidgetResizable(true);
  settings_scroll_->setFrameShape(QFrame::NoFrame);
  settings_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  settings_widget_ = new TeleopSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  control_ = new TeleopControlWidget(this);
  root->addWidget(control_, 1);

  publish_timer_ = new QTimer(this);
  publish_timer_->setTimerType(Qt::PreciseTimer);
  connect(publish_timer_, &QTimer::timeout, this, &TeleopPanel::onPublishTick);
  connect(settings_widget_, &TeleopSettingsWidget::configChanged, this, [this]() {
    applySettings(settings_widget_->config());
  });
  connect(control_, &TeleopControlWidget::linearChanged, this,
          &TeleopPanel::onLinearChanged);
  connect(control_, &TeleopControlWidget::angularChanged, this,
          &TeleopPanel::onAngularChanged);
  connect(control_, &TeleopControlWidget::linearReleased, this,
          &TeleopPanel::onLinearReleased);
  connect(control_, &TeleopControlWidget::angularReleased, this,
          &TeleopPanel::onAngularReleased);
  connect(control_, &TeleopControlWidget::stopClicked, this,
          &TeleopPanel::onStopClicked);
  connect(control_, &TeleopControlWidget::stickModeChanged, this,
          [this](TeleopStickMode mode) {
            config_.stick_mode = mode;
            syncSettingsWidgetFromConfig();
            emit configChanged();
          });
  connect(control_, &TeleopControlWidget::maxSpeedsChanged, this,
          [this](double max_linear, double max_angular) {
            config_.max_linear_speed = max_linear;
            config_.max_angular_speed = max_angular;
            config_.up.value = max_linear;
            config_.down.value = -max_linear;
            config_.left.value = max_angular;
            config_.right.value = -max_angular;
            syncSettingsWidgetFromConfig();
            emit configChanged();
          });
  connect(control_, &TeleopControlWidget::smartTeleopChanged, this,
          [this](bool enabled) {
            if (config_.smart_teleop_enabled && !enabled) {
              publishTeleopSessionStop();
            }
            config_.smart_teleop_enabled = enabled;
            if (enabled) {
              smart_teleop_session_active_ = false;
              teleop_start_logged_this_connect_ = false;
              ensureTeleopGoalWriter();
              ensureTeleopFeedbackReader();
              publishTeleopSessionStart();
            } else {
              shutdownTeleopReaders();
            }
            updateSmartTeleopUiStatus();
            syncSettingsWidgetFromConfig();
            emit configChanged();
          });

  updatePublishTimer();
  if (control_ != nullptr) {
    control_->setStickMode(config_.stick_mode);
    control_->setMaxSpeeds(config_.max_linear_speed, config_.max_angular_speed);
    control_->setSmartTeleopEnabled(config_.smart_teleop_enabled);
    control_->setSmartTeleopAvailable(true);
    if (config_.smart_teleop_enabled) {
      ensureTeleopGoalWriter();
      ensureTeleopFeedbackReader();
      publishTeleopSessionStart();
    }
    updateSmartTeleopUiStatus();
  }
  syncSettingsWidgetFromConfig();
}

TeleopPanel::~TeleopPanel() {
  publishStop();
  publish_timer_->stop();
  shutdownTeleopReaders();
}

void TeleopPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("TeleopDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.show_settings = true;
  options.settings_checked = config_.settings_visible;
  options.on_settings_toggled = [this](bool visible) { onToggleSettings(visible); };
  options.show_expand = false;

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  settings_button_ = tools.settings_button;
  dock->setTitleBarTools(tools.widget);
}

TeleopPanelConfig TeleopPanel::config() const { return config_; }

void TeleopPanel::setConfig(const TeleopPanelConfig& config) {
  if (config_.smart_teleop_enabled && !config.smart_teleop_enabled) {
    publishTeleopSessionStop();
  }
  config_ = config;
  updatePublishTimer();
  if (control_ != nullptr) {
    control_->setStickMode(config_.stick_mode);
    control_->setMaxSpeeds(config_.max_linear_speed, config_.max_angular_speed);
    control_->setSmartTeleopEnabled(config_.smart_teleop_enabled);
    if (config_.smart_teleop_enabled) {
      ensureTeleopGoalWriter();
      ensureTeleopFeedbackReader();
      publishTeleopSessionStart();
    } else {
      shutdownTeleopReaders();
    }
    updateSmartTeleopUiStatus();
  }
  syncSettingsWidgetFromConfig();
  syncSettingsToolState();
}

void TeleopPanel::cloneConfigFrom(const TeleopPanelConfig& config) {
  publishStop();
  config_ = config;
  if (control_ != nullptr) {
    control_->setStickMode(config_.stick_mode);
    control_->setMaxSpeeds(config_.max_linear_speed, config_.max_angular_speed);
  }
  syncSettingsWidgetFromConfig();
  syncSettingsToolState();
}

void TeleopPanel::applySettings(const TeleopPanelConfig& config) {
  setConfig(config);
  emit configChanged();
}

void TeleopPanel::setSettingsVisible(bool visible) {
  config_.settings_visible = visible;
  syncSettingsToolState();
}

bool TeleopPanel::settingsVisible() const { return config_.settings_visible; }

void TeleopPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ == nullptr) {
    return;
  }
  settings_button_->blockSignals(true);
  settings_button_->setChecked(checked);
  settings_button_->blockSignals(false);
}

QWidget* TeleopPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void TeleopPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void TeleopPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ == nullptr) {
    return;
  }
  settings_widget_->blockSignals(true);
  settings_widget_->setConfig(config_);
  settings_widget_->blockSignals(false);
}

void TeleopPanel::syncSettingsToolState() {
  setSettingsButtonChecked(config_.settings_visible);
}

void TeleopPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void TeleopPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void TeleopPanel::updatePublishTimer() {
  const int interval_ms = std::max(
      10, static_cast<int>(1000.0 / std::max(config_.publish_rate_hz, 0.1)));
  publish_timer_->setInterval(interval_ms);
}

void TeleopPanel::setActiveTwist(
    const automsgs::msgs::geometry_msgs::Twist& twist) {
  active_twist_ = twist;
}

void TeleopPanel::publishTwist(const automsgs::msgs::geometry_msgs::Twist& twist) {
  if (config_.smart_teleop_enabled) {
    publishTeleopVelocity(twist);
    return;
  }
  if (config_.topic.isEmpty() || manager_ == nullptr) {
    return;
  }
  auto node = manager_->autolinkNode();
  if (node == nullptr) {
    return;
  }
  // Must advertise TwistStamped — same type as controller_server / autosim_bridge.
  // Skip creating a writer for all-zero stop unless we already published (avoids
  // declaring /cmd_vel at panel open and fighting autonomy.control).
  const bool is_zero =
      std::abs(twist.linear().x()) < 1e-9 && std::abs(twist.linear().y()) < 1e-9 &&
      std::abs(twist.linear().z()) < 1e-9 && std::abs(twist.angular().x()) < 1e-9 &&
      std::abs(twist.angular().y()) < 1e-9 && std::abs(twist.angular().z()) < 1e-9;

  static thread_local std::unordered_map<
      std::string,
      std::shared_ptr<
          ::autolink::Writer<automsgs::msgs::geometry_msgs::TwistStamped>>>
      writers;
  const std::string channel = config_.topic.toStdString();
  auto& writer = writers[channel];
  if (writer == nullptr) {
    if (is_zero) {
      return;
    }
    writer =
        node->CreateWriter<automsgs::msgs::geometry_msgs::TwistStamped>(channel);
  }
  if (writer == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TwistStamped stamped;
  *stamped.mutable_header()->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeNow();
  stamped.mutable_header()->set_frame_id("base_link");
  *stamped.mutable_twist() = twist;
  writer->Write(stamped);
}

void TeleopPanel::publishTeleopSessionStart() {
  if (!config_.smart_teleop_enabled || manager_ == nullptr) {
    return;
  }
  ensureTeleopGoalWriter();
  if (!teleop_goal_writer_) {
    return;
  }
  const float max_linear =
      static_cast<float>(std::max(0.01, config_.max_linear_speed));
  const float max_angular =
      static_cast<float>(std::max(0.01, config_.max_angular_speed));
  const float watchdog = TeleopWatchdogSec(config_.publish_rate_hz);
  if (!teleop_goal_writer_->Write(MakeTeleopGoal(
          tp::TELEOP_CMD_START, ZeroTwist(), max_linear, max_angular, watchdog))) {
    ++goal_write_fail_streak_;
    if (goal_write_fail_streak_ == 1) {
      LOG(WARNING) << "TeleopPanel: failed to write TELEOP_CMD_START "
                      "(waiting for autonomy.task)";
    }
    smart_teleop_status_text_ = tr("等待 task 连接…");
    QMetaObject::invokeMethod(
        this, [this]() { updateSmartTeleopUiStatus(); }, Qt::QueuedConnection);
  } else {
    goal_write_fail_streak_ = 0;
    smart_teleop_session_active_ = true;
    if (!teleop_start_logged_this_connect_) {
      teleop_start_logged_this_connect_ = true;
      LOG(INFO) << "TeleopPanel: wrote TELEOP_CMD_START";
    }
  }
}

void TeleopPanel::publishTeleopVelocity(
    const automsgs::msgs::geometry_msgs::Twist& twist) {
  if (!config_.smart_teleop_enabled || manager_ == nullptr) {
    return;
  }
  ensureTeleopGoalWriter();
  if (!teleop_goal_writer_) {
    return;
  }

  const float max_linear =
      static_cast<float>(std::max(0.01, config_.max_linear_speed));
  const float max_angular =
      static_cast<float>(std::max(0.01, config_.max_angular_speed));
  const float watchdog = TeleopWatchdogSec(config_.publish_rate_hz);

  const bool twist_active =
      std::abs(twist.linear().x()) >= 1e-4 ||
      std::abs(twist.angular().z()) >= 1e-4;
  if (!smart_teleop_session_active_ && !twist_active) {
    return;
  }
  if (!teleop_goal_writer_->Write(MakeTeleopGoal(tp::TELEOP_CMD_VELOCITY, twist, max_linear,
                               max_angular, watchdog))) {
    ++goal_write_fail_streak_;
  } else {
    goal_write_fail_streak_ = 0;
  }
}

void TeleopPanel::startSessionConnectTimer() {
  if (session_connect_timer_ == nullptr) {
    session_connect_timer_ = new QTimer(this);
    session_connect_timer_->setInterval(250);
    connect(session_connect_timer_, &QTimer::timeout, this, [this]() {
      if (!config_.smart_teleop_enabled) {
        stopSessionConnectTimer();
        return;
      }
      if (!teleop_goal_writer_) {
        ensureTeleopGoalWriter();
      }
      if (!teleop_goal_writer_) {
        return;
      }
      if (!smart_teleop_session_active_) {
        maybeResetTeleopGoalWriter();
        publishTeleopSessionStart();
      }
      if (linear_active_ || angular_active_) {
        publishTeleopVelocity(composeTwist());
      } else if (smart_teleop_session_active_) {
        stopSessionConnectTimer();
      }
    });
  }
  if (!session_connect_timer_->isActive()) {
    teleop_start_logged_this_connect_ = false;
    session_connect_timer_->start();
  }
}

void TeleopPanel::stopSessionConnectTimer() {
  if (session_connect_timer_ != nullptr) {
    session_connect_timer_->stop();
  }
}

void TeleopPanel::maybeResetTeleopGoalWriter() {
  if (!config_.smart_teleop_enabled || goal_write_fail_streak_ <= 0) {
    return;
  }
  if (goal_writer_reset_timer_ == nullptr) {
    goal_writer_reset_timer_ = new QElapsedTimer();
    goal_writer_reset_timer_->start();
    return;
  }
  if (goal_writer_reset_timer_->elapsed() < kGoalWriterResetCooldownMs) {
    return;
  }
  resetTeleopGoalWriter();
}

void TeleopPanel::resetTeleopGoalWriter() {
  if (!config_.smart_teleop_enabled) {
    return;
  }
  LOG(INFO) << "TeleopPanel: resetting TeleopGoal writer (SHM reconnect)";
  teleop_goal_writer_.reset();
  smart_teleop_session_active_ = false;
  goal_write_fail_streak_ = 0;
  if (goal_writer_reset_timer_ != nullptr) {
    goal_writer_reset_timer_->restart();
  }
  ensureTeleopGoalWriter();
}

void TeleopPanel::ensureTeleopGoalWriter() {
  if (teleop_goal_writer_ != nullptr || manager_ == nullptr) {
    return;
  }
  auto node = manager_->autolinkNode();
  if (node == nullptr) {
    LOG(ERROR) << "TeleopPanel: autolink node unavailable for smart teleop";
    return;
  }
  teleop_goal_writer_ =
      node->CreateWriter<::autonomy::task::proto::TeleopGoal>(
          integration::kTeleopGoalChannel);
  if (!teleop_goal_writer_) {
    LOG(ERROR) << "TeleopPanel: failed to create writer on "
               << integration::kTeleopGoalChannel;
  } else {
    LOG(INFO) << "TeleopPanel: publishing TeleopGoal on "
              << integration::kTeleopGoalChannel;
    startSessionConnectTimer();
  }
  updateSmartTeleopUiStatus();
}

void TeleopPanel::ensureTeleopFeedbackReader() {
  if (teleop_feedback_reader_ != nullptr || manager_ == nullptr) {
    return;
  }
  auto node = manager_->autolinkNode();
  if (node == nullptr) {
    return;
  }
  node->DeleteReader(integration::kTeleopFeedbackChannel);
  teleop_feedback_reader_ =
      node->CreateReader<::autonomy::task::proto::TeleopFeedback>(
          integration::kTeleopFeedbackChannel,
          [this](const std::shared_ptr<tp::TeleopFeedback>& feedback) {
            if (!feedback) {
              return;
            }
            if (feedback->status() == tp::TELEOP_STATUS_ACTIVE) {
              smart_teleop_session_active_ = true;
              stopSessionConnectTimer();
            } else if (feedback->status() == tp::TELEOP_STATUS_REJECTED ||
                       feedback->status() == tp::TELEOP_STATUS_IDLE) {
              smart_teleop_session_active_ = false;
            }
            smart_teleop_status_text_ =
                TeleopStatusLabel(feedback->status());
            QMetaObject::invokeMethod(
                this, [this]() { updateSmartTeleopUiStatus(); },
                Qt::QueuedConnection);
          });
  if (!teleop_feedback_reader_) {
    LOG(WARNING) << "TeleopPanel: failed to subscribe "
                 << integration::kTeleopFeedbackChannel;
  }
}

void TeleopPanel::shutdownTeleopReaders() {
  stopSessionConnectTimer();
  auto node = manager_ != nullptr ? manager_->autolinkNode() : nullptr;
  teleop_goal_writer_.reset();
  if (teleop_feedback_reader_) {
    teleop_feedback_reader_.reset();
    if (node != nullptr) {
      node->DeleteReader(integration::kTeleopFeedbackChannel);
    }
  }
  smart_teleop_status_text_.clear();
  smart_teleop_session_active_ = false;
  goal_write_fail_streak_ = 0;
  updateSmartTeleopUiStatus();
}

void TeleopPanel::updateSmartTeleopUiStatus() {
  if (control_ == nullptr) {
    return;
  }
  if (!config_.smart_teleop_enabled) {
    control_->setSmartTeleopStatusText(QString());
    return;
  }
  QString status;
  if (!teleop_goal_writer_) {
    status = tr("通道未就绪");
  } else if (goal_write_fail_streak_ > 0 && !smart_teleop_session_active_) {
    status = tr("等待 task 连接…");
  } else if (!smart_teleop_status_text_.isEmpty()) {
    status = smart_teleop_status_text_;
  } else {
    status = tr("已连接，拖动摇杆发送");
  }
  control_->setSmartTeleopStatusText(status);
}

void TeleopPanel::publishTeleopSessionStop() {
  if (!smart_teleop_session_active_ || teleop_goal_writer_ == nullptr) {
    smart_teleop_session_active_ = false;
    return;
  }

  const float watchdog = TeleopWatchdogSec(config_.publish_rate_hz);
  teleop_goal_writer_->Write(MakeTeleopGoal(tp::TELEOP_CMD_STOP, ZeroTwist(), 0.f, 0.f,
                              watchdog));
  smart_teleop_session_active_ = false;
}

void TeleopPanel::publishStop() {
  linear_x_ = 0.0;
  linear_y_ = 0.0;
  angular_turn_ = 0.0;
  linear_active_ = false;
  angular_active_ = false;
  if (control_ != nullptr) {
    control_->resetJoysticks();
  }
  setActiveTwist(ZeroTwist());
  if (config_.smart_teleop_enabled) {
    publishTeleopSessionStop();
  } else {
    publishTwist(ZeroTwist());
  }
  publish_timer_->stop();
}

automsgs::msgs::geometry_msgs::Twist TeleopPanel::composeTwist() const {
  automsgs::msgs::geometry_msgs::Twist twist = ZeroTwist();

  const double max_linear = std::max(0.01, config_.max_linear_speed);
  const double max_angular = std::max(0.01, config_.max_angular_speed);

  // Stick Y is screen-down positive; negate so stick-up = forward (+linear.x).
  const double forward = -linear_y_;
  twist.mutable_linear()->set_x(forward * max_linear);
  // Dual Move stick X = strafe; Arcade keeps this at 0.
  twist.mutable_linear()->set_y(linear_x_ * max_linear);
  twist.mutable_angular()->set_z(angular_turn_ * max_angular);

  return twist;
}

void TeleopPanel::publishComposedTwist() {
  const automsgs::msgs::geometry_msgs::Twist twist = composeTwist();
  setActiveTwist(twist);
  publishTwist(twist);
  if (!publish_timer_->isActive()) {
    publish_timer_->start();
  }
}

void TeleopPanel::onLinearChanged(double x, double y) {
  linear_x_ = x;
  linear_y_ = y;
  linear_active_ = std::hypot(x, y) > 1e-3;
  publishComposedTwist();
}

void TeleopPanel::onAngularChanged(double turn) {
  angular_turn_ = turn;
  angular_active_ = std::abs(turn) > 1e-3;
  publishComposedTwist();
}

void TeleopPanel::onLinearReleased() {
  linear_x_ = 0.0;
  linear_y_ = 0.0;
  linear_active_ = false;
  if (config_.stop_on_release && !angular_active_) {
    if (config_.smart_teleop_enabled) {
      setActiveTwist(ZeroTwist());
      publishTeleopVelocity(ZeroTwist());
      publish_timer_->stop();
      return;
    }
    publishStop();
    return;
  }
  publishComposedTwist();
}

void TeleopPanel::onAngularReleased() {
  angular_turn_ = 0.0;
  angular_active_ = false;
  if (config_.stop_on_release && !linear_active_) {
    if (config_.smart_teleop_enabled) {
      setActiveTwist(ZeroTwist());
      publishTeleopVelocity(ZeroTwist());
      publish_timer_->stop();
      return;
    }
    publishStop();
    return;
  }
  publishComposedTwist();
}

void TeleopPanel::onStopClicked() {
  publishStop();
}

void TeleopPanel::onPublishTick() {
  if (!linear_active_ && !angular_active_) {
    publish_timer_->stop();
    return;
  }
  publishComposedTwist();
}

}  // namespace teleop
}  // namespace autoviz
