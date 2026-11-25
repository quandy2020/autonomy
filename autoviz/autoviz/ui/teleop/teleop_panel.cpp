/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_panel.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QElapsedTimer>
#include <QFocusEvent>
#include <QFrame>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMetaObject>
#include <QRect>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <glog/logging.h>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/time_utils.hpp>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/common/interaction/channel_payload.hpp"
#include "autoviz/common/interaction/channel_writer_registry.hpp"
#include "autoviz/common/interaction/teleop_channels.hpp"
#include "autoviz/ui/theme/tokens.hpp"
#include "autoviz/ui/theme/style.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/teleop/teleop_control_widget.hpp"
#include "autoviz/ui/teleop/teleop_hud_overlay.hpp"
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
      return QObject::tr("Active");
    case tp::TELEOP_STATUS_TIMEOUT:
      return QObject::tr("Timeout");
    case tp::TELEOP_STATUS_REJECTED:
      return QObject::tr("Rejected");
    case tp::TELEOP_STATUS_IDLE:
      return QObject::tr("Idle");
    default:
      return QObject::tr("Unknown");
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
  setMinimumWidth(240);
  setStyleSheet(
      style::sheet(QStringLiteral("teleop/panel_root")));

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

  hud_placeholder_ = new QLabel(this);
  hud_placeholder_->setObjectName(QStringLiteral("TeleopHudPlaceholder"));
  hud_placeholder_->setAlignment(Qt::AlignCenter);
  hud_placeholder_->setWordWrap(true);
  hud_placeholder_->setText(
      tr("Gamepad HUD is floating above all panels.\n"
         "Click the HUD button in the title bar to return it here."));
  hud_placeholder_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/hud_placeholder")));
  hud_placeholder_->setMinimumHeight(80);
  hud_placeholder_->hide();
  root->addWidget(hud_placeholder_, 1);

  control_ = new TeleopControlWidget(this);
  root->addWidget(control_, 1);

  setContextMenuPolicy(Qt::CustomContextMenu);
  control_->setContextMenuPolicy(Qt::CustomContextMenu);
  auto copy_topic = [this](const QPoint& pos) {
    QMenu menu(this);
    QAction* copy = menu.addAction(tr("Copy topic"));
    QWidget* source = qobject_cast<QWidget*>(sender());
    if (source == nullptr) {
      source = this;
    }
    if (menu.exec(source->mapToGlobal(pos)) != copy) {
      return;
    }
    if (QClipboard* clipboard = QApplication::clipboard()) {
      clipboard->setText(config_.topic);
    }
  };
  connect(this, &QWidget::customContextMenuRequested, this, copy_topic);
  connect(control_, &QWidget::customContextMenuRequested, this, copy_topic);

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
            if (linear_active_ || angular_active_) {
              publishComposedTwist();
            }
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
  restoreControlFromHud();
  if (overlay_ != nullptr) {
    overlay_->hide();
    overlay_->deleteLater();
    overlay_ = nullptr;
  }
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
      CreatePanelTitleBarTools(dock, callbacks, options);
  settings_button_ = tools.settings_button;

  if (tools.widget != nullptr) {
    hud_button_ = CreateTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("panel.hud")),
        tr("Gamepad overlay — float above all panels"), true);
    hud_button_->setChecked(config_.overlay_enabled);
    hud_button_->setToolTip(
        hud_button_->isChecked()
            ? tr("Return gamepad to this panel")
            : tr("Gamepad overlay — float above all panels, no snap"));
    if (auto* layout = qobject_cast<QHBoxLayout*>(tools.widget->layout())) {
      int insert_at = 0;
      if (settings_button_ != nullptr) {
        const int settings_index = layout->indexOf(settings_button_);
        if (settings_index >= 0) {
          insert_at = settings_index + 1;
        }
      }
      layout->insertWidget(insert_at, hud_button_);
    }
    QObject::connect(hud_button_, &QToolButton::clicked, this, [this](bool checked) {
      setHudVisible(checked, true);
    });
  }

  dock->setTitleBarTools(tools.widget);
}

TeleopPanelConfig TeleopPanel::config() const {
  TeleopPanelConfig out = config_;
  if (overlay_ != nullptr && overlay_->isVisible()) {
    const QRect geo = overlay_->geometry();
    out.overlay_enabled = true;
    out.overlay_opacity = overlay_->opacity();
    out.overlay_x = geo.x();
    out.overlay_y = geo.y();
    out.overlay_w = geo.width();
    out.overlay_h = geo.height();
  }
  return out;
}

void TeleopPanel::setConfig(const TeleopPanelConfig& config) {
  if (config_.smart_teleop_enabled && !config.smart_teleop_enabled) {
    publishTeleopSessionStop();
  }
  if (config_.topic != config.topic) {
    cmd_vel_advertised_ = false;
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
  applyHudFromConfig();
}

void TeleopPanel::cloneConfigFrom(const TeleopPanelConfig& config) {
  publishStop();
  config_ = config;
  // A split panel should not spawn a second HUD over the same pad.
  config_.overlay_enabled = false;
  if (control_ != nullptr) {
    control_->setStickMode(config_.stick_mode);
    control_->setMaxSpeeds(config_.max_linear_speed, config_.max_angular_speed);
  }
  syncSettingsWidgetFromConfig();
  syncSettingsToolState();
  applyHudFromConfig();
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

void TeleopPanel::setHudVisible(bool visible) { setHudVisible(visible, true); }

bool TeleopPanel::hudVisible() const {
  return overlay_ != nullptr && overlay_->isVisible();
}

void TeleopPanel::setHudVisible(bool visible, bool persist) {
  if (visible) {
    ensureHudOverlay();
    if (overlay_ == nullptr || control_ == nullptr) {
      return;
    }
    if (overlay_->control() != control_) {
      if (auto* root = qobject_cast<QVBoxLayout*>(layout())) {
        root->removeWidget(control_);
      }
      overlay_->setControl(control_);
      control_->setHudMode(true);
    }
    overlay_->setOpacity(config_.overlay_opacity);
    const bool was_visible = overlay_->isVisible();
    if (!was_visible) {
      const QRect saved(config_.overlay_x, config_.overlay_y, config_.overlay_w,
                        config_.overlay_h);
      if (!overlay_->applyGeometryIfOnScreen(saved)) {
        overlay_->placeDefault(this);
      }
    }
    overlay_->show();
    overlay_->raise();
    if (!was_visible) {
      overlay_->activateWindow();
      control_->setFocus(Qt::OtherFocusReason);
    }
    if (hud_placeholder_ != nullptr) {
      hud_placeholder_->show();
    }
  } else {
    restoreControlFromHud();
    if (overlay_ != nullptr) {
      overlay_->hide();
    }
    if (hud_placeholder_ != nullptr) {
      hud_placeholder_->hide();
    }
  }
  config_.overlay_enabled = visible;
  syncHudButton();
  if (persist) {
    captureHudGeometry();
    emit configChanged();
  }
}

void TeleopPanel::ensureHudOverlay() {
  if (overlay_ != nullptr) {
    return;
  }
  overlay_ = new TeleopHudOverlay(window());
  overlay_->setOpacity(config_.overlay_opacity);
  connect(overlay_, &TeleopHudOverlay::closeRequested, this, [this]() {
    setHudVisible(false, true);
  });
  connect(overlay_, &TeleopHudOverlay::opacityChanged, this,
          [this](double opacity) {
            config_.overlay_opacity = opacity;
            emit configChanged();
          });
  connect(overlay_, &TeleopHudOverlay::geometryChanged, this, [this]() {
    captureHudGeometry();
    emit configChanged();
  });
  connect(qApp, &QGuiApplication::applicationStateChanged, overlay_,
          [this](Qt::ApplicationState state) {
            if (overlay_ == nullptr || !config_.overlay_enabled) {
              return;
            }
            if (state == Qt::ApplicationActive) {
              overlay_->show();
              overlay_->raise();
            } else {
              overlay_->lower();
            }
          });
}

void TeleopPanel::restoreControlFromHud() {
  if (control_ == nullptr) {
    return;
  }
  QWidget* hosted = nullptr;
  if (overlay_ != nullptr) {
    hosted = overlay_->takeControl();
  }
  if (hosted != control_ && control_->parentWidget() == this) {
    return;
  }
  control_->setParent(this);
  control_->setHudMode(false);
  setAttribute(Qt::WA_TranslucentBackground, false);
  if (auto* root = qobject_cast<QVBoxLayout*>(layout())) {
    if (root->indexOf(control_) < 0) {
      root->addWidget(control_, 1);
    }
  }
  control_->show();
}

void TeleopPanel::applyHudFromConfig() {
  if (overlay_ != nullptr) {
    overlay_->setOpacity(config_.overlay_opacity);
  }
  setHudVisible(config_.overlay_enabled, false);
}

void TeleopPanel::captureHudGeometry() {
  if (overlay_ == nullptr) {
    return;
  }
  const QRect geo = overlay_->geometry();
  config_.overlay_opacity = overlay_->opacity();
  config_.overlay_x = geo.x();
  config_.overlay_y = geo.y();
  config_.overlay_w = geo.width();
  config_.overlay_h = geo.height();
}

void TeleopPanel::syncHudButton() {
  if (hud_button_ == nullptr) {
    return;
  }
  const bool on = hudVisible() || config_.overlay_enabled;
  hud_button_->blockSignals(true);
  hud_button_->setChecked(on);
  hud_button_->setToolTip(
      on ? tr("Return gamepad to this panel")
         : tr("Gamepad overlay — float above all panels, no snap"));
  hud_button_->blockSignals(false);
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
    // Keep driving /cmd_vel until autonomy.task actually ACKs ACTIVE. Write()
    // can succeed with no reader (eager SHM enable), which used to drop all
    // stick motion on a dead TeleopGoal channel.
    if (smart_teleop_session_active_) {
      return;
    }
  }
  publishCmdVel(twist);
}

void TeleopPanel::publishCmdVel(
    const automsgs::msgs::geometry_msgs::Twist& twist) {
  if (config_.topic.isEmpty() || manager_ == nullptr) {
    return;
  }
  // Skip creating a writer for all-zero stop unless we already published
  // (avoids declaring /cmd_vel at panel open and fighting autonomy.control).
  const bool is_zero =
      std::abs(twist.linear().x()) < 1e-9 && std::abs(twist.linear().y()) < 1e-9 &&
      std::abs(twist.linear().z()) < 1e-9 && std::abs(twist.angular().x()) < 1e-9 &&
      std::abs(twist.angular().y()) < 1e-9 && std::abs(twist.angular().z()) < 1e-9;
  if (is_zero && !cmd_vel_advertised_) {
    return;
  }

  automsgs::msgs::geometry_msgs::TwistStamped stamped;
  *stamped.mutable_header()->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeNow();
  stamped.mutable_header()->set_frame_id("base_link");
  *stamped.mutable_twist() = twist;
  std::string payload;
  if (!stamped.SerializeToString(&payload)) {
    LOG_EVERY_N(WARNING, 40) << "Teleop TwistStamped serialize failed";
    return;
  }
  const std::string channel = config_.topic.toStdString();
  if (!integration::ChannelWriterRegistry::instance().publishLoop(
          channel, payload, stamped.GetTypeName())) {
    LOG_EVERY_N(WARNING, 40) << "Teleop /cmd_vel publish failed on " << channel;
    return;
  }
  cmd_vel_advertised_ = true;
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
    smart_teleop_status_text_ = tr("Waiting for task…");
    QMetaObject::invokeMethod(
        this, [this]() { updateSmartTeleopUiStatus(); }, Qt::QueuedConnection);
  } else {
    goal_write_fail_streak_ = 0;
    // Session stays inactive until TeleopFeedback reports ACTIVE so /cmd_vel
    // still drives the plant when autonomy.task is not running.
    if (!teleop_start_logged_this_connect_) {
      teleop_start_logged_this_connect_ = true;
      LOG(INFO) << "TeleopPanel: wrote TELEOP_CMD_START (waiting for feedback)";
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
  if (teleop_feedback_subscription_id_ != 0 || manager_ == nullptr) {
    return;
  }
  if (manager_->autolinkNode() == nullptr) {
    return;
  }
  teleop_feedback_subscription_id_ =
      integration::ChannelReaderRegistry::instance().subscribe(
          integration::kTeleopFeedbackChannel,
          [this](const std::string& payload) {
            tp::TeleopFeedback feedback;
            if (!integration::ParseChannelMessage(payload, &feedback)) {
              return;
            }
            if (feedback.status() == tp::TELEOP_STATUS_ACTIVE) {
              smart_teleop_session_active_ = true;
              stopSessionConnectTimer();
            } else if (feedback.status() == tp::TELEOP_STATUS_REJECTED ||
                       feedback.status() == tp::TELEOP_STATUS_IDLE) {
              smart_teleop_session_active_ = false;
            }
            smart_teleop_status_text_ = TeleopStatusLabel(feedback.status());
            QMetaObject::invokeMethod(
                this, [this]() { updateSmartTeleopUiStatus(); },
                Qt::QueuedConnection);
          });
  if (teleop_feedback_subscription_id_ == 0) {
    LOG(WARNING) << "TeleopPanel: failed to subscribe "
                 << integration::kTeleopFeedbackChannel;
  }
}

void TeleopPanel::shutdownTeleopReaders() {
  stopSessionConnectTimer();
  teleop_goal_writer_.reset();
  if (teleop_feedback_subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        teleop_feedback_subscription_id_);
    teleop_feedback_subscription_id_ = 0;
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
  if (smart_teleop_session_active_) {
    status = smart_teleop_status_text_.isEmpty()
                 ? tr("Connected — drag stick to drive")
                 : smart_teleop_status_text_;
  } else if (!teleop_goal_writer_) {
    status = tr("Task offline — driving %1").arg(config_.topic);
  } else {
    status = tr("Waiting for task — driving %1").arg(config_.topic);
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
  }
  publishCmdVel(ZeroTwist());
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
    setActiveTwist(ZeroTwist());
    if (config_.smart_teleop_enabled) {
      publishTeleopVelocity(ZeroTwist());
    }
    publishCmdVel(ZeroTwist());
    publish_timer_->stop();
    return;
  }
  publishComposedTwist();
}

void TeleopPanel::onAngularReleased() {
  angular_turn_ = 0.0;
  angular_active_ = false;
  if (config_.stop_on_release && !linear_active_) {
    setActiveTwist(ZeroTwist());
    if (config_.smart_teleop_enabled) {
      publishTeleopVelocity(ZeroTwist());
    }
    publishCmdVel(ZeroTwist());
    publish_timer_->stop();
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
