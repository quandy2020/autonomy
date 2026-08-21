/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <unordered_map>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/time_utils.hpp>

#include "autolink/node/writer.hpp"

#include "autoviz/common/visualization_manager.hpp"
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


double ScaleSignedAxis(double normalized, double positive_value,
                       double negative_value) {
  if (normalized >= 0.0) {
    return normalized * positive_value;
  }
  return normalized * std::abs(negative_value);
}

}  // namespace

TeleopPanel::TeleopPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultTeleopPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);

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

  updatePublishTimer();
  syncSettingsWidgetFromConfig();
}

TeleopPanel::~TeleopPanel() {
  publishStop();
  publish_timer_->stop();
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
  config_ = config;
  updatePublishTimer();
  syncSettingsWidgetFromConfig();
  syncSettingsToolState();
}

void TeleopPanel::cloneConfigFrom(const TeleopPanelConfig& config) {
  publishStop();
  config_ = config;
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
  publishTwist(ZeroTwist());
  publish_timer_->stop();
}

automsgs::msgs::geometry_msgs::Twist TeleopPanel::composeTwist() const {
  automsgs::msgs::geometry_msgs::Twist twist = ZeroTwist();

  const double forward = -linear_y_;
  twist.mutable_linear()->set_x(
      ScaleSignedAxis(forward, config_.up.value, config_.down.value));
  twist.mutable_linear()->set_y(
      ScaleSignedAxis(linear_x_, config_.right.value, config_.left.value));

  const double turn = angular_turn_;
  twist.mutable_angular()->set_z(
      turn >= 0.0 ? turn * config_.right.value : turn * (-config_.left.value));

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
    publishStop();
    return;
  }
  publishComposedTwist();
}

void TeleopPanel::onAngularReleased() {
  angular_turn_ = 0.0;
  angular_active_ = false;
  if (config_.stop_on_release && !linear_active_) {
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
