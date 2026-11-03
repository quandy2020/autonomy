/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/teleop/tablet_controller_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <automsgs/msgs/visualization_msgs/string_stamped.pb.h>
#include <automsgs/msgs/time_utils.hpp>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

TabletControllerPanel::TabletControllerPanel(
    common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);

  channel_edit_ = new QLineEdit(QStringLiteral("/tablet_cmd"), this);
  forward_button_ = new QPushButton(tr("Forward"), this);
  back_button_ = new QPushButton(tr("Back"), this);
  left_button_ = new QPushButton(tr("Left"), this);
  right_button_ = new QPushButton(tr("Right"), this);
  stop_button_ = new QPushButton(tr("Stop"), this);

  auto* pad = new QGridLayout();
  pad->addWidget(forward_button_, 0, 1);
  pad->addWidget(left_button_, 1, 0);
  pad->addWidget(stop_button_, 1, 1);
  pad->addWidget(right_button_, 1, 2);
  pad->addWidget(back_button_, 2, 1);

  auto* top = new QHBoxLayout();
  top->addWidget(new QLabel(tr("Channel:"), this));
  top->addWidget(channel_edit_, 1);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(top);
  layout->addLayout(pad);

  connect(forward_button_, &QPushButton::clicked, this,
          &TabletControllerPanel::onDirectionClicked);
  connect(back_button_, &QPushButton::clicked, this,
          &TabletControllerPanel::onDirectionClicked);
  connect(left_button_, &QPushButton::clicked, this,
          &TabletControllerPanel::onDirectionClicked);
  connect(right_button_, &QPushButton::clicked, this,
          &TabletControllerPanel::onDirectionClicked);
  connect(stop_button_, &QPushButton::clicked, this,
          &TabletControllerPanel::onDirectionClicked);
}

void TabletControllerPanel::publishCommand(const QString& command) {
  if (manager_ == nullptr) {
    return;
  }
  const auto node = manager_->autolinkNode();
  if (node == nullptr) {
    return;
  }
  const std::string channel = channel_edit_->text().trimmed().toStdString();
  if (channel.empty()) {
    return;
  }
  automsgs::msgs::visualization_msgs::StringStamped message;
  *message.mutable_header()->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeNow();
  message.mutable_header()->set_frame_id("map");
  message.set_data(command.toStdString());
  auto writer =
      node->CreateWriter<automsgs::msgs::visualization_msgs::StringStamped>(channel);
  if (writer != nullptr) {
    writer->Write(message);
  }
}

void TabletControllerPanel::onDirectionClicked() {
  const auto* button = qobject_cast<QPushButton*>(sender());
  if (button == forward_button_) {
    publishCommand(QStringLiteral("forward"));
  } else if (button == back_button_) {
    publishCommand(QStringLiteral("back"));
  } else if (button == left_button_) {
    publishCommand(QStringLiteral("left"));
  } else if (button == right_button_) {
    publishCommand(QStringLiteral("right"));
  } else if (button == stop_button_) {
    publishCommand(QStringLiteral("stop"));
  }
}

}  // namespace autoviz
