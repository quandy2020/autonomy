/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/teleop/robot_command_interface_panel.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <automsgs/msgs/visualization_msgs/string_stamped.pb.h>
#include <automsgs/msgs/time_utils.hpp>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

RobotCommandInterfacePanel::RobotCommandInterfacePanel(
    common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);

  channel_edit_ = new QLineEdit(QStringLiteral("/robot_command"), this);
  command_edit_ = new QLineEdit(this);
  command_edit_->setPlaceholderText(tr("Command string"));

  auto* send_btn = new QPushButton(tr("Send"), this);

  auto* form = new QFormLayout();
  form->addRow(tr("Channel:"), channel_edit_);
  form->addRow(tr("Command:"), command_edit_);

  auto* row = new QHBoxLayout();
  row->addStretch(1);
  row->addWidget(send_btn);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(form);
  layout->addLayout(row);

  connect(send_btn, &QPushButton::clicked, this,
          &RobotCommandInterfacePanel::onSendClicked);
}

void RobotCommandInterfacePanel::onSendClicked() {
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
  message.set_data(command_edit_->text().toStdString());
  auto writer =
      node->CreateWriter<automsgs::msgs::visualization_msgs::StringStamped>(channel);
  if (writer != nullptr) {
    writer->Write(message);
  }
}

}  // namespace autoviz
