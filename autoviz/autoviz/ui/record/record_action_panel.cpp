/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/record/record_action_panel.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <automsgs/msgs/visualization_msgs/record_command.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

RecordActionPanel::RecordActionPanel(common::VisualizationManager* manager,
                                     QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);

  channel_edit_ = new QLineEdit(QStringLiteral("/record_command"), this);
  file_edit_ = new QLineEdit(QStringLiteral("/tmp/autoviz_record.bag"), this);

  auto* record_btn = new QPushButton(tr("Record"), this);
  auto* play_btn = new QPushButton(tr("Play"), this);
  auto* stop_btn = new QPushButton(tr("Stop"), this);

  auto* form = new QFormLayout();
  form->addRow(tr("Channel:"), channel_edit_);
  form->addRow(tr("File:"), file_edit_);

  auto* buttons = new QHBoxLayout();
  buttons->addWidget(record_btn);
  buttons->addWidget(play_btn);
  buttons->addWidget(stop_btn);
  buttons->addStretch(1);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(form);
  layout->addLayout(buttons);

  connect(record_btn, &QPushButton::clicked, this,
          &RecordActionPanel::onRecordClicked);
  connect(play_btn, &QPushButton::clicked, this,
          &RecordActionPanel::onPlayClicked);
  connect(stop_btn, &QPushButton::clicked, this,
          &RecordActionPanel::onStopClicked);
}

void RecordActionPanel::publishCommand(
    automsgs::msgs::visualization_msgs::RecordCommand::Command cmd) {
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
  automsgs::msgs::visualization_msgs::RecordCommand message;
  message.set_command(cmd);
  message.set_file(file_edit_->text().trimmed().toStdString());
  auto writer =
      node->CreateWriter<automsgs::msgs::visualization_msgs::RecordCommand>(channel);
  if (writer != nullptr) {
    writer->Write(message);
  }
}

void RecordActionPanel::onRecordClicked() {
  publishCommand(automsgs::msgs::visualization_msgs::RecordCommand::RECORD);
}

void RecordActionPanel::onPlayClicked() {
  publishCommand(automsgs::msgs::visualization_msgs::RecordCommand::PLAY);
}

void RecordActionPanel::onStopClicked() {
  publishCommand(automsgs::msgs::visualization_msgs::RecordCommand::STOP);
}

}  // namespace autoviz
