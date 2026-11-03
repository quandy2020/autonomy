/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/object_fit/object_fit_operator_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <automsgs/msgs/visualization_msgs/object_fit_command.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

ObjectFitOperatorPanel::ObjectFitOperatorPanel(
    common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);

  channel_edit_ = new QLineEdit(QStringLiteral("/object_fit"), this);
  auto* fit_btn = new QPushButton(tr("Fit"), this);
  auto* clear_btn = new QPushButton(tr("Clear"), this);

  auto* row = new QHBoxLayout();
  row->addWidget(new QLabel(tr("Channel:"), this));
  row->addWidget(channel_edit_, 1);
  row->addWidget(fit_btn);
  row->addWidget(clear_btn);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(row);

  connect(fit_btn, &QPushButton::clicked, this,
          &ObjectFitOperatorPanel::onFitClicked);
  connect(clear_btn, &QPushButton::clicked, this,
          &ObjectFitOperatorPanel::onClearClicked);
}

void ObjectFitOperatorPanel::publishCommand(
    automsgs::msgs::visualization_msgs::ObjectFitCommand::Command cmd) {
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
  automsgs::msgs::visualization_msgs::ObjectFitCommand message;
  message.set_command(cmd);
  auto writer =
      node->CreateWriter<automsgs::msgs::visualization_msgs::ObjectFitCommand>(
          channel);
  if (writer != nullptr) {
    writer->Write(message);
  }
}

void ObjectFitOperatorPanel::onFitClicked() {
  publishCommand(automsgs::msgs::visualization_msgs::ObjectFitCommand::FIT);
}

void ObjectFitOperatorPanel::onClearClicked() {
  publishCommand(automsgs::msgs::visualization_msgs::ObjectFitCommand::CLEAR);
}

}  // namespace autoviz
