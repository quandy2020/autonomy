/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/service/empty_service_call_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <automsgs/msgs/std_msgs/empty.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

EmptyServiceCallPanel::EmptyServiceCallPanel(
    common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);

  channel_edit_ = new QLineEdit(QStringLiteral("/empty_trigger"), this);
  trigger_button_ = new QPushButton(tr("Trigger"), this);

  auto* row = new QHBoxLayout();
  row->addWidget(new QLabel(tr("Channel:"), this));
  row->addWidget(channel_edit_, 1);
  row->addWidget(trigger_button_);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(row);

  connect(trigger_button_, &QPushButton::clicked, this,
          &EmptyServiceCallPanel::onTriggerClicked);
}

void EmptyServiceCallPanel::onTriggerClicked() {
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
  automsgs::msgs::std_msgs::Empty message;
  auto writer =
      node->CreateWriter<automsgs::msgs::std_msgs::Empty>(channel);
  if (writer != nullptr) {
    writer->Write(message);
  }
}

}  // namespace autoviz
