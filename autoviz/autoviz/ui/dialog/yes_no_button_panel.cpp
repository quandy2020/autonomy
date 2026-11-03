/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/dialog/yes_no_button_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <automsgs/msgs/visualization_msgs/yes_no.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

YesNoButtonPanel::YesNoButtonPanel(common::VisualizationManager* manager,
                                   QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);

  channel_edit_ = new QLineEdit(QStringLiteral("/yes_no"), this);
  yes_button_ = new QPushButton(tr("Yes"), this);
  no_button_ = new QPushButton(tr("No"), this);

  auto* row = new QHBoxLayout();
  row->addWidget(new QLabel(tr("Channel:"), this));
  row->addWidget(channel_edit_, 1);
  row->addWidget(yes_button_);
  row->addWidget(no_button_);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(row);

  connect(yes_button_, &QPushButton::clicked, this,
          &YesNoButtonPanel::onYesClicked);
  connect(no_button_, &QPushButton::clicked, this,
          &YesNoButtonPanel::onNoClicked);
}

void YesNoButtonPanel::publishYesNo(bool yes) {
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
  automsgs::msgs::visualization_msgs::YesNo message;
  message.set_yes(yes);
  auto writer =
      node->CreateWriter<automsgs::msgs::visualization_msgs::YesNo>(channel);
  if (writer != nullptr) {
    writer->Write(message);
  }
}

void YesNoButtonPanel::onYesClicked() { publishYesNo(true); }

void YesNoButtonPanel::onNoClicked() { publishYesNo(false); }

}  // namespace autoviz
