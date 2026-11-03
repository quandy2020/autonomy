/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/ui/publish/publish_topic_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <automsgs/msgs/std_msgs/string.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

PublishTopicPanel::PublishTopicPanel(common::VisualizationManager* manager,
                                     QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);

  note_label_ = new QLabel(
      tr("For arbitrary message types use the Autoviz Publish panel."), this);
  note_label_->setWordWrap(true);

  channel_edit_ = new QLineEdit(QStringLiteral("/publish_click"), this);
  publish_button_ = new QPushButton(tr("Publish \"clicked\""), this);

  auto* row = new QHBoxLayout();
  row->addWidget(new QLabel(tr("Channel:"), this));
  row->addWidget(channel_edit_, 1);
  row->addWidget(publish_button_);

  auto* layout = new QVBoxLayout(this);
  layout->addWidget(note_label_);
  layout->addLayout(row);

  connect(publish_button_, &QPushButton::clicked, this,
          &PublishTopicPanel::onPublishClicked);
}

void PublishTopicPanel::onPublishClicked() {
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
  automsgs::msgs::std_msgs::String message;
  message.set_data("clicked");
  auto writer =
      node->CreateWriter<automsgs::msgs::std_msgs::String>(channel);
  if (writer != nullptr) {
    writer->Write(message);
  }
}

}  // namespace autoviz
