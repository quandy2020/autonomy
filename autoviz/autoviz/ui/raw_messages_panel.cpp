/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/raw_messages_panel.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <QComboBox>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QVBoxLayout>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"

namespace autoviz {
namespace {

using automsgs::msgs::DynamicFactory;

std::string StripPackagePrefix(const std::string& type_name) {
  static const char* kPrefix = "automsgs.msgs.";
  if (type_name.rfind(kPrefix, 0) == 0) {
    return type_name.substr(std::char_traits<char>::length(kPrefix));
  }
  return type_name;
}

QString FormatPayload(const std::string& message_type,
                      const std::string& payload) {
  if (payload.empty()) {
    return QStringLiteral("(empty message)");
  }

  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  if (normalized.empty()) {
    return QStringLiteral("(%1 bytes binary payload)")
        .arg(static_cast<qulonglong>(payload.size()));
  }

  static DynamicFactory factory;
  DynamicFactory::MessagePtr message = factory.New(normalized);
  if (message == nullptr) {
    message = factory.New(StripPackagePrefix(normalized));
  }
  if (message == nullptr || !message->ParseFromString(payload)) {
    return QStringLiteral("Failed to parse %1 (%2 bytes)")
        .arg(QString::fromStdString(normalized))
        .arg(static_cast<qulonglong>(payload.size()));
  }
  return QString::fromStdString(message->DebugString());
}

}  // namespace

RawMessagesPanel::RawMessagesPanel(common::VisualizationManager* manager,
                                   QWidget* parent)
    : manager_(manager), QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* header = new QHBoxLayout();
  header->addWidget(new QLabel(tr("Channel"), this));
  channel_combo_ = new QComboBox(this);
  channel_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  header->addWidget(channel_combo_, 1);
  layout->addLayout(header);

  content_ = new QPlainTextEdit(this);
  content_->setReadOnly(true);
  content_->setPlaceholderText(tr("Select a channel to inspect messages."));
  QFont mono = content_->font();
  mono.setFamily(QStringLiteral("Monospace"));
  content_->setFont(mono);
  layout->addWidget(content_, 1);

  connect(channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &RawMessagesPanel::onChannelChanged);

  refreshChannels();
}

RawMessagesPanel::~RawMessagesPanel() { unsubscribe(); }

void RawMessagesPanel::refreshChannels() {
  const QString previous =
      channel_combo_->currentData().toString();
  channel_combo_->blockSignals(true);
  channel_combo_->clear();
  channel_combo_->addItem(tr("(select channel)"), QString());

  if (manager_ != nullptr) {
    for (const integration::ChannelInfo& info : manager_->channels()) {
      const QString channel = QString::fromStdString(info.channel_name);
      const QString label =
          QStringLiteral("%1  [%2]")
              .arg(channel, QString::fromStdString(
                                commsgs::NormalizeMessageType(info.message_type)));
      channel_combo_->addItem(label, channel);
    }
  }

  const int restore_index = channel_combo_->findData(previous);
  if (restore_index >= 0) {
    channel_combo_->setCurrentIndex(restore_index);
  }
  channel_combo_->blockSignals(false);

  if (channel_combo_->currentIndex() <= 0) {
    unsubscribe();
    content_->clear();
  } else {
    onChannelChanged(channel_combo_->currentIndex());
  }
}

void RawMessagesPanel::onChannelChanged(int index) {
  if (index <= 0) {
    unsubscribe();
    content_->clear();
    return;
  }
  active_channel_ = channel_combo_->currentData().toString().toStdString();
  resubscribe();
}

void RawMessagesPanel::unsubscribe() {
  if (subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        static_cast<integration::ChannelReaderRegistry::SubscriptionId>(
            subscription_id_));
    subscription_id_ = 0;
  }
  active_channel_.clear();
}

void RawMessagesPanel::resubscribe() {
  unsubscribe();
  if (active_channel_.empty()) {
    return;
  }
  subscription_id_ =
      integration::ChannelReaderRegistry::instance().subscribe(
          active_channel_, [this](const std::string& payload) {
            showPayload(payload);
          });
}

void RawMessagesPanel::showPayload(const std::string& payload) {
  const std::string message_type = messageTypeForChannel(active_channel_);
  content_->setPlainText(FormatPayload(message_type, payload));
}

std::string RawMessagesPanel::messageTypeForChannel(
    const std::string& channel) const {
  if (manager_ == nullptr) {
    return {};
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (info.channel_name == channel) {
      return info.message_type;
    }
  }
  return {};
}

}  // namespace autoviz
