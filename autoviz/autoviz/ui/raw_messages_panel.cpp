/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/raw_messages_panel.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <QComboBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QPlainTextEdit>
#include <QVBoxLayout>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/plot/message_path_navigation.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/variables/variable_path_utils.hpp"
#include "autoviz/variables/variable_store.hpp"

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

google::protobuf::Message* ParseMessage(const std::string& message_type,
                                        const std::string& payload,
                                        DynamicFactory::MessagePtr* out) {
  if (out == nullptr || message_type.empty() || payload.empty()) {
    return nullptr;
  }
  static DynamicFactory factory;
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  *out = factory.New(normalized);
  if (*out == nullptr) {
    *out = factory.New(StripPackagePrefix(normalized));
  }
  if (*out == nullptr || !(*out)->ParseFromString(payload)) {
    out->reset();
    return nullptr;
  }
  return out->get();
}

QString FormatPayload(const std::string& message_type,
                      const std::string& payload,
                      const QString& message_path,
                      common::VisualizationManager* manager) {
  if (payload.empty()) {
    return QStringLiteral("(empty message)");
  }

  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  if (normalized.empty()) {
    return QStringLiteral("(%1 bytes binary payload)")
        .arg(static_cast<qulonglong>(payload.size()));
  }

  DynamicFactory::MessagePtr message;
  if (ParseMessage(message_type, payload, &message) == nullptr) {
    return QStringLiteral("Failed to parse %1 (%2 bytes)")
        .arg(QString::fromStdString(normalized))
        .arg(static_cast<qulonglong>(payload.size()));
  }

  if (!message_path.trimmed().isEmpty()) {
    const QString resolved =
        manager != nullptr
            ? plot::ResolveMessagePath(message_path.trimmed(),
                                       &manager->variableStore())
            : message_path.trimmed();
    const std::optional<std::string> filtered =
        plot::FormatMessagePathValue(*message, resolved.toStdString());
    if (!filtered.has_value()) {
      return QStringLiteral("No value at path \"%1\"").arg(resolved);
    }
    return QString::fromStdString(*filtered);
  }

  return QString::fromStdString(message->DebugString());
}

}  // namespace

RawMessagesPanel::RawMessagesPanel(common::VisualizationManager* manager,
                                   QWidget* parent)
    : manager_(manager), QWidget(parent) {
  setAcceptDrops(true);
  ApplyPanelShell(this);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                             PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  layout->setSpacing(PanelSettingsLayout::kOuterSpacing);

  auto* header = new QHBoxLayout();
  header->addWidget(new QLabel(tr("Channel"), this));
  channel_combo_ = new QComboBox(this);
  channel_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  header->addWidget(channel_combo_, 1);
  layout->addLayout(header);

  auto* path_row = new QHBoxLayout();
  path_row->addWidget(new QLabel(tr("Path"), this));
  message_path_edit_ = new QLineEdit(this);
  message_path_edit_->setPlaceholderText(
      tr("Optional message path, e.g. objects[:]{id==$vehicle_id}"));
  StyleFilterLineEdit(message_path_edit_);
  path_row->addWidget(message_path_edit_, 1);
  layout->addLayout(path_row);

  content_ = new QPlainTextEdit(this);
  content_->setReadOnly(true);
  content_->setPlaceholderText(tr("Select a channel to inspect messages."));
  QFont mono = content_->font();
  mono.setFamily(QStringLiteral("Monospace"));
  content_->setFont(mono);
  layout->addWidget(content_, 1);

  connect(channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &RawMessagesPanel::onChannelChanged);
  connect(message_path_edit_, &QLineEdit::editingFinished, this,
          &RawMessagesPanel::onMessagePathEdited);

  if (manager_ != nullptr) {
    connect(&manager_->variableStore(),
            &variables::VariableStore::variablesChanged, this,
            &RawMessagesPanel::refreshFromVariables);
  }

  refreshChannels();
}

void RawMessagesPanel::selectChannel(const QString& channel) {
  if (channel.isEmpty()) {
    return;
  }
  const int index = channel_combo_->findData(channel);
  if (index >= 0) {
    channel_combo_->setCurrentIndex(index);
  }
}

QString RawMessagesPanel::resolvedMessagePath() const {
  return message_path_edit_ != nullptr ? message_path_edit_->text().trimmed()
                                       : QString();
}

bool RawMessagesPanel::acceptChannelDrop(const QMimeData* mime) const {
  plot::PlotSeriesDragPayload payload;
  if (!plot::ReadPlotSeriesDragPayload(mime, &payload) || payload.channel.isEmpty()) {
    return false;
  }
  return true;
}

void RawMessagesPanel::dragEnterEvent(QDragEnterEvent* event) {
  if (acceptChannelDrop(event->mimeData())) {
    event->acceptProposedAction();
    return;
  }
  QWidget::dragEnterEvent(event);
}

void RawMessagesPanel::dragMoveEvent(QDragMoveEvent* event) {
  if (acceptChannelDrop(event->mimeData())) {
    event->acceptProposedAction();
    return;
  }
  QWidget::dragMoveEvent(event);
}

void RawMessagesPanel::dropEvent(QDropEvent* event) {
  plot::PlotSeriesDragPayload payload;
  if (!plot::ReadPlotSeriesDragPayload(event->mimeData(), &payload) ||
      payload.channel.isEmpty()) {
    QWidget::dropEvent(event);
    return;
  }
  selectChannel(payload.channel);
  if (!payload.field_path.isEmpty() && message_path_edit_ != nullptr) {
    message_path_edit_->setText(payload.field_path);
    onMessagePathEdited();
  }
  event->acceptProposedAction();
}

RawMessagesPanel::~RawMessagesPanel() { unsubscribe(); }

void RawMessagesPanel::refreshChannels() {
  const QString previous = channel_combo_->currentData().toString();
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
    last_payload_.clear();
    return;
  }
  active_channel_ = channel_combo_->currentData().toString().toStdString();
  resubscribe();
}

void RawMessagesPanel::onMessagePathEdited() {
  if (!last_payload_.empty()) {
    showPayload(last_payload_);
  }
}

void RawMessagesPanel::refreshFromVariables() {
  if (!last_payload_.empty()) {
    showPayload(last_payload_);
  }
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
  last_payload_ = payload;
  const std::string message_type = messageTypeForChannel(active_channel_);
  content_->setPlainText(
      FormatPayload(message_type, payload, resolvedMessagePath(), manager_));
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
