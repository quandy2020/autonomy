/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/raw_messages_panel.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <QAbstractItemView>
#include <QComboBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFont>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/plot/message_path_navigation.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_path_utils.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/raw_message_tree.hpp"
#include "autoviz/variables/variable_path_utils.hpp"

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

google::protobuf::Message* ParseMessageWithGeneratedPool(
    const std::string& normalized, const std::string& payload,
    DynamicFactory::MessagePtr* out) {
  if (out == nullptr || normalized.empty() || payload.empty()) {
    return nullptr;
  }
  const google::protobuf::DescriptorPool* pool =
      google::protobuf::DescriptorPool::generated_pool();
  if (pool == nullptr) {
    return nullptr;
  }
  const google::protobuf::Descriptor* desc = pool->FindMessageTypeByName(normalized);
  if (desc == nullptr) {
    desc = pool->FindMessageTypeByName(StripPackagePrefix(normalized));
  }
  if (desc == nullptr) {
    return nullptr;
  }
  const google::protobuf::Message* prototype =
      google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc);
  if (prototype == nullptr) {
    return nullptr;
  }
  *out = DynamicFactory::MessagePtr(prototype->New());
  if (*out == nullptr || !(*out)->ParseFromString(payload)) {
    out->reset();
    return nullptr;
  }
  return out->get();
}

google::protobuf::Message* ParsePayload(const std::string& message_type,
                                        const std::string& payload,
                                        DynamicFactory::MessagePtr* out) {
  if (out == nullptr || message_type.empty() || payload.empty()) {
    return nullptr;
  }
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  const std::string decoded = integration::DecodeChannelPayload(payload);

  auto try_parse = [&](const std::string& bytes) -> google::protobuf::Message* {
    if (google::protobuf::Message* parsed =
            ParseMessageWithGeneratedPool(normalized, bytes, out)) {
      return parsed;
    }
    static DynamicFactory factory;
    *out = factory.New(normalized);
    if (*out == nullptr) {
      *out = factory.New(StripPackagePrefix(normalized));
    }
    if (*out != nullptr && (*out)->ParseFromString(bytes)) {
      return out->get();
    }
    out->reset();
    return nullptr;
  };

  if (try_parse(decoded.empty() ? payload : decoded) != nullptr) {
    return out->get();
  }
  if (!decoded.empty() && decoded != payload && try_parse(payload) != nullptr) {
    return out->get();
  }
  return nullptr;
}

QString DisplaySchemaName(const std::string& message_type) {
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  if (normalized.empty()) {
    return QStringLiteral("(unknown schema)");
  }
  return QString::fromStdString(normalized);
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
  header->addWidget(new QLabel(tr("Topic"), this));
  channel_combo_ = new QComboBox(this);
  channel_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  header->addWidget(channel_combo_, 1);
  layout->addLayout(header);

  schema_label_ = new QLabel(this);
  schema_label_->setWordWrap(true);
  StyleHintLabel(schema_label_);
  layout->addWidget(schema_label_);

  auto* path_row = new QHBoxLayout();
  path_row->addWidget(new QLabel(tr("Path"), this));
  message_path_edit_ = new QLineEdit(this);
  message_path_edit_->setPlaceholderText(
      tr("Optional message path, e.g. pose.pose.position.x"));
  StyleFilterLineEdit(message_path_edit_);
  path_row->addWidget(message_path_edit_, 1);
  layout->addLayout(path_row);

  message_tree_ = new raw_messages::RawMessageTreeWidget(this);
  message_tree_->setColumnCount(3);
  message_tree_->setHeaderLabels({tr("Name"), tr("Type"), tr("Value")});
  message_tree_->setRootIsDecorated(true);
  message_tree_->setAlternatingRowColors(true);
  message_tree_->setUniformRowHeights(true);
  message_tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  message_tree_->setToolTip(
      tr("Drag a numeric field to Plot, or right-click and choose Add to Plot."));
  message_tree_->header()->setStretchLastSection(true);
  message_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  message_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  QFont mono = message_tree_->font();
  mono.setFamily(QStringLiteral("Monospace"));
  message_tree_->setFont(mono);
  StylePanelTree(message_tree_);
  layout->addWidget(message_tree_, 1);

  connect(message_tree_, &raw_messages::RawMessageTreeWidget::addToPlotRequested, this,
          &RawMessagesPanel::addToPlotRequested);

  connect(channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &RawMessagesPanel::onChannelChanged);
  connect(message_path_edit_, &QLineEdit::editingFinished, this,
          &RawMessagesPanel::onMessagePathEdited);

  if (manager_ != nullptr) {
    connect(&manager_->variableStore(),
            &variables::VariableStore::variablesChanged, this,
            &RawMessagesPanel::refreshFromVariables);
  }

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &RawMessagesPanel::onTick);
  tick_timer_->start(50);

  schema_label_->setText(tr("Select a topic to inspect the latest message."));
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

bool RawMessagesPanel::channelsStructureChanged() {
  if (manager_ == nullptr) {
    const bool changed = !cached_channel_keys_.isEmpty();
    cached_channel_keys_.clear();
    return changed;
  }

  QStringList current;
  current.reserve(static_cast<int>(manager_->channels().size()));
  for (const integration::ChannelInfo& channel : manager_->channels()) {
    current.push_back(QStringLiteral("%1|%2")
                          .arg(QString::fromStdString(channel.channel_name),
                               QString::fromStdString(channel.message_type)));
  }
  current.sort(Qt::CaseInsensitive);
  if (current == cached_channel_keys_) {
    return false;
  }
  cached_channel_keys_ = current;
  return true;
}

void RawMessagesPanel::rebuildChannelCombo() {
  const QString previous = channel_combo_->currentData().toString();
  channel_combo_->blockSignals(true);
  channel_combo_->clear();
  channel_combo_->addItem(tr("(select topic)"), QString());

  if (manager_ != nullptr) {
    for (const integration::ChannelInfo& info : manager_->channels()) {
      const QString channel = QString::fromStdString(info.channel_name);
      const QString label =
          QStringLiteral("%1  [%2]")
              .arg(channel, DisplaySchemaName(info.message_type));
      channel_combo_->addItem(label, channel);
    }
  }

  const int restore_index = channel_combo_->findData(previous);
  if (restore_index >= 0) {
    channel_combo_->setCurrentIndex(restore_index);
  }
  channel_combo_->blockSignals(false);

  if (channel_combo_->currentIndex() <= 0) {
    clearSelection();
  } else {
    onChannelChanged(channel_combo_->currentIndex());
  }
}

void RawMessagesPanel::refreshChannels() {
  if (!channelsStructureChanged() && channel_combo_->count() > 0) {
    tryResubscribeIfNeeded();
    return;
  }
  rebuildChannelCombo();
}

void RawMessagesPanel::updateSchemaHeader() {
  if (schema_label_ == nullptr) {
    return;
  }
  if (active_channel_.empty()) {
    schema_label_->setText(tr("Select a topic to inspect the latest message."));
    return;
  }

  const QString channel = QString::fromStdString(active_channel_);
  const QString schema = DisplaySchemaName(messageTypeForChannel(active_channel_));
  schema_label_->setText(QStringLiteral("%1: %2\n%3: %4")
                             .arg(tr("Topic"), channel, tr("Schema"), schema));
}

void RawMessagesPanel::showSchemaPlaceholder() {
  updateSchemaHeader();
  if (message_tree_ == nullptr) {
    return;
  }
  const QString channel = QString::fromStdString(active_channel_);
  message_tree_->setActiveChannel(channel);
  const std::string message_type = messageTypeForChannel(active_channel_);
  if (message_type.empty()) {
    message_tree_->clear();
    auto* item = new QTreeWidgetItem(message_tree_, {tr("Waiting for messages…"), QString(), QString()});
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
    return;
  }
  raw_messages::PopulateSchemaTree(message_tree_, message_type, channel);
}

void RawMessagesPanel::onChannelChanged(int index) {
  if (index <= 0) {
    clearSelection();
    return;
  }
  active_channel_ = channel_combo_->currentData().toString().toStdString();
  payload_queue_.clear();
  last_payload_.clear();
  last_rendered_payload_.clear();
  message_tree_seeded_ = false;
  last_tree_root_label_.clear();
  last_tree_path_filter_.clear();
  if (message_tree_ != nullptr) {
    message_tree_->setActiveChannel(QString::fromStdString(active_channel_));
  }
  updateSchemaHeader();
  showSchemaPlaceholder();
  resubscribe();
}

void RawMessagesPanel::onMessagePathEdited() {
  last_rendered_payload_.clear();
  message_tree_seeded_ = false;
  last_tree_path_filter_.clear();
  if (!last_payload_.empty()) {
    showPayload(last_payload_);
  } else {
    showSchemaPlaceholder();
  }
}

void RawMessagesPanel::refreshFromVariables() {
  if (!last_payload_.empty()) {
    last_rendered_payload_.clear();
    showPayload(last_payload_);
  }
}

void RawMessagesPanel::onTick() {
  tryResubscribeIfNeeded();
  if (auto payload = payload_queue_.takeLatest()) {
    showPayload(*payload);
  }
}

void RawMessagesPanel::tryResubscribeIfNeeded() {
  if (active_channel_.empty() || subscription_id_ != 0) {
    return;
  }
  resubscribe();
}

void RawMessagesPanel::unsubscribe() {
  if (subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        static_cast<integration::ChannelReaderRegistry::SubscriptionId>(
            subscription_id_));
    subscription_id_ = 0;
  }
  payload_queue_.clear();
}

void RawMessagesPanel::clearSelection() {
  unsubscribe();
  active_channel_.clear();
  last_payload_.clear();
  last_rendered_payload_.clear();
  message_tree_seeded_ = false;
  last_tree_root_label_.clear();
  last_tree_path_filter_.clear();
  if (message_tree_ != nullptr) {
    message_tree_->clear();
  }
  updateSchemaHeader();
}

void RawMessagesPanel::resubscribe() {
  unsubscribe();
  if (active_channel_.empty()) {
    return;
  }

  const std::string channel = active_channel_;
  subscription_id_ =
      integration::ChannelReaderRegistry::instance().subscribe(
          channel, [this](const std::string& payload) {
            payload_queue_.push(payload);
          });
  if (subscription_id_ == 0 && message_tree_ != nullptr) {
    message_tree_->clear();
    auto* item = new QTreeWidgetItem(
        message_tree_,
        {tr("Failed to subscribe to %1").arg(QString::fromStdString(channel)), QString(),
         tr("Autolink may not be ready yet.")});
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
  }
}

void RawMessagesPanel::renderMessage(const google::protobuf::Message& message) {
  if (message_tree_ == nullptr) {
    return;
  }
  const QString channel = QString::fromStdString(active_channel_);
  message_tree_->setActiveChannel(channel);
  const QString root_label = QString::fromStdString(
      commsgs::NormalizeMessageType(messageTypeForChannel(active_channel_)));
  QString path_filter = resolvedMessagePath();
  if (manager_ != nullptr && !path_filter.isEmpty()) {
    path_filter = plot::ResolveMessagePath(path_filter, &manager_->variableStore());
  }

  const bool structure_changed =
      message_tree_seeded_ &&
      (root_label != last_tree_root_label_ || path_filter != last_tree_path_filter_);
  const bool apply_initial_expand = !message_tree_seeded_;

  if (!structure_changed &&
      raw_messages::UpdateMessageTreeValues(message_tree_, message, root_label,
                                            path_filter)) {
    last_tree_root_label_ = root_label;
    last_tree_path_filter_ = path_filter;
    message_tree_seeded_ = true;
    return;
  }

  raw_messages::PopulateMessageTree(message_tree_, message, root_label, channel,
                                    path_filter, apply_initial_expand);
  last_tree_root_label_ = root_label;
  last_tree_path_filter_ = path_filter;
  message_tree_seeded_ = true;
}

void RawMessagesPanel::showPayload(const std::string& payload) {
  last_payload_ = payload;
  if (payload == last_rendered_payload_) {
    return;
  }

  updateSchemaHeader();
  const std::string message_type = messageTypeForChannel(active_channel_);
  if (payload.empty()) {
    showSchemaPlaceholder();
    return;
  }

  DynamicFactory::MessagePtr message;
  if (ParsePayload(message_type, payload, &message) == nullptr) {
    message_tree_->clear();
    auto* item = new QTreeWidgetItem(
        message_tree_,
        {tr("Failed to parse message"), DisplaySchemaName(message_type),
         tr("%1 bytes").arg(static_cast<qulonglong>(payload.size()))});
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
    return;
  }

  renderMessage(*message);
  last_rendered_payload_ = payload;
}

std::string RawMessagesPanel::messageTypeForChannel(
    const std::string& channel) const {
  if (channel.empty()) {
    return {};
  }
  return plot::MessageTypeForChannel(
      manager_, QString::fromStdString(channel));
}

}  // namespace autoviz
