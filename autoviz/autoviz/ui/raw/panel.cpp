/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/raw/panel.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <algorithm>
#include <cstring>
#include <unordered_set>
#include <vector>

#include <QAbstractItemView>
#include <QApplication>
#include <QClipboard>
#include <QColor>
#include <QComboBox>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QSizePolicy>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <google/protobuf/util/json_util.h>

#include "autolink/common/types.hpp"
#include "autoviz/common/protobuf_json_compat.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/channel_stats_registry.hpp"
#include "autoviz/ui/plot/message_path_navigation.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_path_utils.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/raw/message_tree.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

using automsgs::msgs::DynamicFactory;

/** Light tokens shared with Channels / Log / TF Tree. */
constexpr char kBg[] = "rgba(255,255,255,128)";
constexpr char kSurface[] = "rgba(240,249,255,155)";
constexpr char kBorder[] = "rgba(186,230,253,140)";
constexpr char kText[] = "#1e293b";
constexpr char kTextMuted[] = "#64748b";
constexpr char kAccent[] = "#0891b2";

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

QString ShortSchemaName(const std::string& message_type) {
  QString schema = DisplaySchemaName(message_type);
  if (schema.startsWith(QLatin1String("automsgs.msgs."))) {
    schema = schema.mid(14);
  }
  return schema;
}

bool EndsWith(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/** Hide service transport + Autolink Action protocol channels (matches CLI). */
bool IsServiceOrActionChannel(const std::string& channel,
                              const std::unordered_set<std::string>& all) {
  if (EndsWith(channel, ::autolink::SRV_CHANNEL_REQ_SUFFIX) ||
      EndsWith(channel, ::autolink::SRV_CHANNEL_RES_SUFFIX) ||
      channel.find("__SRV__") != std::string::npos) {
    return true;
  }
  static const char* kActionPubSubSuffixes[] = {"/feedback", "/status"};
  for (const char* suffix : kActionPubSubSuffixes) {
    if (!EndsWith(channel, suffix)) {
      continue;
    }
    const std::string base =
        channel.substr(0, channel.size() - std::strlen(suffix));
    if (all.count(base + "/send_goal" + ::autolink::SRV_CHANNEL_REQ_SUFFIX) ||
        all.count(base + "/send_goal" + ::autolink::SRV_CHANNEL_RES_SUFFIX)) {
      return true;
    }
  }
  return false;
}

std::vector<integration::ChannelInfo> PubSubChannels(
    common::VisualizationManager* manager) {
  std::vector<integration::ChannelInfo> out;
  if (manager == nullptr) {
    return out;
  }
  const auto& all_infos = manager->channels();
  std::unordered_set<std::string> all_names;
  all_names.reserve(all_infos.size());
  for (const integration::ChannelInfo& info : all_infos) {
    all_names.insert(info.channel_name);
  }
  out.reserve(all_infos.size());
  for (const integration::ChannelInfo& info : all_infos) {
    if (IsServiceOrActionChannel(info.channel_name, all_names)) {
      continue;
    }
    out.push_back(info);
  }
  return out;
}

QLabel* MakeFieldCaption(const QString& text, QWidget* parent) {
  auto* label = new QLabel(text, parent);
  label->setStyleSheet(style::type(
      style::Role::PanelMuted, 10, 700, false,
      QStringLiteral("letter-spacing: 0.04em;")));
  return label;
}

}  // namespace

RawMessagesPanel::RawMessagesPanel(common::VisualizationManager* manager,
                                   QWidget* parent)
    : manager_(manager), QWidget(parent) {
  setAcceptDrops(true);
  ApplyPanelShell(this);
  setObjectName(QStringLiteral("RawMessagesPanelContent"));
  applyChromeStyles();

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto* toolbar = new QFrame(this);
  toolbar->setObjectName(QStringLiteral("RawMessagesToolbar"));
  auto* toolbar_layout = new QVBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(10, 8, 10, 8);
  toolbar_layout->setSpacing(8);

  auto* channel_row = new QHBoxLayout();
  channel_row->setSpacing(8);
  channel_row->addWidget(MakeFieldCaption(tr("CHANNEL"), toolbar));

  channel_combo_ = new QComboBox(toolbar);
  channel_combo_->setObjectName(QStringLiteral("AutovizCyanCombo"));
  channel_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  channel_combo_->setStyleSheet(
      style::sheet(QStringLiteral("widget/fields"), style::Tone::Frost));
  channel_row->addWidget(channel_combo_, 1);

  auto* expand_button = new QToolButton(toolbar);
  expand_button->setObjectName(QStringLiteral("AutovizCyanToolButton"));
  expand_button->setText(tr("Expand"));
  expand_button->setToolTip(tr("Expand all message fields"));
  expand_button->setCursor(Qt::PointingHandCursor);
  expand_button->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  channel_row->addWidget(expand_button);

  auto* collapse_button = new QToolButton(toolbar);
  collapse_button->setObjectName(QStringLiteral("AutovizCyanToolButton"));
  collapse_button->setText(tr("Collapse"));
  collapse_button->setToolTip(tr("Collapse all message fields"));
  collapse_button->setCursor(Qt::PointingHandCursor);
  collapse_button->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  channel_row->addWidget(collapse_button);

  freeze_button_ = new QToolButton(toolbar);
  freeze_button_->setObjectName(QStringLiteral("AutovizCyanToolButton"));
  freeze_button_->setText(tr("Freeze"));
  freeze_button_->setCheckable(true);
  freeze_button_->setToolTip(tr("Freeze the tree on the current message"));
  freeze_button_->setCursor(Qt::PointingHandCursor);
  freeze_button_->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  channel_row->addWidget(freeze_button_);

  diff_button_ = new QToolButton(toolbar);
  diff_button_->setObjectName(QStringLiteral("AutovizCyanToolButton"));
  diff_button_->setText(tr("Diff"));
  diff_button_->setCheckable(true);
  diff_button_->setChecked(true);
  diff_button_->setToolTip(tr("Highlight fields that changed vs previous frame"));
  diff_button_->setCursor(Qt::PointingHandCursor);
  diff_button_->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  channel_row->addWidget(diff_button_);

  status_label_ = new QLabel(toolbar);
  status_label_->setObjectName(QStringLiteral("AutovizCyanStatusChip"));
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_label_->setStyleSheet(
      style::sheet(QStringLiteral("widget/badges"), style::Tone::Frost));
  channel_row->addWidget(status_label_);
  toolbar_layout->addLayout(channel_row);

  auto* schema_row = new QHBoxLayout();
  schema_row->setSpacing(8);
  schema_badge_ = new QLabel(tr("SCHEMA"), toolbar);
  schema_badge_->setObjectName(QStringLiteral("AutovizAccentBadge"));
  schema_badge_->setStyleSheet(
      style::sheet(QStringLiteral("widget/badges"), style::Tone::Frost));
  schema_row->addWidget(schema_badge_, 0, Qt::AlignVCenter);

  schema_label_ = new QLabel(toolbar);
  schema_label_->setWordWrap(true);
  schema_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  schema_label_->setStyleSheet(
      style::type(style::Role::PanelBody, 12));
  schema_row->addWidget(schema_label_, 1);
  toolbar_layout->addLayout(schema_row);

  auto* path_row = new QHBoxLayout();
  path_row->setSpacing(8);
  path_row->addWidget(MakeFieldCaption(tr("PATH"), toolbar));
  message_path_edit_ = new QLineEdit(toolbar);
  message_path_edit_->setObjectName(QStringLiteral("AutovizCyanLineEdit"));
  message_path_edit_->setPlaceholderText(
      tr("Optional message path, e.g. pose.pose.position.x"));
  message_path_edit_->setClearButtonEnabled(true);
  message_path_edit_->setStyleSheet(
      style::sheet(QStringLiteral("widget/fields"), style::Tone::Frost));
  path_row->addWidget(message_path_edit_, 1);
  toolbar_layout->addLayout(path_row);

  auto* drag_hint = new QLabel(
      tr("Drop a channel here, or drag numeric fields to Plot."), toolbar);
  drag_hint->setStyleSheet(
      style::type(style::Role::PanelMuted, 11));
  toolbar_layout->addWidget(drag_hint);
  layout->addWidget(toolbar);

  auto* tree_card = new QFrame(this);
  tree_card->setObjectName(QStringLiteral("RawMessagesTreeCard"));
  auto* tree_card_layout = new QVBoxLayout(tree_card);
  tree_card_layout->setContentsMargins(0, 0, 0, 0);
  tree_card_layout->setSpacing(0);

  message_tree_ = new raw_messages::RawMessageTreeWidget(tree_card);
  message_tree_->setObjectName(QStringLiteral("RawMessagesTree"));
  message_tree_->setColumnCount(3);
  message_tree_->setHeaderLabels({tr("Name"), tr("Type"), tr("Value")});
  message_tree_->setRootIsDecorated(true);
  message_tree_->setAlternatingRowColors(false);
  message_tree_->setUniformRowHeights(true);
  message_tree_->setAnimated(true);
  message_tree_->setIndentation(18);
  message_tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  message_tree_->setToolTip(
      tr("Drag a numeric field to Plot. Right-click to copy path / value / JSON."));
  message_tree_->header()->setStretchLastSection(true);
  message_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  message_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  message_tree_->header()->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  QFont mono = message_tree_->font();
  mono.setFamily(QStringLiteral("Monospace"));
  mono.setPointSizeF(std::max(10.0, mono.pointSizeF() - 0.5));
  message_tree_->setFont(mono);
  message_tree_->setStyleSheet(
      style::sheet(QStringLiteral("raw_messages"), style::Tone::Frost));
  tree_card_layout->addWidget(message_tree_, 1);

  empty_hint_ = new QFrame(tree_card);
  empty_hint_->setObjectName(QStringLiteral("RawMessagesEmpty"));
  auto* empty_layout = new QVBoxLayout(empty_hint_);
  empty_layout->setContentsMargins(24, 40, 24, 40);
  empty_layout->setAlignment(Qt::AlignCenter);
  auto* empty_title = new QLabel(tr("Select a channel"), empty_hint_);
  empty_title->setAlignment(Qt::AlignCenter);
  empty_title->setStyleSheet(
      style::type(style::Role::PanelBody, 15, 600));
  empty_layout->addWidget(empty_title);
  auto* empty_body = new QLabel(
      tr("Choose a pub/sub channel above, or drag one from the Channels panel."),
      empty_hint_);
  empty_body->setAlignment(Qt::AlignCenter);
  empty_body->setWordWrap(true);
  empty_body->setStyleSheet(
      style::type(style::Role::PanelMuted, 12));
  empty_layout->addWidget(empty_body);
  empty_hint_->hide();
  tree_card_layout->addWidget(empty_hint_, 1);
  layout->addWidget(tree_card, 1);

  auto* footer = new QFrame(this);
  footer->setObjectName(QStringLiteral("RawMessagesFooter"));
  auto* footer_layout = new QHBoxLayout(footer);
  footer_layout->setContentsMargins(12, 6, 12, 6);
  auto* footer_label = new QLabel(
      tr("Double-click a row to expand or collapse its subtree"), footer);
  footer_label->setStyleSheet(
      style::type(style::Role::PanelMuted, 11));
  footer_layout->addWidget(footer_label);
  layout->addWidget(footer);

  connect(message_tree_, &raw_messages::RawMessageTreeWidget::addToPlotRequested, this,
          &RawMessagesPanel::addToPlotRequested);
  connect(message_tree_,
          &raw_messages::RawMessageTreeWidget::copyMessageJsonRequested, this,
          &RawMessagesPanel::onCopyMessageJsonRequested);
  connect(channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &RawMessagesPanel::onChannelChanged);
  connect(message_path_edit_, &QLineEdit::editingFinished, this,
          &RawMessagesPanel::onMessagePathEdited);
  connect(freeze_button_, &QToolButton::toggled, this,
          &RawMessagesPanel::onFreezeToggled);
  connect(diff_button_, &QToolButton::toggled, this,
          &RawMessagesPanel::onDiffToggled);
  connect(expand_button, &QToolButton::clicked, this, [this]() {
    if (message_tree_ != nullptr) {
      raw_messages::ExpandAllNodes(message_tree_);
    }
  });
  connect(collapse_button, &QToolButton::clicked, this, [this]() {
    if (message_tree_ != nullptr) {
      raw_messages::CollapseAllNodes(message_tree_);
    }
  });

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &RawMessagesPanel::onTick);
  tick_timer_->start(50);

  updateSchemaHeader();
  refreshStatusChip();
  refreshChannels();
}

void RawMessagesPanel::applyChromeStyles() {
  setStyleSheet(
      style::sheet(QStringLiteral("raw_messages"), style::Tone::Frost));
}

void RawMessagesPanel::updateStatusChip(const QString& text) {
  if (status_label_ == nullptr) {
    return;
  }
  status_label_->setText(text);
}

void RawMessagesPanel::refreshStatusChip() {
  if (active_channel_.empty()) {
    updateStatusChip(tr("Idle"));
    return;
  }
  const integration::ChannelStats stats =
      integration::ChannelStatsRegistry::instance().stats(active_channel_);
  QString hz = QStringLiteral("—");
  if (stats.frequency_hz > 0.0) {
    if (stats.frequency_hz >= 100.0) {
      hz = QString::number(stats.frequency_hz, 'f', 0);
    } else if (stats.frequency_hz >= 10.0) {
      hz = QString::number(stats.frequency_hz, 'f', 1);
    } else {
      hz = QString::number(stats.frequency_hz, 'f', 2);
    }
  }
  if (freeze_) {
    updateStatusChip(tr("Frozen · %1 Hz").arg(hz));
  } else if (subscription_id_ == 0) {
    updateStatusChip(tr("Error"));
  } else if (last_payload_.empty()) {
    updateStatusChip(tr("Waiting · %1 Hz").arg(hz));
  } else {
    updateStatusChip(tr("Live · %1 Hz").arg(hz));
  }
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

common::RawMessagesPersistConfig RawMessagesPanel::config() const {
  common::RawMessagesPersistConfig out;
  out.channel = active_channel_;
  if (message_path_edit_ != nullptr) {
    out.message_path = message_path_edit_->text().trimmed().toStdString();
  }
  out.freeze = freeze_;
  out.diff_highlight = diff_highlight_;
  return out;
}

void RawMessagesPanel::setConfig(const common::RawMessagesPersistConfig& config) {
  freeze_ = config.freeze;
  diff_highlight_ = config.diff_highlight;
  if (freeze_button_ != nullptr) {
    freeze_button_->blockSignals(true);
    freeze_button_->setChecked(freeze_);
    freeze_button_->blockSignals(false);
  }
  if (diff_button_ != nullptr) {
    diff_button_->blockSignals(true);
    diff_button_->setChecked(diff_highlight_);
    diff_button_->blockSignals(false);
  }
  if (message_tree_ != nullptr) {
    message_tree_->setDiffHighlightEnabled(diff_highlight_);
  }
  if (message_path_edit_ != nullptr) {
    message_path_edit_->setText(QString::fromStdString(config.message_path));
  }
  refreshChannels();
  if (!config.channel.empty()) {
    selectChannel(QString::fromStdString(config.channel));
  }
  refreshStatusChip();
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
  const std::string channel = payload.channel.toStdString();
  std::unordered_set<std::string> all_names;
  if (manager_ != nullptr) {
    for (const integration::ChannelInfo& info : manager_->channels()) {
      all_names.insert(info.channel_name);
    }
  }
  all_names.insert(channel);
  return !IsServiceOrActionChannel(channel, all_names);
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

  const std::vector<integration::ChannelInfo> channels = PubSubChannels(manager_);
  QStringList current;
  current.reserve(static_cast<int>(channels.size()));
  for (const integration::ChannelInfo& channel : channels) {
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
  channel_combo_->addItem(tr("(select channel)"), QString());

  for (const integration::ChannelInfo& info : PubSubChannels(manager_)) {
    const QString channel = QString::fromStdString(info.channel_name);
    const QString label =
        QStringLiteral("%1  [%2]")
            .arg(channel, ShortSchemaName(info.message_type));
    channel_combo_->addItem(label, channel);
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
  const bool has_channel = !active_channel_.empty();
  if (empty_hint_ != nullptr && message_tree_ != nullptr) {
    empty_hint_->setVisible(!has_channel);
    message_tree_->setVisible(has_channel);
  }
  if (!has_channel) {
    schema_label_->setText(tr("Select a channel to inspect the latest message."));
    schema_label_->setStyleSheet(
        style::type(style::Role::PanelMuted, 12));
    updateStatusChip(tr("Idle"));
    return;
  }

  const QString channel = QString::fromStdString(active_channel_);
  const QString schema = ShortSchemaName(messageTypeForChannel(active_channel_));
  schema_label_->setText(QStringLiteral("%1  ·  %2").arg(channel, schema));
  schema_label_->setStyleSheet(
      style::type(style::Role::PanelBody, 12, 600));
}

void RawMessagesPanel::showSchemaPlaceholder() {
  updateSchemaHeader();
  if (message_tree_ == nullptr || active_channel_.empty()) {
    return;
  }
  const QString channel = QString::fromStdString(active_channel_);
  message_tree_->setActiveChannel(channel);
  const std::string message_type = messageTypeForChannel(active_channel_);
  if (message_type.empty()) {
    message_tree_->clear();
    auto* item = new QTreeWidgetItem(message_tree_,
                                     {tr("Waiting for messages…"), QString(), QString()});
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
    item->setForeground(0, QColor(kTextMuted));
    updateStatusChip(tr("Waiting"));
    return;
  }
  raw_messages::PopulateSchemaTree(message_tree_, message_type, channel);
  updateStatusChip(tr("Schema"));
}

void RawMessagesPanel::onChannelChanged(int index) {
  if (index <= 0) {
    clearSelection();
    emit configChanged();
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
  emit configChanged();
}

void RawMessagesPanel::onMessagePathEdited() {
  last_rendered_payload_.clear();
  message_tree_seeded_ = false;
  last_tree_path_filter_.clear();
  emit configChanged();
  if (!last_payload_.empty()) {
    showPayload(last_payload_);
  } else {
    showSchemaPlaceholder();
  }
}

void RawMessagesPanel::onFreezeToggled(bool frozen) {
  freeze_ = frozen;
  emit configChanged();
  refreshStatusChip();
  if (!freeze_ && !last_payload_.empty()) {
    last_rendered_payload_.clear();
    showPayload(last_payload_);
  }
}

void RawMessagesPanel::onDiffToggled(bool enabled) {
  diff_highlight_ = enabled;
  if (message_tree_ != nullptr) {
    message_tree_->setDiffHighlightEnabled(enabled);
  }
  emit configChanged();
}

void RawMessagesPanel::onCopyMessageJsonRequested() {
  if (last_payload_.empty() || active_channel_.empty()) {
    return;
  }
  const std::string message_type = messageTypeForChannel(active_channel_);
  DynamicFactory::MessagePtr message;
  if (ParsePayload(message_type, last_payload_, &message) == nullptr ||
      message == nullptr) {
    return;
  }
  google::protobuf::util::JsonPrintOptions options;
  SetAlwaysPrintPrimitiveFields(&options);
  options.add_whitespace = true;
  std::string json;
  if (!google::protobuf::util::MessageToJsonString(*message, &json, options).ok()) {
    return;
  }
  if (QClipboard* clipboard = QApplication::clipboard()) {
    clipboard->setText(QString::fromStdString(json));
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
    last_payload_ = *payload;
    if (!freeze_) {
      showPayload(*payload);
    } else {
      refreshStatusChip();
    }
  } else if (!active_channel_.empty()) {
    refreshStatusChip();
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
  refreshStatusChip();
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
    item->setForeground(0, QColor(kTextMuted));
    refreshStatusChip();
  } else {
    refreshStatusChip();
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
  const QString path_filter = resolvedMessagePath();

  const bool structure_changed =
      message_tree_seeded_ &&
      (root_label != last_tree_root_label_ || path_filter != last_tree_path_filter_);
  const bool apply_initial_expand = !message_tree_seeded_;

  if (!structure_changed &&
      raw_messages::UpdateMessageTreeValues(message_tree_, message, root_label,
                                            path_filter, diff_highlight_)) {
    last_tree_root_label_ = root_label;
    last_tree_path_filter_ = path_filter;
    message_tree_seeded_ = true;
    refreshStatusChip();
    return;
  }

  raw_messages::PopulateMessageTree(message_tree_, message, root_label, channel,
                                    path_filter, apply_initial_expand);
  last_tree_root_label_ = root_label;
  last_tree_path_filter_ = path_filter;
  message_tree_seeded_ = true;
  refreshStatusChip();
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
        {tr("Failed to parse message"), ShortSchemaName(message_type),
         tr("%1 bytes").arg(static_cast<qulonglong>(payload.size()))});
    item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
    item->setForeground(0, QColor(kTextMuted));
    updateStatusChip(tr("Parse error"));
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
