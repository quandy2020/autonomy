/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/raw_messages_panel.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <algorithm>
#include <cstring>
#include <unordered_set>
#include <vector>

#include <QAbstractItemView>
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

#include "autolink/common/types.hpp"
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

/** Light tokens shared with Channels / Log / TF Tree. */
constexpr char kBg[] = "#f8f9fb";
constexpr char kSurface[] = "#ffffff";
constexpr char kBorder[] = "#cbd5e1";
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
  label->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 10px; font-weight: 700; letter-spacing: 0.04em;")
                           .arg(QLatin1String(kTextMuted)));
  return label;
}

QString ToolButtonStyle() {
  return QStringLiteral(
      "QToolButton {"
      "  color: %1; background: rgba(8,145,178,0.10);"
      "  border: 1px solid rgba(8,145,178,0.35); border-radius: 8px;"
      "  padding: 4px 10px; font-size: 12px; font-weight: 600;"
      "}"
      "QToolButton:hover { background: rgba(8,145,178,0.18); }"
      "QToolButton:pressed { background: rgba(8,145,178,0.26); }")
      .arg(QLatin1String(kAccent));
}

QString ComboStyle() {
  return QStringLiteral(
      "QComboBox {"
      "  background: %1; color: %2;"
      "  border: 1px solid %3; border-radius: 8px;"
      "  padding: 5px 10px; min-height: 28px;"
      "}"
      "QComboBox:hover { border-color: %4; }"
      "QComboBox:focus { border-color: %4; }"
      "QComboBox::drop-down {"
      "  subcontrol-origin: padding; subcontrol-position: top right;"
      "  width: 28px; border: none;"
      "}"
      "QComboBox QAbstractItemView {"
      "  background: %1; color: %2;"
      "  border: 1px solid %3; selection-background-color: rgba(8,145,178,0.16);"
      "}")
      .arg(QLatin1String(kSurface), QLatin1String(kText), QLatin1String(kBorder),
           QLatin1String(kAccent));
}

QString LineEditStyle() {
  return QStringLiteral(
      "QLineEdit {"
      "  background: %1; color: %2;"
      "  border: 1px solid %3; border-radius: 8px;"
      "  padding: 6px 10px; min-height: 26px;"
      "}"
      "QLineEdit:focus { border-color: %4; }")
      .arg(QLatin1String(kSurface), QLatin1String(kText), QLatin1String(kBorder),
           QLatin1String(kAccent));
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
  channel_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  channel_combo_->setStyleSheet(ComboStyle());
  channel_row->addWidget(channel_combo_, 1);

  auto* expand_button = new QToolButton(toolbar);
  expand_button->setText(tr("Expand"));
  expand_button->setToolTip(tr("Expand all message fields"));
  expand_button->setCursor(Qt::PointingHandCursor);
  expand_button->setStyleSheet(ToolButtonStyle());
  channel_row->addWidget(expand_button);

  auto* collapse_button = new QToolButton(toolbar);
  collapse_button->setText(tr("Collapse"));
  collapse_button->setToolTip(tr("Collapse all message fields"));
  collapse_button->setCursor(Qt::PointingHandCursor);
  collapse_button->setStyleSheet(ToolButtonStyle());
  channel_row->addWidget(collapse_button);

  status_label_ = new QLabel(toolbar);
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_label_->setStyleSheet(QStringLiteral(
      "color: %1; background: rgba(8,145,178,0.10);"
      "border: 1px solid rgba(8,145,178,0.28); border-radius: 10px;"
      "padding: 3px 10px; font-size: 11px; font-weight: 700;")
                                   .arg(QLatin1String(kAccent)));
  channel_row->addWidget(status_label_);
  toolbar_layout->addLayout(channel_row);

  auto* schema_row = new QHBoxLayout();
  schema_row->setSpacing(8);
  schema_badge_ = new QLabel(tr("SCHEMA"), toolbar);
  schema_badge_->setStyleSheet(QStringLiteral(
      "color: white; background: %1; border-radius: 9px;"
      "padding: 2px 8px; font-size: 10px; font-weight: 700;")
                                   .arg(QLatin1String(kAccent)));
  schema_row->addWidget(schema_badge_, 0, Qt::AlignVCenter);

  schema_label_ = new QLabel(toolbar);
  schema_label_->setWordWrap(true);
  schema_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  schema_label_->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 12px;").arg(QLatin1String(kText)));
  schema_row->addWidget(schema_label_, 1);
  toolbar_layout->addLayout(schema_row);

  auto* path_row = new QHBoxLayout();
  path_row->setSpacing(8);
  path_row->addWidget(MakeFieldCaption(tr("PATH"), toolbar));
  message_path_edit_ = new QLineEdit(toolbar);
  message_path_edit_->setPlaceholderText(
      tr("Optional message path, e.g. pose.pose.position.x"));
  message_path_edit_->setClearButtonEnabled(true);
  message_path_edit_->setStyleSheet(LineEditStyle());
  path_row->addWidget(message_path_edit_, 1);
  toolbar_layout->addLayout(path_row);

  auto* drag_hint = new QLabel(
      tr("Drop a channel here, or drag numeric fields to Plot."), toolbar);
  drag_hint->setStyleSheet(
      QStringLiteral("color: %1; font-size: 11px;").arg(QLatin1String(kTextMuted)));
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
      tr("Drag a numeric field to Plot, or right-click and choose Add to Plot."));
  message_tree_->header()->setStretchLastSection(true);
  message_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  message_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  message_tree_->header()->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  QFont mono = message_tree_->font();
  mono.setFamily(QStringLiteral("Monospace"));
  mono.setPointSizeF(std::max(10.0, mono.pointSizeF() - 0.5));
  message_tree_->setFont(mono);
  message_tree_->setStyleSheet(QStringLiteral(
      "QTreeWidget#RawMessagesTree {"
      "  background: %1; color: %2; border: none; outline: none;"
      "  font-size: 12px;"
      "}"
      "QTreeWidget#RawMessagesTree::item {"
      "  padding: 2px 6px; min-height: 24px;"
      "}"
      "QTreeWidget#RawMessagesTree::item:selected {"
      "  background: rgba(8,145,178,0.14); color: %2;"
      "}"
      "QTreeWidget#RawMessagesTree::item:hover:!selected {"
      "  background: rgba(15,23,42,0.04);"
      "}"
      "QTreeWidget#RawMessagesTree::branch {"
      "  background: transparent;"
      "}"
      "QHeaderView::section {"
      "  background: %3; color: %4;"
      "  border: none; border-bottom: 1px solid %5;"
      "  padding: 7px 8px;"
      "  font-size: 11px; font-weight: 700;"
      "}"
      "QScrollBar:vertical {"
      "  background: %3; width: 10px; margin: 0;"
      "}"
      "QScrollBar::handle:vertical {"
      "  background: #94a3b8; border-radius: 5px; min-height: 24px;"
      "}"
      "QScrollBar::handle:vertical:hover { background: %6; }"
      "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
      "  height: 0;"
      "}")
                                   .arg(QLatin1String(kSurface), QLatin1String(kText),
                                        QLatin1String(kBg), QLatin1String(kTextMuted),
                                        QLatin1String(kBorder), QLatin1String(kAccent)));
  tree_card_layout->addWidget(message_tree_, 1);

  empty_hint_ = new QFrame(tree_card);
  empty_hint_->setObjectName(QStringLiteral("RawMessagesEmpty"));
  auto* empty_layout = new QVBoxLayout(empty_hint_);
  empty_layout->setContentsMargins(24, 40, 24, 40);
  empty_layout->setAlignment(Qt::AlignCenter);
  auto* empty_title = new QLabel(tr("Select a channel"), empty_hint_);
  empty_title->setAlignment(Qt::AlignCenter);
  empty_title->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 15px; font-weight: 600;").arg(QLatin1String(kText)));
  empty_layout->addWidget(empty_title);
  auto* empty_body = new QLabel(
      tr("Choose a pub/sub channel above, or drag one from the Channels panel."),
      empty_hint_);
  empty_body->setAlignment(Qt::AlignCenter);
  empty_body->setWordWrap(true);
  empty_body->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 12px;").arg(QLatin1String(kTextMuted)));
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
      QStringLiteral("color: %1; font-size: 11px;").arg(QLatin1String(kTextMuted)));
  footer_layout->addWidget(footer_label);
  layout->addWidget(footer);

  connect(message_tree_, &raw_messages::RawMessageTreeWidget::addToPlotRequested, this,
          &RawMessagesPanel::addToPlotRequested);
  connect(channel_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &RawMessagesPanel::onChannelChanged);
  connect(message_path_edit_, &QLineEdit::editingFinished, this,
          &RawMessagesPanel::onMessagePathEdited);
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

  if (manager_ != nullptr) {
    connect(&manager_->variableStore(),
            &variables::VariableStore::variablesChanged, this,
            &RawMessagesPanel::refreshFromVariables);
  }

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &RawMessagesPanel::onTick);
  tick_timer_->start(50);

  updateSchemaHeader();
  updateStatusChip(tr("Idle"));
  refreshChannels();
}

void RawMessagesPanel::applyChromeStyles() {
  setStyleSheet(QStringLiteral(
      "QWidget#RawMessagesPanelContent {"
      "  background: %1; color: %2;"
      "}"
      "QFrame#RawMessagesToolbar {"
      "  background: %3;"
      "  border-bottom: 1px solid %4;"
      "}"
      "QFrame#RawMessagesTreeCard, QFrame#RawMessagesEmpty {"
      "  background: %3;"
      "  border: none;"
      "}"
      "QFrame#RawMessagesFooter {"
      "  background: %1;"
      "  border-top: 1px solid %4;"
      "}")
                    .arg(QLatin1String(kBg), QLatin1String(kText),
                         QLatin1String(kSurface), QLatin1String(kBorder)));
}

void RawMessagesPanel::updateStatusChip(const QString& text) {
  if (status_label_ == nullptr) {
    return;
  }
  status_label_->setText(text);
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
    schema_label_->setStyleSheet(QStringLiteral(
        "color: %1; font-size: 12px;").arg(QLatin1String(kTextMuted)));
    updateStatusChip(tr("Idle"));
    return;
  }

  const QString channel = QString::fromStdString(active_channel_);
  const QString schema = ShortSchemaName(messageTypeForChannel(active_channel_));
  schema_label_->setText(QStringLiteral("%1  ·  %2").arg(channel, schema));
  schema_label_->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 12px; font-weight: 600;").arg(QLatin1String(kText)));
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
  updateStatusChip(tr("Idle"));
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
    updateStatusChip(tr("Error"));
  } else {
    updateStatusChip(tr("Live"));
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
    updateStatusChip(tr("Live"));
    return;
  }

  raw_messages::PopulateMessageTree(message_tree_, message, root_label, channel,
                                    path_filter, apply_initial_expand);
  last_tree_root_label_ = root_label;
  last_tree_path_filter_ = path_filter;
  message_tree_seeded_ = true;
  updateStatusChip(tr("Live"));
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
