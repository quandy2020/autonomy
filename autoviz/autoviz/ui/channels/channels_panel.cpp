/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channels/channels_panel.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include <QAbstractItemView>
#include <QApplication>
#include <QClipboard>
#include <QColor>
#include <QDrag>
#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMimeData>
#include <QPushButton>
#include <QSet>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/channel_stats_registry.hpp"
#include "autoviz/ui/map/map_message_ingest.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_field_extractor.hpp"
#include "autoviz/ui/plot/plot_path_utils.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

constexpr int kColumnName = 0;
constexpr int kColumnSchema = 1;
constexpr int kColumnHz = 2;
constexpr int kColumnCount = 3;
constexpr int kColumnValue = 4;

/** Cap concurrent probe subscriptions to avoid bandwidth spikes. */
constexpr int kMaxProbeChannels = 48;

/** Light tokens shared with Log / TF Tree / Teleop. */
constexpr char kText[] = "#1e293b";
constexpr char kTextMuted[] = "#64748b";
constexpr char kAccent[] = "#0891b2";

constexpr int kRoleMessageType = Qt::UserRole + 20;
constexpr int kRoleFieldsPopulated = Qt::UserRole + 21;

QString ShortSchemaName(const QString& message_type) {
  QString schema = message_type.trimmed();
  if (schema.startsWith(QLatin1String("automsgs.msgs."))) {
    schema = schema.mid(14);
  }
  return schema;
}

QString FormatFrequency(double hz) {
  if (hz <= 0.0) {
    return QStringLiteral("—");
  }
  if (hz >= 100.0) {
    return QString::number(hz, 'f', 0);
  }
  if (hz >= 10.0) {
    return QString::number(hz, 'f', 1);
  }
  return QString::number(hz, 'f', 2);
}

QString FormatLastValue(double value) {
  const double abs_v = std::abs(value);
  if (abs_v >= 1000.0 || (abs_v > 0.0 && abs_v < 1e-3)) {
    return QString::number(value, 'g', 5);
  }
  return QString::number(value, 'f', 4);
}

bool IsTfMessageType(const QString& message_type) {
  const QString lower = message_type.toLower();
  return lower.contains(QLatin1String("tfmessage")) ||
         lower.contains(QLatin1String("tf2_msgs")) ||
         lower.contains(QLatin1String("transformstamped"));
}

bool IsNumericMessageType(const std::string& message_type) {
  return !plot::NumericFieldPathsForMessageType(message_type).isEmpty();
}

bool ItemMatchesFilter(QTreeWidgetItem* item, const QString& needle) {
  if (item == nullptr || needle.isEmpty()) {
    return true;
  }
  const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
  const QString schema = item->data(kColumnName, kRoleMessageType).toString();
  const QString field_path = item->data(kColumnName, plot::kTopicFieldPathRole).toString();
  const QString haystack = QStringLiteral("%1 %2 %3 %4")
                               .arg(item->text(kColumnName), channel, schema, field_path);
  return haystack.contains(needle, Qt::CaseInsensitive);
}

void ApplyFilterRecursive(QTreeWidgetItem* item, const QString& needle,
                          const std::function<bool(QTreeWidgetItem*)>& type_ok) {
  if (item == nullptr) {
    return;
  }
  bool child_visible = false;
  for (int i = 0; i < item->childCount(); ++i) {
    ApplyFilterRecursive(item->child(i), needle, type_ok);
    if (!item->child(i)->isHidden()) {
      child_visible = true;
    }
  }
  const bool self_match = ItemMatchesFilter(item, needle);
  const bool type_match = type_ok(item);
  const bool hide =
      (!needle.isEmpty() && !self_match && !child_visible) ||
      (!type_match && !child_visible);
  item->setHidden(hide);
}

bool IsDraggableItem(const QTreeWidgetItem* item) {
  if (item == nullptr) {
    return false;
  }
  return item->data(kColumnName, plot::kTopicDraggableRole).toBool() ||
         item->data(kColumnName, plot::kTopicTableDraggableRole).toBool() ||
         item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool() ||
         item->data(kColumnName, plot::kTopicImageDraggableRole).toBool() ||
         item->data(kColumnName, plot::kTopicMapDraggableRole).toBool();
}

QWidget* MakeTypeFilterChips(QWidget* parent, int checked_index,
                             const std::function<void(int)>& on_changed) {
  auto* container = new QWidget(parent);
  container->setObjectName(QStringLiteral("plot_segmented_toggle"));
  container->setAttribute(Qt::WA_StyledBackground, true);
  auto* layout = new QHBoxLayout(container);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  container->setFixedHeight(24);
  container->setStyleSheet(
      style::sheet(QStringLiteral("plot/segmented_toggle_inline"),
                   style::Tone::Frost));

  const QStringList labels = {QObject::tr("All"), QObject::tr("Numeric"),
                              QObject::tr("Image"), QObject::tr("Geo"),
                              QObject::tr("TF")};
  for (int i = 0; i < labels.size(); ++i) {
    auto* button = new QPushButton(labels.at(i), container);
    button->setCheckable(true);
    button->setChecked(i == checked_index);
    button->setAutoExclusive(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setFlat(true);
    button->setProperty("channelsTypeFilterIndex", i);
    layout->addWidget(button, 1);
    QObject::connect(button, &QPushButton::clicked, container, [on_changed, i]() {
      on_changed(i);
    });
  }
  return container;
}

class ChannelsTreeWidget : public QTreeWidget {
 public:
  using QTreeWidget::QTreeWidget;

 protected:
  void startDrag(Qt::DropActions supported_actions) override {
    QVector<plot::PlotSeriesDragPayload> payloads;
    const QList<QTreeWidgetItem*> selected = selectedItems();
    if (!selected.isEmpty()) {
      for (QTreeWidgetItem* item : selected) {
        if (!IsDraggableItem(item)) {
          continue;
        }
        plot::PlotSeriesDragPayload payload;
        payload.channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
        payload.field_path = item->data(kColumnName, plot::kTopicFieldPathRole).toString();
        if (!payload.channel.isEmpty()) {
          payloads.push_back(payload);
        }
      }
    } else if (QTreeWidgetItem* item = currentItem()) {
      if (IsDraggableItem(item)) {
        plot::PlotSeriesDragPayload payload;
        payload.channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
        payload.field_path = item->data(kColumnName, plot::kTopicFieldPathRole).toString();
        if (!payload.channel.isEmpty()) {
          payloads.push_back(payload);
        }
      }
    }
    if (payloads.isEmpty()) {
      return;
    }
    auto* drag = new QDrag(this);
    drag->setMimeData(plot::MakePlotSeriesListDragPayload(payloads));
    drag->exec(supported_actions, Qt::CopyAction);
  }
};

}  // namespace

ChannelsPanel::ChannelsPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), QWidget(parent) {
  ApplyPanelShell(this);
  setObjectName(QStringLiteral("ChannelsPanelContent"));
  applyChromeStyles();

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto* toolbar = new QFrame(this);
  toolbar->setObjectName(QStringLiteral("ChannelsToolbar"));
  auto* toolbar_layout = new QVBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(10, 8, 10, 8);
  toolbar_layout->setSpacing(6);

  auto* filter_row = new QHBoxLayout();
  filter_row->setSpacing(8);

  auto* filter_icon = new QLabel(QStringLiteral("⌕"), toolbar);
  filter_icon->setStyleSheet(style::type(style::Role::PanelMuted, 14));
  filter_icon->setFixedWidth(16);
  filter_row->addWidget(filter_icon);

  filter_edit_ = new QLineEdit(toolbar);
  filter_edit_->setObjectName(QStringLiteral("AutovizCyanLineEdit"));
  filter_edit_->setPlaceholderText(tr("Search channels…"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setStyleSheet(
      style::sheet(QStringLiteral("widget/fields"), style::Tone::Frost));
  filter_row->addWidget(filter_edit_, 1);

  auto* expand_button = new QToolButton(toolbar);
  expand_button->setObjectName(QStringLiteral("AutovizCyanToolButton"));
  expand_button->setText(tr("Expand"));
  expand_button->setToolTip(tr("Expand all channel trees"));
  expand_button->setCursor(Qt::PointingHandCursor);
  expand_button->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  filter_row->addWidget(expand_button);

  auto* collapse_button = new QToolButton(toolbar);
  collapse_button->setObjectName(QStringLiteral("AutovizCyanToolButton"));
  collapse_button->setText(tr("Collapse"));
  collapse_button->setToolTip(tr("Collapse all channel trees"));
  collapse_button->setCursor(Qt::PointingHandCursor);
  collapse_button->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  filter_row->addWidget(collapse_button);

  probe_button_ = new QToolButton(toolbar);
  probe_button_->setObjectName(QStringLiteral("AutovizCyanToolButton"));
  probe_button_->setText(tr("Probe"));
  probe_button_->setCheckable(true);
  probe_button_->setChecked(true);
  probe_button_->setToolTip(
      tr("Lightweight subscribe visible channels for Hz / last value "
         "(off = only already-subscribed traffic)"));
  probe_button_->setCursor(Qt::PointingHandCursor);
  probe_button_->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  filter_row->addWidget(probe_button_);

  status_label_ = new QLabel(toolbar);
  status_label_->setObjectName(QStringLiteral("AutovizCyanStatusChip"));
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_label_->setStyleSheet(
      style::sheet(QStringLiteral("widget/badges"), style::Tone::Frost));
  filter_row->addWidget(status_label_);
  toolbar_layout->addLayout(filter_row);

  auto* chips_row = new QHBoxLayout();
  chips_row->setSpacing(8);
  type_filter_chips_ = MakeTypeFilterChips(
      toolbar, static_cast<int>(TypeFilter::kAll), [this](int index) {
        setTypeFilterIndex(index);
      });
  chips_row->addWidget(type_filter_chips_, 1);
  toolbar_layout->addLayout(chips_row);

  auto* hint = new QLabel(
      tr("Drag a channel or field onto Plot / Table / Messages / Image / Map. "
         "Right-click for Copy / Open / Plot."),
      toolbar);
  hint->setWordWrap(true);
  hint->setStyleSheet(style::type(style::Role::PanelMuted, 11));
  toolbar_layout->addWidget(hint);
  layout->addWidget(toolbar);

  auto* tree_card = new QFrame(this);
  tree_card->setObjectName(QStringLiteral("ChannelsTreeCard"));
  auto* tree_card_layout = new QVBoxLayout(tree_card);
  tree_card_layout->setContentsMargins(0, 0, 0, 0);
  tree_card_layout->setSpacing(0);

  tree_ = new ChannelsTreeWidget(tree_card);
  tree_->setObjectName(QStringLiteral("ChannelsTree"));
  tree_->setColumnCount(5);
  tree_->setHeaderLabels(
      {tr("Channel"), tr("Schema"), tr("Hz"), tr("Count"), tr("Value")});
  tree_->header()->setStretchLastSection(false);
  tree_->header()->setSectionResizeMode(kColumnName, QHeaderView::Stretch);
  tree_->header()->setSectionResizeMode(kColumnSchema, QHeaderView::ResizeToContents);
  tree_->header()->setSectionResizeMode(kColumnHz, QHeaderView::ResizeToContents);
  tree_->header()->setSectionResizeMode(kColumnCount, QHeaderView::ResizeToContents);
  tree_->header()->setSectionResizeMode(kColumnValue, QHeaderView::ResizeToContents);
  tree_->header()->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  tree_->setRootIsDecorated(true);
  tree_->setUniformRowHeights(true);
  tree_->setAnimated(true);
  tree_->setIndentation(18);
  tree_->setDragEnabled(true);
  tree_->setDragDropMode(QAbstractItemView::DragOnly);
  tree_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  tree_->setExpandsOnDoubleClick(true);
  tree_->setAlternatingRowColors(false);
  tree_->setContextMenuPolicy(Qt::CustomContextMenu);
  tree_->setStyleSheet(
      style::sheet(QStringLiteral("channels"), style::Tone::Frost));
  tree_card_layout->addWidget(tree_, 1);

  empty_hint_ = new QLabel(
      tr("No channels yet.\nPublish or connect a node to populate this list."),
      tree_card);
  empty_hint_->setAlignment(Qt::AlignCenter);
  empty_hint_->setWordWrap(true);
  empty_hint_->setStyleSheet(
      style::sheet(QStringLiteral("channels"), style::Tone::Frost));
  empty_hint_->hide();
  tree_card_layout->addWidget(empty_hint_, 1);
  layout->addWidget(tree_card, 1);

  auto* footer = new QFrame(this);
  footer->setObjectName(QStringLiteral("ChannelsFooter"));
  auto* footer_layout = new QHBoxLayout(footer);
  footer_layout->setContentsMargins(12, 6, 12, 6);
  auto* footer_label = new QLabel(
      tr("Expand a channel to browse message fields"), footer);
  footer_label->setStyleSheet(style::type(style::Role::PanelMuted, 11));
  footer_layout->addWidget(footer_label);
  layout->addWidget(footer);

  connect(filter_edit_, &QLineEdit::textChanged, this, [this](const QString&) {
    applyFilter();
    syncStatsProbes();
  });
  connect(tree_, &QTreeWidget::itemExpanded, this, &ChannelsPanel::onItemExpanded);
  connect(tree_, &QTreeWidget::customContextMenuRequested, this,
          &ChannelsPanel::onCustomContextMenu);
  connect(tree_, &QTreeWidget::itemDoubleClicked, this,
          &ChannelsPanel::onItemDoubleClicked);
  connect(expand_button, &QToolButton::clicked, this, [this]() {
    if (tree_ != nullptr) {
      tree_->expandAll();
      syncStatsProbes();
    }
  });
  connect(collapse_button, &QToolButton::clicked, this, [this]() {
    if (tree_ != nullptr) {
      tree_->collapseAll();
      syncStatsProbes();
    }
  });
  connect(probe_button_, &QToolButton::toggled, this,
          &ChannelsPanel::setProbeEnabled);

  stats_timer_ = new QTimer(this);
  stats_timer_->setInterval(500);
  connect(stats_timer_, &QTimer::timeout, this, &ChannelsPanel::refreshStats);
  stats_timer_->start();

  rebuildTree();
}

ChannelsPanel::~ChannelsPanel() { clearStatsProbes(); }

void ChannelsPanel::applyChromeStyles() {
  setStyleSheet(style::sheet(QStringLiteral("channels"), style::Tone::Frost));
}

void ChannelsPanel::styleChannelItem(QTreeWidgetItem* item,
                                     bool is_channel_leaf) const {
  if (item == nullptr) {
    return;
  }
  QFont name_font = item->font(kColumnName);
  name_font.setBold(is_channel_leaf);
  item->setFont(kColumnName, name_font);
  item->setForeground(kColumnName,
                      QColor(is_channel_leaf ? kText : kTextMuted));
  item->setForeground(kColumnSchema, QColor(kTextMuted));
  item->setForeground(kColumnHz, QColor(kTextMuted));
  item->setForeground(kColumnCount, QColor(kTextMuted));
  item->setForeground(kColumnValue, QColor(kTextMuted));
}

void ChannelsPanel::updateStatusChip() {
  if (status_label_ == nullptr) {
    return;
  }
  const int count = cached_channel_keys_.size();
  const int probing = static_cast<int>(probe_subscriptions_.size());
  if (probe_enabled_ && probing > 0) {
    status_label_->setText(count == 1
                               ? tr("1 channel · probing %1").arg(probing)
                               : tr("%1 channels · probing %2")
                                     .arg(count)
                                     .arg(probing));
  } else {
    status_label_->setText(count == 1 ? tr("1 channel")
                                      : tr("%1 channels").arg(count));
  }
  if (empty_hint_ != nullptr && tree_ != nullptr) {
    const bool empty = count == 0;
    empty_hint_->setVisible(empty);
    tree_->setVisible(!empty);
  }
}

void ChannelsPanel::refreshChannels() {
  if (channelsStructureChanged()) {
    rebuildTree();
  } else {
    refreshStats();
  }
}

void ChannelsPanel::refreshStats() {
  updateChannelStatsColumns();
  syncStatsProbes();
  updateStatusChip();
}

common::ChannelsBrowserPersistConfig ChannelsPanel::config() const {
  common::ChannelsBrowserPersistConfig out;
  out.filter_text =
      filter_edit_ != nullptr ? filter_edit_->text().toStdString() : std::string();
  out.type_filter = static_cast<int>(type_filter_);
  out.probe_enabled = probe_enabled_;
  for (const QString& channel : expandedChannels()) {
    out.expanded_channels.push_back(channel.toStdString());
  }
  return out;
}

void ChannelsPanel::setConfig(const common::ChannelsBrowserPersistConfig& config) {
  type_filter_ = static_cast<TypeFilter>(std::clamp(config.type_filter, 0, 4));
  probe_enabled_ = config.probe_enabled;
  if (probe_button_ != nullptr) {
    probe_button_->blockSignals(true);
    probe_button_->setChecked(probe_enabled_);
    probe_button_->blockSignals(false);
  }
  if (filter_edit_ != nullptr) {
    filter_edit_->blockSignals(true);
    filter_edit_->setText(QString::fromStdString(config.filter_text));
    filter_edit_->blockSignals(false);
  }
  syncTypeFilterChips();
  applyFilter();
  if (!probe_enabled_) {
    clearStatsProbes();
  } else {
    syncStatsProbes();
  }
  QStringList expanded;
  expanded.reserve(static_cast<int>(config.expanded_channels.size()));
  for (const std::string& channel : config.expanded_channels) {
    expanded.push_back(QString::fromStdString(channel));
  }
  expandChannels(expanded);
  updateChannelStatsColumns();
  updateStatusChip();
}

bool ChannelsPanel::channelsStructureChanged() const {
  if (manager_ == nullptr) {
    return !cached_channel_keys_.isEmpty();
  }
  QStringList current;
  current.reserve(static_cast<int>(manager_->channels().size()));
  for (const integration::ChannelInfo& channel : manager_->channels()) {
    current.push_back(QStringLiteral("%1|%2")
                          .arg(QString::fromStdString(channel.channel_name),
                               QString::fromStdString(channel.message_type)));
  }
  current.sort(Qt::CaseInsensitive);
  return current != cached_channel_keys_;
}

void ChannelsPanel::rebuildTree() {
  if (tree_ == nullptr) {
    return;
  }
  clearStatsProbes();
  tree_->clear();
  cached_channel_keys_.clear();
  if (manager_ == nullptr) {
    updateStatusChip();
    return;
  }

  std::map<QString, QTreeWidgetItem*> path_nodes;
  for (const integration::ChannelInfo& channel : manager_->channels()) {
    const QString channel_q = QString::fromStdString(channel.channel_name);
    if (channel_q.isEmpty()) {
      continue;
    }
    const QString message_type = QString::fromStdString(channel.message_type);
    cached_channel_keys_.push_back(
        QStringLiteral("%1|%2").arg(channel_q, message_type));

    const QStringList segments =
        channel_q.split(QLatin1Char('/'), Qt::SkipEmptyParts);
    QTreeWidgetItem* parent = nullptr;
    QString built_path;
    for (const QString& segment : segments) {
      built_path += QLatin1Char('/') + segment;
      const auto found = path_nodes.find(built_path);
      QTreeWidgetItem* node = found == path_nodes.end() ? nullptr : found->second;
      if (node == nullptr) {
        node = parent == nullptr ? new QTreeWidgetItem(tree_)
                                 : new QTreeWidgetItem(parent);
        node->setText(kColumnName, segment);
        node->setData(kColumnName, plot::kTopicDraggableRole, false);
        node->setData(kColumnName, plot::kTopicChannelDraggableRole, false);
        node->setData(kColumnName, plot::kTopicImageDraggableRole, false);
        node->setData(kColumnName, plot::kTopicMapDraggableRole, false);
        styleChannelItem(node, false);
        path_nodes.emplace(built_path, node);
      }
      parent = node;
    }
    if (parent == nullptr) {
      continue;
    }

    parent->setData(kColumnName, plot::kTopicChannelRole, channel_q);
    parent->setData(kColumnName, plot::kTopicFieldPathRole, QString());
    parent->setData(kColumnName, kRoleMessageType, message_type);
    parent->setToolTip(kColumnName,
                       QStringLiteral("%1\n%2").arg(channel_q, message_type));
    parent->setText(kColumnSchema, ShortSchemaName(message_type));
    parent->setData(kColumnName, plot::kTopicChannelDraggableRole, true);
    parent->setData(kColumnName, plot::kTopicImageDraggableRole,
                    display::isImageMessageType(channel.message_type));
    parent->setData(kColumnName, plot::kTopicMapDraggableRole,
                    map::MapMessageIngest::SupportsMessageType(message_type));
    parent->setFlags(parent->flags() | Qt::ItemIsDragEnabled);
    styleChannelItem(parent, true);
  }

  cached_channel_keys_.sort(Qt::CaseInsensitive);
  tree_->sortItems(kColumnName, Qt::AscendingOrder);
  applyFilter();
  updateChannelStatsColumns();
  syncStatsProbes();
  updateStatusChip();
}

void ChannelsPanel::updateChannelStatsColumns() {
  if (tree_ == nullptr) {
    return;
  }

  std::unordered_map<std::string, std::string> payloads;
  std::unordered_map<std::string, std::string> types;
  {
    std::lock_guard<std::mutex> lock(probe_mutex_);
    payloads = last_payloads_;
    types = last_message_types_;
  }

  std::function<void(QTreeWidgetItem*)> visit;
  visit = [&](QTreeWidgetItem* item) {
    if (item == nullptr) {
      return;
    }
    const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
    const QString field_path =
        item->data(kColumnName, plot::kTopicFieldPathRole).toString();
    const bool is_channel_leaf =
        !channel.isEmpty() &&
        item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool();
    const bool is_numeric_field =
        item->data(kColumnName, plot::kTopicDraggableRole).toBool();

    if (is_channel_leaf) {
      const integration::ChannelStats stats =
          integration::ChannelStatsRegistry::instance().stats(
              channel.toStdString());
      const bool live = stats.frequency_hz > 0.0;
      item->setText(kColumnHz, FormatFrequency(stats.frequency_hz));
      item->setText(kColumnCount, stats.message_count > 0
                                     ? QString::number(stats.message_count)
                                     : QStringLiteral("—"));
      item->setText(kColumnValue, QStringLiteral("—"));
      item->setForeground(kColumnHz, QColor(live ? kAccent : kTextMuted));
      item->setForeground(kColumnCount,
                          QColor(stats.message_count > 0 ? kText : kTextMuted));
      QFont hz_font = item->font(kColumnHz);
      hz_font.setBold(live);
      item->setFont(kColumnHz, hz_font);
      if (!live && probe_enabled_) {
        item->setToolTip(kColumnHz, tr("No traffic yet (probe active)"));
      } else if (!live) {
        item->setToolTip(
            kColumnHz,
            tr("No stats — enable Probe or open in a panel that subscribes"));
      } else {
        item->setToolTip(kColumnHz, QString());
      }
    } else if (is_numeric_field && !channel.isEmpty() && !field_path.isEmpty()) {
      item->setText(kColumnHz, QString());
      item->setText(kColumnCount, QString());
      const std::string channel_key = channel.toStdString();
      const auto payload_it = payloads.find(channel_key);
      const auto type_it = types.find(channel_key);
      if (payload_it != payloads.end() && type_it != types.end()) {
        const auto value = plot::PlotFieldExtractor::instance().extractNumeric(
            type_it->second, payload_it->second, field_path.toStdString());
        if (value.has_value()) {
          item->setText(kColumnValue, FormatLastValue(*value));
          item->setForeground(kColumnValue, QColor(kAccent));
          item->setToolTip(
              kColumnValue,
              plot::CombinedPlotValuePath(channel, field_path));
        } else {
          item->setText(kColumnValue, QStringLiteral("—"));
          item->setForeground(kColumnValue, QColor(kTextMuted));
        }
      } else {
        item->setText(kColumnValue, QStringLiteral("—"));
        item->setForeground(kColumnValue, QColor(kTextMuted));
      }
    }

    for (int i = 0; i < item->childCount(); ++i) {
      visit(item->child(i));
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    visit(tree_->topLevelItem(i));
  }
}

bool ChannelsPanel::matchesTypeFilter(const QString& message_type) const {
  if (type_filter_ == TypeFilter::kAll) {
    return true;
  }
  if (message_type.isEmpty()) {
    return false;
  }
  const std::string type_std = message_type.toStdString();
  switch (type_filter_) {
    case TypeFilter::kNumeric:
      return IsNumericMessageType(type_std);
    case TypeFilter::kImage:
      return display::isImageMessageType(type_std);
    case TypeFilter::kGeo:
      return map::MapMessageIngest::SupportsMessageType(message_type);
    case TypeFilter::kTf:
      return IsTfMessageType(message_type);
    case TypeFilter::kAll:
    default:
      return true;
  }
}

void ChannelsPanel::applyFilter() {
  if (tree_ == nullptr) {
    return;
  }
  const QString needle =
      filter_edit_ != nullptr ? filter_edit_->text().trimmed() : QString();
  auto type_ok = [this](QTreeWidgetItem* item) -> bool {
    if (type_filter_ == TypeFilter::kAll) {
      return true;
    }
    // Path groups (no schema) stay visible only when a child matches.
    return matchesTypeFilter(
        item->data(kColumnName, kRoleMessageType).toString());
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    ApplyFilterRecursive(tree_->topLevelItem(i), needle, type_ok);
  }
}

void ChannelsPanel::onItemExpanded(QTreeWidgetItem* item) {
  if (item == nullptr || item->data(kColumnName, kRoleFieldsPopulated).toBool()) {
    return;
  }
  const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
  const QString message_type = item->data(kColumnName, kRoleMessageType).toString();
  if (channel.isEmpty() || message_type.isEmpty()) {
    return;
  }
  // Numeric leaves for Plot + repeated arrays for Table.
  plot::PopulateMessageFieldTree(item, message_type.toStdString(), QString());
  plot::PopulateTableArrayFieldTree(item, message_type.toStdString(), QString());
  std::function<void(QTreeWidgetItem*)> set_channel;
  set_channel = [&](QTreeWidgetItem* node) {
    if (node == nullptr) {
      return;
    }
    if (node->data(kColumnName, plot::kTopicDraggableRole).toBool() ||
        node->data(kColumnName, plot::kTopicTableDraggableRole).toBool()) {
      node->setData(kColumnName, plot::kTopicChannelRole, channel);
      node->setForeground(kColumnName, QColor(kTextMuted));
      const QString field =
          node->data(kColumnName, plot::kTopicFieldPathRole).toString();
      node->setToolTip(
          kColumnName, plot::CombinedPlotValuePath(channel, field));
    }
    for (int i = 0; i < node->childCount(); ++i) {
      set_channel(node->child(i));
    }
  };
  set_channel(item);
  item->setData(kColumnName, kRoleFieldsPopulated, true);
  syncStatsProbes();
}

void ChannelsPanel::onCustomContextMenu(const QPoint& pos) {
  if (tree_ == nullptr) {
    return;
  }
  QTreeWidgetItem* item = tree_->itemAt(pos);
  if (item == nullptr) {
    return;
  }
  const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
  const QString field_path =
      item->data(kColumnName, plot::kTopicFieldPathRole).toString();
  if (channel.isEmpty()) {
    return;
  }

  const QString combined = plot::CombinedPlotValuePath(channel, field_path);
  const bool is_channel =
      item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool();
  const bool is_plottable =
      item->data(kColumnName, plot::kTopicDraggableRole).toBool() &&
      !field_path.isEmpty();

  QMenu menu(this);
  QAction* copy_action =
      menu.addAction(field_path.isEmpty() ? tr("Copy channel path")
                                          : tr("Copy field path"));
  QAction* open_raw_action = nullptr;
  if (is_channel || !channel.isEmpty()) {
    open_raw_action = menu.addAction(tr("Open in Raw Messages"));
  }
  QAction* add_plot_action = nullptr;
  if (is_plottable) {
    add_plot_action = menu.addAction(tr("Add to Plot"));
  } else if (is_channel) {
    // Offer first numeric leaf when expanding would be heavy — list paths.
    const QString message_type =
        item->data(kColumnName, kRoleMessageType).toString();
    const QStringList numeric =
        plot::NumericFieldPathsForMessageType(message_type.toStdString());
    if (!numeric.isEmpty()) {
      add_plot_action =
          menu.addAction(tr("Add to Plot (%1)").arg(numeric.first()));
      add_plot_action->setData(numeric.first());
    }
  }

  QAction* chosen = menu.exec(tree_->viewport()->mapToGlobal(pos));
  if (chosen == nullptr) {
    return;
  }
  if (chosen == copy_action) {
    if (QClipboard* clipboard = QApplication::clipboard()) {
      clipboard->setText(combined);
    }
    return;
  }
  if (chosen == open_raw_action) {
    emit openInRawMessagesRequested(channel);
    return;
  }
  if (chosen == add_plot_action) {
    QString path = field_path;
    if (path.isEmpty()) {
      path = chosen->data().toString();
    }
    if (!path.isEmpty()) {
      emit addToPlotRequested(channel, path);
    }
  }
}

void ChannelsPanel::setProbeEnabled(bool enabled) {
  probe_enabled_ = enabled;
  if (!probe_enabled_) {
    clearStatsProbes();
  } else {
    syncStatsProbes();
  }
  updateChannelStatsColumns();
  updateStatusChip();
}

void ChannelsPanel::setTypeFilterIndex(int index) {
  type_filter_ = static_cast<TypeFilter>(std::clamp(index, 0, 4));
  applyFilter();
  syncStatsProbes();
}

void ChannelsPanel::onItemDoubleClicked(QTreeWidgetItem* item, int /*column*/) {
  if (item == nullptr) {
    return;
  }
  const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
  if (channel.isEmpty()) {
    return;
  }
  // Field leaves keep their channel role; only open Raw for channel roots.
  if (item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool()) {
    emit openInRawMessagesRequested(channel);
  }
}

QStringList ChannelsPanel::expandedChannels() const {
  QStringList out;
  if (tree_ == nullptr) {
    return out;
  }
  std::function<void(QTreeWidgetItem*)> visit;
  visit = [&](QTreeWidgetItem* item) {
    if (item == nullptr) {
      return;
    }
    if (item->isExpanded() &&
        item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool()) {
      const QString channel =
          item->data(kColumnName, plot::kTopicChannelRole).toString();
      if (!channel.isEmpty()) {
        out.push_back(channel);
      }
    }
    for (int i = 0; i < item->childCount(); ++i) {
      visit(item->child(i));
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    visit(tree_->topLevelItem(i));
  }
  return out;
}

void ChannelsPanel::expandChannels(const QStringList& channels) {
  if (tree_ == nullptr || channels.isEmpty()) {
    return;
  }
  QSet<QString> wanted;
  for (const QString& channel : channels) {
    wanted.insert(channel);
  }
  std::function<void(QTreeWidgetItem*)> visit;
  visit = [&](QTreeWidgetItem* item) {
    if (item == nullptr) {
      return;
    }
    if (item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool()) {
      const QString channel =
          item->data(kColumnName, plot::kTopicChannelRole).toString();
      if (wanted.contains(channel)) {
        // Expand ancestors so the leaf is reachable.
        QTreeWidgetItem* ancestor = item->parent();
        while (ancestor != nullptr) {
          ancestor->setExpanded(true);
          ancestor = ancestor->parent();
        }
        item->setExpanded(true);
      }
    }
    for (int i = 0; i < item->childCount(); ++i) {
      visit(item->child(i));
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    visit(tree_->topLevelItem(i));
  }
}

void ChannelsPanel::syncTypeFilterChips() const {
  if (type_filter_chips_ == nullptr) {
    return;
  }
  const int wanted = static_cast<int>(type_filter_);
  const QList<QPushButton*> buttons =
      type_filter_chips_->findChildren<QPushButton*>();
  for (QPushButton* button : buttons) {
    if (button == nullptr) {
      continue;
    }
    const int index = button->property("channelsTypeFilterIndex").toInt();
    button->blockSignals(true);
    button->setChecked(index == wanted);
    button->blockSignals(false);
  }
}

void ChannelsPanel::storeProbePayload(const std::string& channel,
                                      const std::string& message_type,
                                      const std::string& payload) {
  std::lock_guard<std::mutex> lock(probe_mutex_);
  last_payloads_[channel] = payload;
  last_message_types_[channel] = message_type;
}

void ChannelsPanel::clearStatsProbes() {
  for (const auto& entry : probe_subscriptions_) {
    integration::ChannelReaderRegistry::instance().unsubscribe(entry.second);
  }
  probe_subscriptions_.clear();
  {
    std::lock_guard<std::mutex> lock(probe_mutex_);
    last_payloads_.clear();
    last_message_types_.clear();
  }
}

void ChannelsPanel::syncStatsProbes() {
  if (!probe_enabled_ || tree_ == nullptr) {
    return;
  }

  // Collect visible channel leaves (prefer expanded / filtered-visible).
  std::vector<std::pair<std::string, std::string>> wanted;
  wanted.reserve(static_cast<std::size_t>(kMaxProbeChannels));

  std::function<void(QTreeWidgetItem*)> collect;
  collect = [&](QTreeWidgetItem* item) {
    if (item == nullptr || item->isHidden()) {
      return;
    }
    const QString channel =
        item->data(kColumnName, plot::kTopicChannelRole).toString();
    const QString message_type =
        item->data(kColumnName, kRoleMessageType).toString();
    if (!channel.isEmpty() &&
        item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool() &&
        !message_type.isEmpty()) {
      if (static_cast<int>(wanted.size()) < kMaxProbeChannels) {
        wanted.emplace_back(channel.toStdString(), message_type.toStdString());
      }
    }
    // Prefer probing expanded subtrees first by walking children when expanded.
    if (item->isExpanded()) {
      for (int i = 0; i < item->childCount(); ++i) {
        collect(item->child(i));
      }
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    collect(tree_->topLevelItem(i));
  }
  // Second pass: fill remaining budget with collapsed visible channel leaves.
  if (static_cast<int>(wanted.size()) < kMaxProbeChannels) {
    std::function<void(QTreeWidgetItem*)> collect_all;
    collect_all = [&](QTreeWidgetItem* item) {
      if (item == nullptr || item->isHidden()) {
        return;
      }
      const QString channel =
          item->data(kColumnName, plot::kTopicChannelRole).toString();
      const QString message_type =
          item->data(kColumnName, kRoleMessageType).toString();
      if (!channel.isEmpty() &&
          item->data(kColumnName, plot::kTopicChannelDraggableRole).toBool() &&
          !message_type.isEmpty()) {
        const std::string key = channel.toStdString();
        const bool already =
            std::any_of(wanted.begin(), wanted.end(),
                        [&](const auto& e) { return e.first == key; });
        if (!already && static_cast<int>(wanted.size()) < kMaxProbeChannels) {
          wanted.emplace_back(key, message_type.toStdString());
        }
      }
      for (int i = 0; i < item->childCount(); ++i) {
        collect_all(item->child(i));
      }
    };
    for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
      collect_all(tree_->topLevelItem(i));
    }
  }

  std::unordered_map<std::string, std::string> wanted_map;
  for (const auto& entry : wanted) {
    wanted_map.emplace(entry.first, entry.second);
  }

  // Drop probes that are no longer needed.
  for (auto it = probe_subscriptions_.begin(); it != probe_subscriptions_.end();) {
    if (wanted_map.find(it->first) == wanted_map.end()) {
      integration::ChannelReaderRegistry::instance().unsubscribe(it->second);
      {
        std::lock_guard<std::mutex> lock(probe_mutex_);
        last_payloads_.erase(it->first);
        last_message_types_.erase(it->first);
      }
      it = probe_subscriptions_.erase(it);
    } else {
      ++it;
    }
  }

  // Add new probes.
  for (const auto& entry : wanted_map) {
    if (probe_subscriptions_.count(entry.first) > 0) {
      continue;
    }
    const std::string channel = entry.first;
    const std::string message_type = entry.second;
    // Internal playback / control channels must not be probed — they are owned
    // by PlaybackController via ChannelReaderRegistry already.
    if (channel.rfind("/autolink/", 0) == 0) {
      continue;
    }
    const auto id = integration::ChannelReaderRegistry::instance().subscribe(
        channel, [this, channel, message_type](const std::string& payload) {
          storeProbePayload(channel, message_type, payload);
        });
    if (id != 0) {
      probe_subscriptions_.emplace(channel, id);
    }
  }
}

}  // namespace autoviz
