/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channels/channels_panel.hpp"

#include <functional>
#include <map>

#include <QAbstractItemView>
#include <QColor>
#include <QDrag>
#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/integration/channel_stats_registry.hpp"
#include "autoviz/ui/map/map_message_ingest.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace {

constexpr int kColumnName = 0;
constexpr int kColumnSchema = 1;
constexpr int kColumnHz = 2;
constexpr int kColumnCount = 3;

/** Light tokens shared with Log / TF Tree / Teleop. */
constexpr char kBg[] = "#f8f9fb";
constexpr char kSurface[] = "#ffffff";
constexpr char kBorder[] = "#cbd5e1";
constexpr char kText[] = "#1e293b";
constexpr char kTextMuted[] = "#64748b";
constexpr char kAccent[] = "#0891b2";

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

bool ItemMatchesFilter(QTreeWidgetItem* item, const QString& needle) {
  if (item == nullptr || needle.isEmpty()) {
    return true;
  }
  const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
  const QString schema = item->data(kColumnName, Qt::UserRole + 20).toString();
  const QString field_path = item->data(kColumnName, plot::kTopicFieldPathRole).toString();
  const QString haystack = QStringLiteral("%1 %2 %3 %4")
                               .arg(item->text(kColumnName), channel, schema, field_path);
  return haystack.contains(needle, Qt::CaseInsensitive);
}

void ApplyFilterRecursive(QTreeWidgetItem* item, const QString& needle) {
  if (item == nullptr) {
    return;
  }
  bool child_visible = false;
  for (int i = 0; i < item->childCount(); ++i) {
    ApplyFilterRecursive(item->child(i), needle);
    if (!item->child(i)->isHidden()) {
      child_visible = true;
    }
  }
  const bool self_match = ItemMatchesFilter(item, needle);
  item->setHidden(!needle.isEmpty() && !self_match && !child_visible);
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
  filter_icon->setStyleSheet(
      QStringLiteral("color: %1; font-size: 14px;").arg(QLatin1String(kTextMuted)));
  filter_icon->setFixedWidth(16);
  filter_row->addWidget(filter_icon);

  filter_edit_ = new QLineEdit(toolbar);
  filter_edit_->setPlaceholderText(tr("Search channels…"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setStyleSheet(QStringLiteral(
      "QLineEdit {"
      "  background: %1; color: %2;"
      "  border: 1px solid %3; border-radius: 8px;"
      "  padding: 6px 10px; min-height: 26px;"
      "}"
      "QLineEdit:focus { border-color: %4; }")
                                  .arg(QLatin1String(kSurface), QLatin1String(kText),
                                       QLatin1String(kBorder), QLatin1String(kAccent)));
  filter_row->addWidget(filter_edit_, 1);

  auto* expand_button = new QToolButton(toolbar);
  expand_button->setText(tr("Expand"));
  expand_button->setToolTip(tr("Expand all channel trees"));
  expand_button->setCursor(Qt::PointingHandCursor);
  expand_button->setStyleSheet(ToolButtonStyle());
  filter_row->addWidget(expand_button);

  auto* collapse_button = new QToolButton(toolbar);
  collapse_button->setText(tr("Collapse"));
  collapse_button->setToolTip(tr("Collapse all channel trees"));
  collapse_button->setCursor(Qt::PointingHandCursor);
  collapse_button->setStyleSheet(ToolButtonStyle());
  filter_row->addWidget(collapse_button);

  status_label_ = new QLabel(toolbar);
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_label_->setStyleSheet(QStringLiteral(
      "color: %1; background: rgba(8,145,178,0.10);"
      "border: 1px solid rgba(8,145,178,0.28); border-radius: 10px;"
      "padding: 3px 10px; font-size: 11px; font-weight: 700;")
                                   .arg(QLatin1String(kAccent)));
  filter_row->addWidget(status_label_);
  toolbar_layout->addLayout(filter_row);

  auto* hint = new QLabel(
      tr("Drag a channel or field onto Plot / Table / Image / Map."), toolbar);
  hint->setStyleSheet(
      QStringLiteral("color: %1; font-size: 11px;").arg(QLatin1String(kTextMuted)));
  toolbar_layout->addWidget(hint);
  layout->addWidget(toolbar);

  auto* tree_card = new QFrame(this);
  tree_card->setObjectName(QStringLiteral("ChannelsTreeCard"));
  auto* tree_card_layout = new QVBoxLayout(tree_card);
  tree_card_layout->setContentsMargins(0, 0, 0, 0);
  tree_card_layout->setSpacing(0);

  tree_ = new ChannelsTreeWidget(tree_card);
  tree_->setObjectName(QStringLiteral("ChannelsTree"));
  tree_->setColumnCount(4);
  tree_->setHeaderLabels(
      {tr("Channel"), tr("Schema"), tr("Hz"), tr("Count")});
  tree_->header()->setStretchLastSection(false);
  tree_->header()->setSectionResizeMode(kColumnName, QHeaderView::Stretch);
  tree_->header()->setSectionResizeMode(kColumnSchema, QHeaderView::ResizeToContents);
  tree_->header()->setSectionResizeMode(kColumnHz, QHeaderView::ResizeToContents);
  tree_->header()->setSectionResizeMode(kColumnCount, QHeaderView::ResizeToContents);
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
  tree_->setStyleSheet(QStringLiteral(
      "QTreeWidget#ChannelsTree {"
      "  background: %1; color: %2; border: none; outline: none;"
      "  font-size: 12px;"
      "}"
      "QTreeWidget#ChannelsTree::item {"
      "  padding: 3px 6px; min-height: 26px;"
      "}"
      "QTreeWidget#ChannelsTree::item:selected {"
      "  background: rgba(8,145,178,0.14); color: %2;"
      "}"
      "QTreeWidget#ChannelsTree::item:hover:!selected {"
      "  background: rgba(15,23,42,0.04);"
      "}"
      "QTreeWidget#ChannelsTree::branch {"
      "  background: transparent;"
      "}"
      "QHeaderView::section {"
      "  background: %3; color: %4;"
      "  border: none; border-bottom: 1px solid %5;"
      "  padding: 7px 8px;"
      "  font-size: 11px; font-weight: 700;"
      "  text-transform: uppercase;"
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
  tree_card_layout->addWidget(tree_, 1);

  empty_hint_ = new QLabel(
      tr("No channels yet.\nPublish or connect a node to populate this list."),
      tree_card);
  empty_hint_->setAlignment(Qt::AlignCenter);
  empty_hint_->setWordWrap(true);
  empty_hint_->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 13px; padding: 32px 24px;").arg(QLatin1String(kTextMuted)));
  empty_hint_->hide();
  tree_card_layout->addWidget(empty_hint_, 1);
  layout->addWidget(tree_card, 1);

  auto* footer = new QFrame(this);
  footer->setObjectName(QStringLiteral("ChannelsFooter"));
  auto* footer_layout = new QHBoxLayout(footer);
  footer_layout->setContentsMargins(12, 6, 12, 6);
  auto* footer_label = new QLabel(
      tr("Expand a channel to browse message fields"), footer);
  footer_label->setStyleSheet(
      QStringLiteral("color: %1; font-size: 11px;").arg(QLatin1String(kTextMuted)));
  footer_layout->addWidget(footer_label);
  layout->addWidget(footer);

  connect(filter_edit_, &QLineEdit::textChanged, this,
          [this](const QString&) { applyFilter(); });
  connect(tree_, &QTreeWidget::itemExpanded, this, &ChannelsPanel::onItemExpanded);
  connect(expand_button, &QToolButton::clicked, this, [this]() {
    if (tree_ != nullptr) {
      tree_->expandAll();
    }
  });
  connect(collapse_button, &QToolButton::clicked, this, [this]() {
    if (tree_ != nullptr) {
      tree_->collapseAll();
    }
  });

  stats_timer_ = new QTimer(this);
  stats_timer_->setInterval(500);
  connect(stats_timer_, &QTimer::timeout, this, &ChannelsPanel::refreshStats);
  stats_timer_->start();

  rebuildTree();
}

void ChannelsPanel::applyChromeStyles() {
  setStyleSheet(QStringLiteral(
      "QWidget#ChannelsPanelContent {"
      "  background: %1; color: %2;"
      "}"
      "QFrame#ChannelsToolbar {"
      "  background: %3;"
      "  border-bottom: 1px solid %4;"
      "}"
      "QFrame#ChannelsTreeCard {"
      "  background: %3;"
      "  border: none;"
      "}"
      "QFrame#ChannelsFooter {"
      "  background: %1;"
      "  border-top: 1px solid %4;"
      "}")
                    .arg(QLatin1String(kBg), QLatin1String(kText),
                         QLatin1String(kSurface), QLatin1String(kBorder)));
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
}

void ChannelsPanel::updateStatusChip() {
  if (status_label_ == nullptr) {
    return;
  }
  const int count = cached_channel_keys_.size();
  status_label_->setText(count == 1 ? tr("1 channel")
                                    : tr("%1 channels").arg(count));
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

void ChannelsPanel::refreshStats() { updateChannelStatsColumns(); }

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
    cached_channel_keys_.push_back(QStringLiteral("%1|%2").arg(channel_q, message_type));

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
        node->setData(kColumnName, plot::kTopicTableDraggableRole, false);
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
    parent->setData(kColumnName, Qt::UserRole + 20, message_type);
    parent->setToolTip(kColumnName, QStringLiteral("%1\n%2").arg(channel_q, message_type));
    parent->setText(kColumnSchema, ShortSchemaName(message_type));
    parent->setData(kColumnName, plot::kTopicChannelDraggableRole, true);
    parent->setData(kColumnName, plot::kTopicTableDraggableRole, true);
    parent->setData(kColumnName, plot::kTopicImageDraggableRole,
                    display::isImageMessageType(channel.message_type));
    parent->setData(kColumnName, plot::kTopicMapDraggableRole,
                    map::MapMessageIngest::SupportsMessageType(message_type));
    parent->setFlags(parent->flags() | Qt::ItemIsDragEnabled);
    styleChannelItem(parent, true);
  }

  cached_channel_keys_.sort(Qt::CaseInsensitive);
  tree_->sortItems(kColumnName, Qt::AscendingOrder);
  updateChannelStatsColumns();
  applyFilter();
  updateStatusChip();
}

void ChannelsPanel::updateChannelStatsColumns() {
  if (tree_ == nullptr) {
    return;
  }
  std::function<void(QTreeWidgetItem*)> visit;
  visit = [&](QTreeWidgetItem* item) {
    if (item == nullptr) {
      return;
    }
    const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
    if (!channel.isEmpty()) {
      const integration::ChannelStats stats =
          integration::ChannelStatsRegistry::instance().stats(
              channel.toStdString());
      const bool live = stats.frequency_hz > 0.0;
      item->setText(kColumnHz, FormatFrequency(stats.frequency_hz));
      item->setText(kColumnCount, stats.message_count > 0
                                     ? QString::number(stats.message_count)
                                     : QStringLiteral("—"));
      item->setForeground(kColumnHz,
                          QColor(live ? kAccent : kTextMuted));
      item->setForeground(kColumnCount,
                          QColor(stats.message_count > 0 ? kText : kTextMuted));
      QFont hz_font = item->font(kColumnHz);
      hz_font.setBold(live);
      item->setFont(kColumnHz, hz_font);
    }
    for (int i = 0; i < item->childCount(); ++i) {
      visit(item->child(i));
    }
  };
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    visit(tree_->topLevelItem(i));
  }
}

void ChannelsPanel::applyFilter() {
  if (tree_ == nullptr) {
    return;
  }
  const QString needle = filter_edit_ != nullptr ? filter_edit_->text().trimmed()
                                                 : QString();
  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    ApplyFilterRecursive(tree_->topLevelItem(i), needle);
  }
}

void ChannelsPanel::onItemExpanded(QTreeWidgetItem* item) {
  if (item == nullptr || item->data(kColumnName, Qt::UserRole + 21).toBool()) {
    return;
  }
  const QString channel = item->data(kColumnName, plot::kTopicChannelRole).toString();
  const QString message_type = item->data(kColumnName, Qt::UserRole + 20).toString();
  if (channel.isEmpty() || message_type.isEmpty()) {
    return;
  }
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
    }
    for (int i = 0; i < node->childCount(); ++i) {
      set_channel(node->child(i));
    }
  };
  set_channel(item);
  item->setData(kColumnName, Qt::UserRole + 21, true);
}

}  // namespace autoviz
