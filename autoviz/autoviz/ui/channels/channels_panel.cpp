/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channels/channels_panel.hpp"

#include <functional>
#include <map>

#include <QDrag>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/integration/channel_stats_registry.hpp"
#include "autoviz/ui/map/map_message_ingest.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"

namespace autoviz {
namespace {

constexpr int kColumnName = 0;
constexpr int kColumnSchema = 1;
constexpr int kColumnHz = 2;
constexpr int kColumnCount = 3;

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
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* header = new QHBoxLayout();
  header->addWidget(new QLabel(tr("Filter"), this));
  filter_edit_ = new QLineEdit(this);
  filter_edit_->setPlaceholderText(tr("Search channels…"));
  filter_edit_->setClearButtonEnabled(true);
  header->addWidget(filter_edit_, 1);
  layout->addLayout(header);

  tree_ = new ChannelsTreeWidget(this);
  tree_->setColumnCount(4);
  tree_->setHeaderLabels(
      {tr("Channel"), tr("Schema"), tr("Hz"), tr("Count")});
  tree_->header()->setStretchLastSection(false);
  tree_->header()->setSectionResizeMode(kColumnName, QHeaderView::Stretch);
  tree_->header()->setSectionResizeMode(kColumnSchema, QHeaderView::ResizeToContents);
  tree_->header()->setSectionResizeMode(kColumnHz, QHeaderView::ResizeToContents);
  tree_->header()->setSectionResizeMode(kColumnCount, QHeaderView::ResizeToContents);
  tree_->setRootIsDecorated(true);
  tree_->setDragEnabled(true);
  tree_->setDragDropMode(QAbstractItemView::DragOnly);
  tree_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  tree_->setExpandsOnDoubleClick(true);
  tree_->setAlternatingRowColors(true);
  layout->addWidget(tree_, 1);

  connect(filter_edit_, &QLineEdit::textChanged, this,
          [this](const QString&) { applyFilter(); });
  connect(tree_, &QTreeWidget::itemExpanded, this, &ChannelsPanel::onItemExpanded);

  stats_timer_ = new QTimer(this);
  stats_timer_->setInterval(500);
  connect(stats_timer_, &QTimer::timeout, this, &ChannelsPanel::refreshStats);
  stats_timer_->start();

  rebuildTree();
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
  }

  cached_channel_keys_.sort(Qt::CaseInsensitive);
  tree_->sortItems(kColumnName, Qt::AscendingOrder);
  updateChannelStatsColumns();
  applyFilter();
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
      item->setText(kColumnHz, FormatFrequency(stats.frequency_hz));
      item->setText(kColumnCount, stats.message_count > 0
                                     ? QString::number(stats.message_count)
                                     : QStringLiteral("—"));
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
    }
    for (int i = 0; i < node->childCount(); ++i) {
      set_channel(node->child(i));
    }
  };
  set_channel(item);
  item->setData(kColumnName, Qt::UserRole + 21, true);
}

}  // namespace autoviz
