/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/topics_panel.hpp"

#include <functional>
#include <map>

#include <QDrag>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_manager.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"

namespace autoviz {
namespace {

class TopicsTreeWidget : public QTreeWidget {
 public:
  using QTreeWidget::QTreeWidget;

 protected:
  void startDrag(Qt::DropActions supported_actions) override {
    QTreeWidgetItem* item = currentItem();
    if (item == nullptr) {
      return;
    }
    const QString channel = item->data(0, plot::kTopicChannelRole).toString();
    const QString field_path = item->data(0, plot::kTopicFieldPathRole).toString();
    if (channel.isEmpty()) {
      return;
    }
    const bool table_draggable =
        item->data(0, plot::kTopicTableDraggableRole).toBool();
    const bool plot_draggable = item->data(0, plot::kTopicDraggableRole).toBool();
    if (!table_draggable && !plot_draggable) {
      return;
    }
    plot::PlotSeriesDragPayload payload;
    payload.channel = channel;
    payload.field_path = field_path;
    auto* drag = new QDrag(this);
    drag->setMimeData(plot::MakePlotSeriesDragPayload(payload));
    drag->exec(supported_actions, Qt::CopyAction);
  }
};

}  // namespace

TopicsPanel::TopicsPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);

  auto* header = new QHBoxLayout();
  header->addWidget(new QLabel(tr("Filter"), this));
  filter_edit_ = new QLineEdit(this);
  filter_edit_->setPlaceholderText(tr("Search topics…"));
  header->addWidget(filter_edit_, 1);
  layout->addLayout(header);

  tree_ = new TopicsTreeWidget(this);
  tree_->setHeaderHidden(true);
  tree_->setDragEnabled(true);
  tree_->setDragDropMode(QAbstractItemView::DragOnly);
  tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  tree_->setExpandsOnDoubleClick(true);
  layout->addWidget(tree_, 1);

  connect(filter_edit_, &QLineEdit::textChanged, this,
          [this](const QString& text) {
            const QString needle = text.trimmed();
            for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
              QTreeWidgetItem* item = tree_->topLevelItem(i);
              if (item == nullptr) {
                continue;
              }
              const bool match =
                  needle.isEmpty() ||
                  item->text(0).contains(needle, Qt::CaseInsensitive);
              item->setHidden(!match);
            }
          });
  connect(tree_, &QTreeWidget::itemExpanded, this, &TopicsPanel::onItemExpanded);

  rebuildTree();
}

void TopicsPanel::refreshChannels() { rebuildTree(); }

void TopicsPanel::rebuildTree() {
  if (tree_ == nullptr) {
    return;
  }
  tree_->clear();
  if (manager_ == nullptr) {
    return;
  }

  std::map<QString, QTreeWidgetItem*> path_nodes;
  for (const integration::ChannelInfo& channel : manager_->channels()) {
    const QString channel_q = QString::fromStdString(channel.channel_name);
    if (channel_q.isEmpty()) {
      continue;
    }
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
        node->setText(0, segment);
        node->setData(0, plot::kTopicDraggableRole, false);
        path_nodes.emplace(built_path, node);
      }
      parent = node;
    }
    if (parent == nullptr) {
      continue;
    }
    parent->setData(0, plot::kTopicChannelRole, channel_q);
    parent->setData(0, plot::kTopicFieldPathRole, QString());
    parent->setToolTip(0, channel_q);
    parent->setData(0, plot::kTopicTableDraggableRole, true);
    parent->setFlags(parent->flags() | Qt::ItemIsDragEnabled);
    parent->setData(0, Qt::UserRole + 20, QString::fromStdString(channel.message_type));
  }
  tree_->sortItems(0, Qt::AscendingOrder);
}

void TopicsPanel::onItemExpanded(QTreeWidgetItem* item) {
  if (item == nullptr || item->data(0, Qt::UserRole + 21).toBool()) {
    return;
  }
  const QString channel = item->data(0, plot::kTopicChannelRole).toString();
  const QString message_type = item->data(0, Qt::UserRole + 20).toString();
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
    if (node->data(0, plot::kTopicDraggableRole).toBool() ||
        node->data(0, plot::kTopicTableDraggableRole).toBool()) {
      node->setData(0, plot::kTopicChannelRole, channel);
    }
    for (int i = 0; i < node->childCount(); ++i) {
      set_channel(node->child(i));
    }
  };
  set_channel(item);
  item->setData(0, Qt::UserRole + 21, true);
}

}  // namespace autoviz
