/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/display_tree_widget.hpp"

#include <QDrag>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFontMetrics>
#include <QMimeData>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QToolTip>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/display/display_group.hpp"
#include "autoviz/ui/display_tree_delegate.hpp"

namespace autoviz {
namespace {

constexpr char kDisplayDragMime[] = "application/x-autoviz-display-move";

const QColor kGroupDropFill(72, 145, 220, 140);
const QColor kGroupDropFillInvalid(220, 80, 80, 90);
const QColor kGroupDropBorder(0, 122, 204);
const QColor kGroupDropBorderInvalid(200, 60, 60);
const QColor kDropLineColor(0, 122, 204);
const QColor kInvalidDropFill(220, 80, 80, 50);
const QColor kDragSourceDim(160, 160, 160);

DisplayTreeItemKind ItemKind(const QTreeWidgetItem* item) {
  if (item == nullptr) {
    return DisplayTreeItemKind::kGlobalOptions;
  }
  return static_cast<DisplayTreeItemKind>(
      item->data(kDisplayTreeColName, kDisplayTreeRoleKind).toInt());
}

int DisplayIndex(const QTreeWidgetItem* item) {
  return item == nullptr
             ? -1
             : item->data(kDisplayTreeColName, kDisplayTreeRoleDisplayIndex)
                   .toInt();
}

int ChildIndex(const QTreeWidgetItem* item) {
  return item == nullptr
             ? -1
             : item->data(kDisplayTreeColName, kDisplayTreeRoleChildIndex).toInt();
}

bool IsGroupItem(const QTreeWidgetItem* item,
                 const common::VisualizationManager& manager) {
  const int index = DisplayIndex(item);
  if (index < 0 ||
      static_cast<std::size_t>(index) >= manager.displays().size()) {
    return false;
  }
  return manager.displays()[static_cast<std::size_t>(index)]->typeId() ==
         "Group";
}

bool IsDisplayRowKind(DisplayTreeItemKind kind) {
  return kind == DisplayTreeItemKind::kDisplay ||
         kind == DisplayTreeItemKind::kDisplayChild;
}

}  // namespace

DisplayTreeWidget::DisplayTreeWidget(
    std::shared_ptr<common::VisualizationManager> manager, QWidget* parent)
    : QTreeWidget(parent), manager_(std::move(manager)) {
  setDragEnabled(true);
  setAcceptDrops(true);
  setDropIndicatorShown(false);
  setDragDropMode(QAbstractItemView::DragDrop);
  setDefaultDropAction(Qt::MoveAction);
  setSelectionMode(QAbstractItemView::SingleSelection);
  setMouseTracking(true);
  viewport()->setMouseTracking(true);

  hover_expand_timer_ = new QTimer(this);
  hover_expand_timer_->setSingleShot(true);
  hover_expand_timer_->setInterval(450);
  connect(hover_expand_timer_, &QTimer::timeout, this, [this]() {
    if (hover_expand_item_ != nullptr) {
      hover_expand_item_->setExpanded(true);
    }
  });
}

QTreeWidgetItem* DisplayTreeWidget::DisplayRowItem(QTreeWidgetItem* item) {
  while (item != nullptr) {
    const auto kind = ItemKind(item);
    if (IsDisplayRowKind(kind)) {
      return item;
    }
    item = item->parent();
  }
  return nullptr;
}

std::size_t DisplayTreeWidget::RootInsertIndexForPosition(
    QTreeWidget* tree, QTreeWidgetItem* target_item,
    QAbstractItemView::DropIndicatorPosition position) {
  if (target_item == nullptr) {
    return tree->topLevelItemCount();
  }

  QTreeWidgetItem* display_row = DisplayRowItem(target_item);
  if (display_row == nullptr) {
    return tree->topLevelItemCount();
  }

  const auto kind = ItemKind(display_row);
  if (kind == DisplayTreeItemKind::kDisplayChild) {
    return tree->indexOfTopLevelItem(display_row->parent()) + 1;
  }

  const int top_index = tree->indexOfTopLevelItem(display_row);
  if (top_index < 0) {
    return tree->topLevelItemCount();
  }
  if (position == QAbstractItemView::BelowItem) {
    return static_cast<std::size_t>(top_index + 1);
  }
  return static_cast<std::size_t>(top_index);
}

QAbstractItemView::DropIndicatorPosition
DisplayTreeWidget::computeDropPosition(const QPoint& pos,
                                       QTreeWidgetItem* item) const {
  if (item == nullptr) {
    return QAbstractItemView::OnViewport;
  }

  if (QTreeWidgetItem* group_row = resolveGroupDropTarget(item)) {
    if (item != group_row) {
      return QAbstractItemView::OnItem;
    }
    const QRect rect = visualItemRect(group_row);
    if (!rect.contains(pos)) {
      return QAbstractItemView::OnViewport;
    }
    const int edge = 5;
    if (pos.y() <= rect.top() + edge) {
      return QAbstractItemView::AboveItem;
    }
    if (pos.y() >= rect.bottom() - edge) {
      return QAbstractItemView::BelowItem;
    }
    return QAbstractItemView::OnItem;
  }

  const QRect rect = visualItemRect(item);
  if (!rect.contains(pos)) {
    return QAbstractItemView::OnViewport;
  }
  const int third = qMax(4, rect.height() / 3);
  if (pos.y() < rect.top() + third) {
    return QAbstractItemView::AboveItem;
  }
  if (pos.y() > rect.bottom() - third) {
    return QAbstractItemView::BelowItem;
  }
  return QAbstractItemView::OnItem;
}

QTreeWidgetItem* DisplayTreeWidget::resolveGroupDropTarget(
    QTreeWidgetItem* item) const {
  for (QTreeWidgetItem* node = item; node != nullptr; node = node->parent()) {
    if (ItemKind(node) == DisplayTreeItemKind::kDisplay &&
        IsGroupItem(node, *manager_)) {
      return node;
    }
  }
  return nullptr;
}

bool DisplayTreeWidget::canDropIntoGroup(const DisplayDragPayload& source,
                                         QTreeWidgetItem* group_row) const {
  if (group_row == nullptr || source.display_index < 0) {
    return false;
  }
  const std::size_t group_index =
      static_cast<std::size_t>(DisplayIndex(group_row));
  if (group_index >= manager_->displays().size()) {
    return false;
  }
  if (source.child_index < 0 &&
      source.display_index == static_cast<int>(group_index)) {
    return false;
  }
  const display::Display* moving =
      manager_->displayAt(static_cast<std::size_t>(source.display_index),
                          source.child_index);
  const display::Display* group_display = manager_->displays()[group_index];
  if (moving == nullptr || group_display == nullptr) {
    return false;
  }
  return !common::VisualizationManager::isDisplayInSubtree(moving,
                                                           group_display);
}

QRect DisplayTreeWidget::groupDropHighlightRect(
    QTreeWidgetItem* group_row) const {
  if (group_row == nullptr) {
    return {};
  }
  QRect rect = visualItemRect(group_row);
  if (!rect.isValid()) {
    return {};
  }
  rect.setLeft(0);
  rect.setRight(viewport()->width() - 1);
  if (group_row->isExpanded() && group_row->childCount() > 0) {
    QTreeWidgetItem* last = group_row->child(group_row->childCount() - 1);
    const QRect last_rect = visualItemRect(last);
    if (last_rect.isValid()) {
      rect.setBottom(last_rect.bottom());
    }
  }
  return rect;
}

QMimeData* DisplayTreeWidget::MakeDragPayload(
    const DisplayDragPayload& payload) {
  auto* mime = new QMimeData();
  mime->setData(kDisplayDragMime,
                QByteArray::number(payload.display_index) + ',' +
                    QByteArray::number(payload.child_index));
  return mime;
}

bool DisplayTreeWidget::ReadDragPayload(const QMimeData* mime,
                                        DisplayDragPayload* out) {
  if (mime == nullptr || out == nullptr ||
      !mime->hasFormat(kDisplayDragMime)) {
    return false;
  }
  const QList<QByteArray> parts = mime->data(kDisplayDragMime).split(',');
  if (parts.size() != 2) {
    return false;
  }
  bool ok_display = false;
  bool ok_child = false;
  out->display_index = parts[0].toInt(&ok_display);
  out->child_index = parts[1].toInt(&ok_child);
  return ok_display && ok_child;
}

void DisplayTreeWidget::restoreItemBackground(QTreeWidgetItem* item) {
  if (item == nullptr) {
    return;
  }
  if (item->data(kDisplayTreeColName, kDisplayTreeRoleDragBgSaved).isValid()) {
    item->setBackground(
        kDisplayTreeColName,
        item->data(kDisplayTreeColName, kDisplayTreeRoleDragBgSaved).value<QBrush>());
    item->setBackground(
        kDisplayTreeColValue,
        item->data(kDisplayTreeColValue, kDisplayTreeRoleDragBgSaved).value<QBrush>());
    item->setData(kDisplayTreeColName, kDisplayTreeRoleDragBgSaved, QVariant());
    item->setData(kDisplayTreeColValue, kDisplayTreeRoleDragBgSaved, QVariant());
  }
}

void DisplayTreeWidget::highlightItem(QTreeWidgetItem* item,
                                      const QBrush& brush) {
  if (item == nullptr) {
    return;
  }
  if (!item->data(kDisplayTreeColName, kDisplayTreeRoleDragBgSaved).isValid()) {
    item->setData(kDisplayTreeColName, kDisplayTreeRoleDragBgSaved,
                  item->background(kDisplayTreeColName));
    item->setData(kDisplayTreeColValue, kDisplayTreeRoleDragBgSaved,
                  item->background(kDisplayTreeColValue));
  }
  item->setBackground(kDisplayTreeColName, brush);
  item->setBackground(kDisplayTreeColValue, brush);
}

void DisplayTreeWidget::resetDragVisualState() {
  hover_expand_timer_->stop();
  hover_expand_item_ = nullptr;
  drop_highlight_row_ = nullptr;
  drag_source_row_ = nullptr;
  drop_line_y_ = -1;
  drop_visual_mode_ = DropVisualMode::kNone;
  group_drop_label_.clear();
  unsetCursor();
  QToolTip::hideText();
}

void DisplayTreeWidget::clearDragVisuals() {
  if (drop_highlight_row_ != nullptr) {
    restoreItemBackground(drop_highlight_row_);
  }
  if (drag_source_row_ != nullptr) {
    drag_source_row_->setForeground(kDisplayTreeColName, QBrush());
    drag_source_row_->setForeground(kDisplayTreeColValue, QBrush());
  }
  resetDragVisualState();
  viewport()->update();
}

void DisplayTreeWidget::scheduleGroupExpand(QTreeWidgetItem* group_row) {
  if (group_row == hover_expand_item_) {
    return;
  }
  hover_expand_timer_->stop();
  hover_expand_item_ = group_row;
  if (group_row != nullptr) {
    hover_expand_timer_->start();
  }
}

void DisplayTreeWidget::updateDragVisuals(
    const DisplayDragPayload& source, QTreeWidgetItem* target_item,
    QAbstractItemView::DropIndicatorPosition position, bool valid) {
  if (drop_highlight_row_ != nullptr) {
    restoreItemBackground(drop_highlight_row_);
    drop_highlight_row_ = nullptr;
  }

  drop_line_y_ = -1;
  group_drop_label_.clear();
  drop_visual_mode_ = valid ? DropVisualMode::kLineAbove : DropVisualMode::kInvalid;

  const bool group_drop =
      position == QAbstractItemView::OnItem &&
      resolveGroupDropTarget(target_item) != nullptr;

  if (group_drop) {
    QTreeWidgetItem* group_row = resolveGroupDropTarget(target_item);
    const bool group_valid = valid && canDropIntoGroup(source, group_row);
    drop_highlight_row_ = group_row;
    drop_visual_mode_ =
        group_valid ? DropVisualMode::kGroupTarget : DropVisualMode::kInvalid;
    if (group_valid) {
      setCursor(Qt::DragMoveCursor);
      scheduleGroupExpand(group_row);
      group_drop_label_ =
          tr("Drop into Group \"%1\"")
              .arg(group_row->text(kDisplayTreeColName));
      QToolTip::showText(QCursor::pos(), group_drop_label_);
    } else {
      setCursor(Qt::ForbiddenCursor);
      group_drop_label_ = tr("Cannot drop here");
      QToolTip::showText(QCursor::pos(), group_drop_label_);
    }
    viewport()->update();
    return;
  }

  QTreeWidgetItem* display_row = DisplayRowItem(target_item);

  if (!valid) {
    if (display_row != nullptr) {
      highlightItem(display_row, QBrush(kInvalidDropFill));
      drop_highlight_row_ = display_row;
      drop_visual_mode_ = DropVisualMode::kInvalid;
    }
    setCursor(Qt::ForbiddenCursor);
    QToolTip::showText(QCursor::pos(), tr("Cannot drop here"));
    viewport()->update();
    return;
  }

  setCursor(Qt::DragMoveCursor);

  if (display_row == nullptr) {
    drop_line_y_ = viewport()->height() - 2;
    drop_visual_mode_ = DropVisualMode::kLineBelow;
    QToolTip::showText(QCursor::pos(), tr("Move to root level"));
    viewport()->update();
    return;
  }

  const QRect rect = visualItemRect(display_row);

  if (position == QAbstractItemView::AboveItem) {
    drop_line_y_ = rect.top();
    drop_visual_mode_ = DropVisualMode::kLineAbove;
    hover_expand_timer_->stop();
    hover_expand_item_ = nullptr;
    QToolTip::showText(QCursor::pos(), tr("Insert above"));
  } else if (position == QAbstractItemView::BelowItem) {
    drop_line_y_ = rect.bottom();
    drop_visual_mode_ = DropVisualMode::kLineBelow;
    hover_expand_timer_->stop();
    hover_expand_item_ = nullptr;
    QToolTip::showText(QCursor::pos(), tr("Insert below"));
  } else {
    drop_line_y_ = rect.bottom();
    drop_visual_mode_ = DropVisualMode::kLineBelow;
    QToolTip::showText(QCursor::pos(), tr("Move to root level"));
  }

  (void)source;
  viewport()->update();
}

QPixmap DisplayTreeWidget::makeDragPixmap(QTreeWidgetItem* row) const {
  const QString text = row->text(kDisplayTreeColName);
  const QIcon icon = row->icon(kDisplayTreeColName);
  const QFontMetrics fm(font());
  const int icon_sz = 22;
  const int pad = 8;
  const int text_w = fm.horizontalAdvance(text);
  const int w = 12 + icon_sz + 6 + text_w + pad;
  const int h = qMax(icon_sz, fm.height()) + pad;
  QPixmap pixmap(w, h);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setOpacity(0.92);
  painter.setBrush(QColor(232, 244, 255));
  painter.setPen(QPen(kGroupDropBorder, 2));
  painter.drawRoundedRect(QRectF(1, 1, w - 2, h - 2), 5, 5);
  if (!icon.isNull()) {
    icon.paint(&painter, pad, (h - icon_sz) / 2, icon_sz, icon_sz);
  }
  painter.setPen(palette().text().color());
  painter.drawText(pad + icon_sz + 6,
                   (h + fm.ascent() - fm.descent()) / 2, text);
  return pixmap;
}

void DisplayTreeWidget::startDrag(Qt::DropActions supported_actions) {
  QTreeWidgetItem* display_row = DisplayRowItem(currentItem());
  if (display_row == nullptr) {
    return;
  }

  DisplayDragPayload payload;
  payload.display_index = DisplayIndex(display_row);
  payload.child_index = ChildIndex(display_row);
  if (payload.display_index < 0) {
    return;
  }

  drag_source_row_ = display_row;
  display_row->setForeground(kDisplayTreeColName, QBrush(kDragSourceDim));
  display_row->setForeground(kDisplayTreeColValue, QBrush(kDragSourceDim));

  auto* drag = new QDrag(this);
  drag->setMimeData(MakeDragPayload(payload));
  const QPixmap pixmap = makeDragPixmap(display_row);
  drag->setPixmap(pixmap);
  drag->setHotSpot(QPoint(8, pixmap.height() / 2));
  drag->exec(supported_actions, Qt::MoveAction);
  clearDragVisuals();
}

void DisplayTreeWidget::dragEnterEvent(QDragEnterEvent* event) {
  DisplayDragPayload payload;
  if (ReadDragPayload(event->mimeData(), &payload)) {
    event->acceptProposedAction();
    return;
  }
  event->ignore();
}

void DisplayTreeWidget::dragMoveEvent(QDragMoveEvent* event) {
  DisplayDragPayload source;
  if (!ReadDragPayload(event->mimeData(), &source)) {
    event->ignore();
    clearDragVisuals();
    return;
  }

  QTreeWidgetItem* target_item = itemAt(event->position().toPoint());
  const auto position =
      computeDropPosition(event->position().toPoint(), target_item);
  const bool valid = canDropOn(source, target_item, position);
  if (valid) {
    event->acceptProposedAction();
  } else {
    event->ignore();
  }
  updateDragVisuals(source, target_item, position, valid);
}

void DisplayTreeWidget::dragLeaveEvent(QDragLeaveEvent* event) {
  QTreeWidget::dragLeaveEvent(event);
  if (drop_highlight_row_ != nullptr || drop_line_y_ >= 0 ||
      drop_visual_mode_ != DropVisualMode::kNone) {
    clearDragVisuals();
  }
}

void DisplayTreeWidget::dropEvent(QDropEvent* event) {
  DisplayDragPayload source;
  if (!ReadDragPayload(event->mimeData(), &source)) {
    event->ignore();
    clearDragVisuals();
    return;
  }

  QTreeWidgetItem* target_item = itemAt(event->position().toPoint());
  const auto position =
      computeDropPosition(event->position().toPoint(), target_item);
  if (!canDropOn(source, target_item, position)) {
    event->ignore();
    clearDragVisuals();
    return;
  }

  const bool moved = applyDrop(source, target_item, position);
  if (moved) {
    // refresh() rebuilds the tree; do not touch QTreeWidgetItem* after applyDrop.
    resetDragVisualState();
    event->acceptProposedAction();
    emit displaysReorganized();
  } else {
    clearDragVisuals();
    event->ignore();
  }
}

void DisplayTreeWidget::paintEvent(QPaintEvent* event) {
  QTreeWidget::paintEvent(event);

  QPainter painter(viewport());
  painter.setRenderHint(QPainter::Antialiasing, false);

  if (drop_line_y_ >= 0) {
    painter.setPen(QPen(kDropLineColor, 3));
    const int left = indentation() / 2;
    painter.drawLine(left, drop_line_y_, viewport()->width() - 4, drop_line_y_);
    painter.setBrush(kDropLineColor);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPoint(left, drop_line_y_), 4, 4);
    painter.drawEllipse(QPoint(viewport()->width() - 4, drop_line_y_), 4, 4);
  }

  if (drop_visual_mode_ == DropVisualMode::kGroupTarget &&
      drop_highlight_row_ != nullptr) {
    const QRect rect = groupDropHighlightRect(drop_highlight_row_);
    if (rect.isValid()) {
      painter.fillRect(rect, kGroupDropFill);
      painter.setPen(QPen(kGroupDropBorder, 2));
      painter.setBrush(Qt::NoBrush);
      painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 4, 4);
      if (!group_drop_label_.isEmpty()) {
        painter.setPen(kGroupDropBorder);
        QFont label_font = font();
        label_font.setBold(true);
        painter.setFont(label_font);
        painter.drawText(rect.adjusted(10, 0, -10, 0),
                         Qt::AlignRight | Qt::AlignVCenter, group_drop_label_);
      }
    }
  } else if (drop_visual_mode_ == DropVisualMode::kInvalid &&
             drop_highlight_row_ != nullptr &&
             resolveGroupDropTarget(drop_highlight_row_) != nullptr) {
    const QRect rect = groupDropHighlightRect(drop_highlight_row_);
    if (rect.isValid()) {
      painter.fillRect(rect, kGroupDropFillInvalid);
      painter.setPen(QPen(kGroupDropBorderInvalid, 2, Qt::DashLine));
      painter.setBrush(Qt::NoBrush);
      painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), 4, 4);
    }
  }
}

bool DisplayTreeWidget::canDropOn(
    const DisplayDragPayload& source, QTreeWidgetItem* target_item,
    QAbstractItemView::DropIndicatorPosition position) const {
  if (source.display_index < 0 ||
      static_cast<std::size_t>(source.display_index) >=
          manager_->displays().size()) {
    return false;
  }

  const display::Display* moving =
      manager_->displayAt(static_cast<std::size_t>(source.display_index),
                          source.child_index);
  if (moving == nullptr) {
    return false;
  }

  QTreeWidgetItem* display_row = DisplayRowItem(target_item);

  if (position == QAbstractItemView::OnItem) {
    if (QTreeWidgetItem* group_row = resolveGroupDropTarget(target_item)) {
      return canDropIntoGroup(source, group_row);
    }
  }

  if (display_row == nullptr) {
    return position == QAbstractItemView::OnItem ||
           position == QAbstractItemView::OnViewport ||
           position == QAbstractItemView::BelowItem;
  }

  const auto kind = ItemKind(display_row);
  if (kind == DisplayTreeItemKind::kDisplay &&
      IsGroupItem(display_row, *manager_) &&
      position == QAbstractItemView::OnItem) {
    return canDropIntoGroup(source, display_row);
  }

  if (kind == DisplayTreeItemKind::kDisplayChild &&
      position == QAbstractItemView::OnItem) {
    QTreeWidgetItem* parent = display_row->parent();
    if (parent == nullptr || !IsGroupItem(parent, *manager_)) {
      return false;
    }
    return canDropIntoGroup(source, parent);
  }

  if (kind == DisplayTreeItemKind::kDisplay ||
      kind == DisplayTreeItemKind::kDisplayChild) {
    return position == QAbstractItemView::AboveItem ||
           position == QAbstractItemView::BelowItem ||
           (kind == DisplayTreeItemKind::kDisplay &&
            !IsGroupItem(display_row, *manager_) &&
            position == QAbstractItemView::OnItem);
  }

  return false;
}

bool DisplayTreeWidget::applyDrop(
    const DisplayDragPayload& source, QTreeWidgetItem* target_item,
    QAbstractItemView::DropIndicatorPosition position) {
  if (position == QAbstractItemView::OnItem) {
    if (QTreeWidgetItem* group_row = resolveGroupDropTarget(target_item)) {
      if (canDropIntoGroup(source, group_row)) {
        return manager_->moveDisplay(
            static_cast<std::size_t>(source.display_index), source.child_index,
            static_cast<std::size_t>(DisplayIndex(group_row)), -1);
      }
      return false;
    }
  }

  QTreeWidgetItem* display_row = DisplayRowItem(target_item);

  if (display_row != nullptr) {
    const auto kind = ItemKind(display_row);

    if (kind == DisplayTreeItemKind::kDisplayChild &&
        position == QAbstractItemView::OnItem) {
      QTreeWidgetItem* parent = display_row->parent();
      if (parent == nullptr) {
        return false;
      }
      const int insert_child = ChildIndex(display_row);
      return manager_->moveDisplay(
          static_cast<std::size_t>(source.display_index), source.child_index,
          static_cast<std::size_t>(DisplayIndex(parent)), insert_child);
    }

    if (kind == DisplayTreeItemKind::kDisplay ||
        kind == DisplayTreeItemKind::kDisplayChild) {
      const std::size_t root_insert =
          RootInsertIndexForPosition(this, display_row, position);
      if (source.child_index < 0) {
        if (source.display_index == static_cast<int>(root_insert) ||
            (position == QAbstractItemView::BelowItem &&
             source.display_index + 1 == static_cast<int>(root_insert))) {
          return false;
        }
        if (position == QAbstractItemView::AboveItem ||
            position == QAbstractItemView::BelowItem) {
          return manager_->reorderRootDisplay(
              static_cast<std::size_t>(source.display_index), root_insert);
        }
      }
      return manager_->moveDisplayToRoot(
          static_cast<std::size_t>(source.display_index), source.child_index,
          root_insert);
    }
  }

  const std::size_t root_insert =
      RootInsertIndexForPosition(this, nullptr, position);
  return manager_->moveDisplayToRoot(
      static_cast<std::size_t>(source.display_index), source.child_index,
      root_insert);
}

}  // namespace autoviz
