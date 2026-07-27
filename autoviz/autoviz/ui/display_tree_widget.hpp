/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>

#include <QTreeWidget>

class QTimer;
class QTreeWidgetItem;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/** Displays tree with RViz-style drag-and-drop grouping. */
class DisplayTreeWidget : public QTreeWidget {
  Q_OBJECT

 public:
  explicit DisplayTreeWidget(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);

 signals:
  void displaysReorganized();

 protected:
  void startDrag(Qt::DropActions supported_actions) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dragLeaveEvent(QDragLeaveEvent* event) override;
  void dropEvent(QDropEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  struct DisplayDragPayload {
    int display_index = -1;
    int child_index = -1;
  };

  enum class DropVisualMode {
    kNone,
    kLineAbove,
    kLineBelow,
    kGroupTarget,
    kInvalid,
  };

  static QTreeWidgetItem* DisplayRowItem(QTreeWidgetItem* item);
  static std::size_t RootInsertIndexForPosition(
      QTreeWidget* tree, QTreeWidgetItem* target_item,
      QAbstractItemView::DropIndicatorPosition position);
  static bool ReadDragPayload(const QMimeData* mime, DisplayDragPayload* out);
  static QMimeData* MakeDragPayload(const DisplayDragPayload& payload);
  QAbstractItemView::DropIndicatorPosition computeDropPosition(
      const QPoint& pos, QTreeWidgetItem* item) const;
  QTreeWidgetItem* resolveGroupDropTarget(QTreeWidgetItem* item) const;
  bool canDropIntoGroup(const DisplayDragPayload& source,
                        QTreeWidgetItem* group_row) const;
  QRect groupDropHighlightRect(QTreeWidgetItem* group_row) const;
  bool canDropOn(const DisplayDragPayload& source, QTreeWidgetItem* target_item,
                 QAbstractItemView::DropIndicatorPosition position) const;
  bool applyDrop(const DisplayDragPayload& source, QTreeWidgetItem* target_item,
                 QAbstractItemView::DropIndicatorPosition position);
  void clearDragVisuals();
  /** Clears drag state without touching tree items (safe after tree rebuild). */
  void resetDragVisualState();
  void updateDragVisuals(const DisplayDragPayload& source,
                         QTreeWidgetItem* target_item,
                         QAbstractItemView::DropIndicatorPosition position,
                         bool valid);
  QPixmap makeDragPixmap(QTreeWidgetItem* row) const;
  void restoreItemBackground(QTreeWidgetItem* item);
  void highlightItem(QTreeWidgetItem* item, const QBrush& brush);
  void scheduleGroupExpand(QTreeWidgetItem* group_row);

  std::shared_ptr<common::VisualizationManager> manager_;
  QTreeWidgetItem* drag_source_row_ = nullptr;
  QTreeWidgetItem* drop_highlight_row_ = nullptr;
  QTreeWidgetItem* hover_expand_item_ = nullptr;
  int drop_line_y_ = -1;
  DropVisualMode drop_visual_mode_ = DropVisualMode::kNone;
  QString group_drop_label_;
  QTimer* hover_expand_timer_ = nullptr;
};

}  // namespace autoviz
