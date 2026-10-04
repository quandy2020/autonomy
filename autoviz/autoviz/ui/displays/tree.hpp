/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tree.hpp
 * @brief Displays property tree with RViz-style drag-and-drop grouping.
 *
 * Extends @c QTreeWidget so Display rows can be reordered at the root or
 * dropped into another Display to form a parent/child group. Visual feedback
 * (insertion line, group highlight, hover-expand) is painted in
 * @ref paintEvent() while a drag is active.
 *
 * @see DisplaysPanel
 * @see DisplayTreeItemKind
 * @see common::VisualizationManager
 */

#pragma once

#include <memory>

#include <QTreeWidget>

class QTimer;
class QTreeWidgetItem;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class DisplayTreeWidget
 * @brief Property tree that reorganizes Displays via drag-and-drop.
 *
 * ## Drop modes
 *
 * | Visual | Meaning |
 * |--------|---------|
 * | Line above / below | Insert as sibling at that root (or group) position |
 * | Group highlight | Nest source Display as a child of the target Display |
 * | Invalid | Drop rejected (e.g. into self / descendant) |
 *
 * Successful drops mutate VisualizationManager's display order / grouping and
 * emit @ref displaysReorganized() so @ref DisplaysPanel can refresh.
 *
 * @note While an in-place cell editor is open (@ref isEditing()), callers
 *       should avoid rebuilding the tree to preserve the editor.
 *
 * @see DisplaysPanel::refresh()
 */
class DisplayTreeWidget : public QTreeWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the tree and enables internal move drag-and-drop.
   *
   * @param manager Shared manager whose display list is mutated on drop.
   * @param parent Qt parent (typically @ref DisplaysPanel).
   */
  explicit DisplayTreeWidget(
      std::shared_ptr<common::VisualizationManager> manager,
      QWidget* parent = nullptr);

  /**
   * @brief True while an in-place cell editor is open.
   *
   * @return @c true when @c state() == @c EditingState.
   */
  bool isEditing() const { return state() == EditingState; }

 signals:
  /**
   * @brief Emitted after a successful drop reorders or regroups Displays.
   *
   * Listeners should call @ref DisplaysPanel::refresh() (or equivalent) and
   * persist session config.
   */
  void displaysReorganized();

 protected:
  /**
   * @brief Begins a drag with a custom pixmap and MIME payload.
   *
   * @param supported_actions Actions offered to the drag (typically Move).
   */
  void startDrag(Qt::DropActions supported_actions) override;

  /**
   * @brief Accepts drags that carry an internal Display payload.
   *
   * @param event Drag enter event.
   */
  void dragEnterEvent(QDragEnterEvent* event) override;

  /**
   * @brief Updates drop indicator / group highlight while dragging.
   *
   * @param event Drag move event with cursor position.
   */
  void dragMoveEvent(QDragMoveEvent* event) override;

  /**
   * @brief Clears drop visuals when the cursor leaves the viewport.
   *
   * @param event Drag leave event.
   */
  void dragLeaveEvent(QDragLeaveEvent* event) override;

  /**
   * @brief Applies the drop to VisualizationManager and emits
   *        @ref displaysReorganized().
   *
   * @param event Drop event.
   */
  void dropEvent(QDropEvent* event) override;

  /**
   * @brief Paints the tree plus active drop line / group highlight overlay.
   *
   * @param event Paint event.
   */
  void paintEvent(QPaintEvent* event) override;

 private:
  /**
   * @struct DisplayDragPayload
   * @brief MIME payload identifying the dragged Display row.
   */
  struct DisplayDragPayload {
    int display_index = -1;  /**< Index into the manager display list. */
    int child_index = -1;    /**< Group child index, or @c -1 for top-level. */
  };

  /**
   * @enum DropVisualMode
   * @brief Overlay drawn during drag-move feedback.
   */
  enum class DropVisualMode {
    kNone,         /**< No overlay. */
    kLineAbove,    /**< Horizontal insert line above the target row. */
    kLineBelow,    /**< Horizontal insert line below the target row. */
    kGroupTarget,  /**< Highlight target as a nest-into-group drop. */
    kInvalid,      /**< Show rejection styling. */
  };

  /**
   * @brief Walks up to the Display / DisplayChild row for @p item.
   *
   * @param item Any descendant property row, or a Display row itself.
   * @return Nearest display-row ancestor, or @c nullptr.
   */
  static QTreeWidgetItem* DisplayRowItem(QTreeWidgetItem* item);

  /**
   * @brief Computes the root insert index for a sibling-style drop.
   *
   * @param tree Tree being reordered.
   * @param target_item Row under the cursor.
   * @param position Qt drop indicator position.
   * @return Index suitable for inserting into the manager's root list.
   */
  static std::size_t RootInsertIndexForPosition(
      QTreeWidget* tree, QTreeWidgetItem* target_item,
      QAbstractItemView::DropIndicatorPosition position);

  /**
   * @brief Deserializes a @ref DisplayDragPayload from MIME data.
   *
   * @param mime Drag MIME data.
   * @param out Filled on success.
   * @return @c true when the payload was present and valid.
   */
  static bool ReadDragPayload(const QMimeData* mime, DisplayDragPayload* out);

  /**
   * @brief Serializes @p payload into newly allocated MIME data.
   *
   * @param payload Source display indices.
   * @return Heap-allocated MIME data (ownership transfers to the drag).
   */
  static QMimeData* MakeDragPayload(const DisplayDragPayload& payload);

  /**
   * @brief Maps a viewport position to Qt's drop indicator enum.
   *
   * @param pos Position in viewport coordinates.
   * @param item Item under the cursor.
   * @return AboveItem / OnItem / BelowItem.
   */
  QAbstractItemView::DropIndicatorPosition computeDropPosition(
      const QPoint& pos, QTreeWidgetItem* item) const;

  /**
   * @brief Resolves which Display row should receive a group-nest drop.
   *
   * @param item Item under the cursor.
   * @return Group target row, or @c nullptr if nesting is not applicable.
   */
  QTreeWidgetItem* resolveGroupDropTarget(QTreeWidgetItem* item) const;

  /**
   * @brief Returns whether @p source may be nested under @p group_row.
   *
   * Rejects self-drops and cycles.
   *
   * @param source Dragged display payload.
   * @param group_row Proposed parent Display row.
   * @return @c true when the nest is allowed.
   */
  bool canDropIntoGroup(const DisplayDragPayload& source,
                        QTreeWidgetItem* group_row) const;

  /**
   * @brief Rectangle used to paint the group-nest highlight.
   *
   * @param group_row Target Display row.
   * @return Highlight rect in viewport coordinates.
   */
  QRect groupDropHighlightRect(QTreeWidgetItem* group_row) const;

  /**
   * @brief Combined validity check for sibling insert or group nest.
   *
   * @param source Dragged display payload.
   * @param target_item Row under the cursor.
   * @param position Drop indicator position.
   * @return @c true when @ref applyDrop() would succeed.
   */
  bool canDropOn(const DisplayDragPayload& source, QTreeWidgetItem* target_item,
                 QAbstractItemView::DropIndicatorPosition position) const;

  /**
   * @brief Mutates VisualizationManager to apply the drop.
   *
   * @param source Dragged display payload.
   * @param target_item Row under the cursor.
   * @param position Drop indicator position.
   * @return @c true on success.
   */
  bool applyDrop(const DisplayDragPayload& source, QTreeWidgetItem* target_item,
                 QAbstractItemView::DropIndicatorPosition position);

  /**
   * @brief Clears highlight brushes and drop-line state (may touch items).
   */
  void clearDragVisuals();

  /**
   * @brief Clears drag state without touching tree items (safe after tree
   *        rebuild).
   */
  void resetDragVisualState();

  /**
   * @brief Updates @c drop_visual_mode_ / highlight rows for the current drag.
   *
   * @param source Dragged payload.
   * @param target_item Row under the cursor.
   * @param position Drop indicator position.
   * @param valid Whether the drop would be accepted.
   */
  void updateDragVisuals(const DisplayDragPayload& source,
                         QTreeWidgetItem* target_item,
                         QAbstractItemView::DropIndicatorPosition position,
                         bool valid);

  /**
   * @brief Builds a translucent pixmap for the drag hot-spot image.
   *
   * @param row Display row being dragged.
   * @return Pixmap sized to the row.
   */
  QPixmap makeDragPixmap(QTreeWidgetItem* row) const;

  /**
   * @brief Restores a row's background from @ref kDisplayTreeRoleDragBgSaved.
   *
   * @param item Previously highlighted item.
   */
  void restoreItemBackground(QTreeWidgetItem* item);

  /**
   * @brief Saves the current background and applies @p brush for drag feedback.
   *
   * @param item Row to highlight.
   * @param brush Temporary highlight brush.
   */
  void highlightItem(QTreeWidgetItem* item, const QBrush& brush);

  /**
   * @brief Starts a short timer to expand @p group_row while hovering during
   *        drag (RViz-style auto-expand).
   *
   * @param group_row Display row under a nest-into-group hover.
   */
  void scheduleGroupExpand(QTreeWidgetItem* group_row);

  /** Shared manager mutated on successful drops. */
  std::shared_ptr<common::VisualizationManager> manager_;

  /** Display row that initiated the current drag (visual only). */
  QTreeWidgetItem* drag_source_row_ = nullptr;

  /** Row currently highlighted as group-drop target. */
  QTreeWidgetItem* drop_highlight_row_ = nullptr;

  /** Group row pending auto-expand via @c hover_expand_timer_. */
  QTreeWidgetItem* hover_expand_item_ = nullptr;

  /** Viewport Y of the insert line, or @c -1 when none. */
  int drop_line_y_ = -1;

  /** Active overlay mode for @ref paintEvent(). */
  DropVisualMode drop_visual_mode_ = DropVisualMode::kNone;

  /** Optional label drawn inside the group-drop highlight. */
  QString group_drop_label_;

  /** Debounced timer for hover-expand during drag. */
  QTimer* hover_expand_timer_ = nullptr;
};

}  // namespace autoviz
