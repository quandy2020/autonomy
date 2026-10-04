/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file message_tree.hpp
 * @brief Protobuf message / schema tree helpers and the Raw Messages tree
 *        widget (expand, drag-to-plot, context menu).
 *
 * Free functions populate and incrementally update a @c QTreeWidget from a
 * @c google::protobuf::Message. @ref RawMessageTreeWidget adds interaction:
 * double-click toggles a full subtree, drag starts a Plot series MIME, and
 * the context menu can emit @ref RawMessageTreeWidget::addToPlotRequested().
 *
 * @see RawMessagesPanel
 * @see plot::PlotSeriesDragPayload
 */

#pragma once

#include <QSet>
#include <QString>
#include <QTreeWidget>

#include <optional>

#include "autoviz/ui/plot/plot_drag_mime.hpp"

class QContextMenuEvent;
class QMouseEvent;
class QTreeWidgetItem;

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace raw_messages {

/**
 * @class RawMessageTreeWidget
 * @brief Tree view: double-click a row to expand/collapse the full subtree.
 *
 * Used as the main content of @ref RawMessagesPanel. Numeric leaf rows can be
 * dragged onto a Plot panel via @ref plot::PlotSeriesDragPayload MIME.
 *
 * @note Call @ref setActiveChannel() whenever the parent panel's selected
 *       channel changes so drag payloads carry the correct channel name.
 *
 * @see PopulateMessageTree()
 * @see UpdateMessageTreeValues()
 */
class RawMessageTreeWidget : public QTreeWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs a tree with drag enabled and custom context menu.
   *
   * @param parent Qt parent (typically @ref RawMessagesPanel).
   */
  explicit RawMessageTreeWidget(QWidget* parent = nullptr);

  /**
   * @brief Sets the channel name embedded in Plot drag / add-to-plot payloads.
   *
   * @param channel Autolink channel currently inspected by the parent panel.
   */
  void setActiveChannel(const QString& channel);

  /**
   * @brief Enables leaf value change highlighting vs the previous frame.
   */
  void setDiffHighlightEnabled(bool enabled);

  /**
   * @brief Whether change highlighting is enabled.
   */
  bool diffHighlightEnabled() const { return diff_highlight_; }

 signals:
  /**
   * @brief Emitted when the user requests adding a field as a Plot series.
   *
   * @param channel Source channel (@c active_channel_).
   * @param field_path Dot-separated field path from the row.
   */
  void addToPlotRequested(const QString& channel, const QString& field_path);

  /**
   * @brief Requests copying the latest full message as JSON.
   */
  void copyMessageJsonRequested();

 protected:
  /**
   * @brief Toggles expand/collapse of the entire subtree under the clicked row.
   *
   * @param event Double-click event.
   */
  void mouseDoubleClickEvent(QMouseEvent* event) override;

  /**
   * @brief Starts a Plot-series drag when the row carries a numeric path.
   *
   * @param supported_actions Actions offered to the drag.
   */
  void startDrag(Qt::DropActions supported_actions) override;

  /**
   * @brief Context menu with "Add to Plot" for eligible numeric fields.
   *
   * @param event Context menu event.
   */
  void contextMenuEvent(QContextMenuEvent* event) override;

 private:
  /**
   * @brief Expands the subtree if collapsed; collapses if fully expanded.
   *
   * @param item Root of the subtree to toggle.
   */
  void toggleSubtree(QTreeWidgetItem* item);

  /**
   * @brief Builds a Plot drag payload from @p item when it is a numeric leaf.
   *
   * @param item Tree row under the cursor / selection.
   * @return Payload if the row is plottable; @c std::nullopt otherwise.
   */
  std::optional<plot::PlotSeriesDragPayload> payloadFromItem(
      QTreeWidgetItem* item) const;

  /**
   * @brief Emits @ref addToPlotRequested() for @p item when plottable.
   *
   * @param item Tree row chosen from the context menu.
   */
  void requestAddToPlot(QTreeWidgetItem* item);

  /** Channel name stamped into drag / add-to-plot requests. */
  QString active_channel_;

  /** Highlight leaves whose value changed since the previous update. */
  bool diff_highlight_ = true;
};

/**
 * @brief Recursively expands @p item and all descendants.
 *
 * @param item Subtree root.
 */
void ExpandSubtree(QTreeWidgetItem* item);

/**
 * @brief Recursively collapses @p item and all descendants.
 *
 * @param item Subtree root.
 */
void CollapseSubtree(QTreeWidgetItem* item);

/**
 * @brief Returns whether @p item and every descendant are expanded.
 *
 * @param item Subtree root.
 * @return @c true when the whole subtree is expanded.
 */
bool SubtreeFullyExpanded(QTreeWidgetItem* item);

/**
 * @brief Expands every item in @p tree.
 *
 * @param tree Target tree widget.
 */
void ExpandAllNodes(QTreeWidget* tree);

/**
 * @brief Collapses every item in @p tree.
 *
 * @param tree Target tree widget.
 */
void CollapseAllNodes(QTreeWidget* tree);

/**
 * @brief Foxglove-style collapsible tree for one protobuf message frame.
 *
 * Clears @p tree and rebuilds rows from @p message. When
 * @p apply_initial_expand is true, applies a shallow default expansion.
 *
 * @param tree Destination tree widget.
 * @param message Parsed protobuf message.
 * @param root_label Text for the root row (usually the type name).
 * @param channel Channel name stored on items for drag payloads.
 * @param path_filter Optional substring / path filter; empty shows all fields.
 * @param apply_initial_expand Whether to expand top-level groups initially.
 */
void PopulateMessageTree(QTreeWidget* tree, const google::protobuf::Message& message,
                         const QString& root_label, const QString& channel,
                         const QString& path_filter, bool apply_initial_expand = false);

/**
 * @brief Update leaf values without rebuilding tree structure (preserves
 *        expansion).
 *
 * @param tree Tree previously built by @ref PopulateMessageTree().
 * @param message New message with the same schema / filtered shape.
 * @param root_label Expected root label (mismatch → caller should rebuild).
 * @param path_filter Path filter that must match the existing tree.
 * @return @c true when values were updated in place; @c false if a full
 *         rebuild is required.
 */
bool UpdateMessageTreeValues(QTreeWidget* tree, const google::protobuf::Message& message,
                             const QString& root_label, const QString& path_filter,
                             bool highlight_changes = false);

/**
 * @brief Schema-only tree before the first payload arrives.
 *
 * Builds field names / types without values from the descriptor of
 * @p message_type.
 *
 * @param tree Destination tree widget.
 * @param message_type Fully-qualified protobuf type name.
 * @param channel Channel name stored on items for drag payloads.
 */
void PopulateSchemaTree(QTreeWidget* tree, const std::string& message_type,
                        const QString& channel);

/**
 * @brief Captures the set of expanded field paths in @p tree.
 *
 * @param tree Source tree.
 * @return Set of path strings suitable for @ref RestoreExpandedPaths().
 */
QSet<QString> CaptureExpandedPaths(QTreeWidget* tree);

/**
 * @brief Restores expansion state from paths captured earlier.
 *
 * @param tree Target tree (same schema / paths).
 * @param expanded_paths Paths previously returned by @ref CaptureExpandedPaths().
 */
void RestoreExpandedPaths(QTreeWidget* tree, const QSet<QString>& expanded_paths);

}  // namespace raw_messages
}  // namespace autoviz
