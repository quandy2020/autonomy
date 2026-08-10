/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

/** Tree view: double-click a row to expand/collapse the full subtree. */
class RawMessageTreeWidget : public QTreeWidget {
  Q_OBJECT

 public:
  explicit RawMessageTreeWidget(QWidget* parent = nullptr);

  void setActiveChannel(const QString& channel);

 signals:
  void addToPlotRequested(const QString& channel, const QString& field_path);

 protected:
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void startDrag(Qt::DropActions supported_actions) override;
  void contextMenuEvent(QContextMenuEvent* event) override;

 private:
  void toggleSubtree(QTreeWidgetItem* item);
  std::optional<plot::PlotSeriesDragPayload> payloadFromItem(
      QTreeWidgetItem* item) const;
  void requestAddToPlot(QTreeWidgetItem* item);

  QString active_channel_;
};

void ExpandSubtree(QTreeWidgetItem* item);
void CollapseSubtree(QTreeWidgetItem* item);
bool SubtreeFullyExpanded(QTreeWidgetItem* item);

void ExpandAllNodes(QTreeWidget* tree);
void CollapseAllNodes(QTreeWidget* tree);

/** Foxglove-style collapsible tree for one protobuf message frame. */
void PopulateMessageTree(QTreeWidget* tree, const google::protobuf::Message& message,
                         const QString& root_label, const QString& channel,
                         const QString& path_filter, bool apply_initial_expand = false);

/** Update leaf values without rebuilding tree structure (preserves expansion). */
bool UpdateMessageTreeValues(QTreeWidget* tree, const google::protobuf::Message& message,
                             const QString& root_label, const QString& path_filter);

/** Schema-only tree before the first payload arrives. */
void PopulateSchemaTree(QTreeWidget* tree, const std::string& message_type,
                        const QString& channel);

QSet<QString> CaptureExpandedPaths(QTreeWidget* tree);
void RestoreExpandedPaths(QTreeWidget* tree, const QSet<QString>& expanded_paths);

}  // namespace raw_messages
}  // namespace autoviz
