/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"
#include "autoviz/ui/behavior_tree/bt_xml_io.hpp"

#include <QGraphicsView>
#include <QHash>
#include <QPoint>
#include <QSet>
#include <QString>
#include <QVector>

#include <optional>
#include <functional>

class QContextMenuEvent;
class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QKeyEvent;
class QMimeData;
class QMouseEvent;
class QWheelEvent;

class QGraphicsScene;
class QTimer;

namespace autoviz {
namespace behavior_tree {

constexpr char kBtNodeDragMime[] = "application/x-autoviz-bt-node";

/** Interactive behavior-tree graph canvas for editing and monitoring. */
class BtGraphView : public QGraphicsView {
  Q_OBJECT

 public:
  explicit BtGraphView(QWidget* parent = nullptr);

  void setTree(BtAbsTree tree, const QHash<QString, BtNodeModel>& models);
  BtAbsTree tree() const;
  /** Tree suitable for persistence (expanded inline nodes stripped). */
  BtAbsTree serializableTree() const;

  using SubTreeLookup = std::function<const BtAbsTree*(const QString& tree_id)>;
  void setSubTreeLookup(SubTreeLookup lookup);

  bool toggleSubTreeExpand(int uid);
  bool expandSubTree(int uid);
  bool collapseSubTree(int uid);
  void refreshExpandedSubTree(const QString& subtree_id);
  void collapseExpandedSubTreesById(const QString& subtree_id);

  void setReadOnly(bool read_only);
  bool isReadOnly() const { return read_only_; }

  void setNodeStatus(int uid, BtNodeStatus status,
                     std::optional<BtNodeStatus> previous = std::nullopt,
                     bool refresh_flow = true);
  void clearStatuses();
  /** Relight full root→RUNNING paths (nodes + edges) for live signal flow. */
  void refreshSignalFlow();
  /** Advance dash offset for RUNNING edges (data-flow animation). */
  void setFlowAnimationEnabled(bool enabled);

  /** Apply Groot2 Node Editor fields (instance / scripts / description). */
  bool applyNodeEditor(int uid, const QString& instance_name, const QString& skip_if,
                       const QString& success_if, const QString& failure_if,
                       const QString& while_script, const QString& on_success,
                       const QString& on_failure, const QString& on_halted,
                       const QString& post_script, const QString& description);

  void fitToView();
  void applyAutoLayout(bool horizontal = false);
  bool exportSvg(const QString& path) const;

  int selectedUid() const;
  void deleteSelected();
  void smartRemoveSelected();
  void morphSelected(const QString& registration_id);
  std::optional<BtAbsTree> extractSubtree(int uid, const QString& subtree_id);
  void addChildNode(int parent_uid, const QString& registration_id, BtNodeKind kind);

  void setHighlightedPortValue(const QString& value);
  void clearPortHighlight();

  /** Highlight nodes that currently have a hook/breakpoint configured. */
  void setHookNodeUids(const QSet<int>& uids);
  void clearHookMarkers();

  bool canUndo() const;
  bool canRedo() const;
  void undo();
  void redo();

 signals:
  void treeChanged();
  void selectionChanged(int uid);
  void nodeDoubleClicked(int uid);
  void subtreeExpandRequested(const QString& tree_id);
  void subTreeExpandToggled(int uid, bool expanded);
  void createSubtreeRequested(int uid);
  void portValueHighlightRequested(const QString& value);
  void configureHookRequested(int uid);

 protected:
  void drawBackground(QPainter* painter, const QRectF& rect) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private:
  void rebuildScene();
  void syncPositionsFromScene();
  void pushSnapshot();
  void restoreSnapshot(const BtAbsTree& snapshot);
  void emitSelectionFromScene();
  void removeSubtree(int uid);
  void removeNodeKeepChildren(int uid);
  int resolveDropParentUid() const;
  QStringList morphCandidates(const BtAbsNode& node) const;
  bool appendSubTreeInline(int subtree_uid, const BtAbsTree& source);
  void removeExpandedInline(int subtree_uid);
  bool nodeIsLocked(const BtAbsNode& node) const;

  QGraphicsScene* scene_ = nullptr;
  BtAbsTree tree_;
  QHash<QString, BtNodeModel> models_;
  SubTreeLookup subtree_lookup_;
  QVector<BtAbsTree> undo_stack_;
  QVector<BtAbsTree> redo_stack_;
  QString highlighted_port_value_;
  QSet<int> hook_node_uids_;
  bool read_only_ = false;
  bool horizontal_layout_ = false;
  bool restoring_snapshot_ = false;
  bool space_pan_ = false;
  bool panning_ = false;
  bool move_snapshot_pending_ = false;
  QPoint last_pan_pos_;
  double zoom_factor_ = 1.0;
  double flow_phase_ = 0.0;
  QTimer* flow_timer_ = nullptr;
};

QMimeData* MakeBtNodeDragPayload(const QString& registration_id, BtNodeKind kind);
bool ReadBtNodeDragPayload(const QMimeData* mime, QString* registration_id, BtNodeKind* kind);

}  // namespace behavior_tree
}  // namespace autoviz
