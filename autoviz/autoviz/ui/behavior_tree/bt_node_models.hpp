/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QHash>
#include <QPointF>
#include <QSet>
#include <QString>
#include <QVector>

#include <optional>

class QWidget;

namespace autoviz {
namespace behavior_tree {

enum class BtNodeKind {
  kAction,
  kCondition,
  kControl,
  kDecorator,
  kSubTree,
  kRoot,
  kUndefined,
};

enum class BtPortDirection {
  kInput,
  kOutput,
  kInOut,
};

enum class BtNodeStatus {
  kIdle,
  kRunning,
  kSuccess,
  kFailure,
};

struct BtPortModel {
  QString name;
  QString type_name;
  QString description;
  QString default_value;
  BtPortDirection direction = BtPortDirection::kInput;
};

struct BtNodeModel {
  BtNodeKind kind = BtNodeKind::kUndefined;
  QString registration_id;
  QVector<BtPortModel> ports;
};

struct BtAbsNode {
  int uid = 0;
  QString registration_id;
  QString instance_name;
  BtNodeKind kind = BtNodeKind::kUndefined;
  QHash<QString, QString> port_remap;
  QPointF pos;
  BtNodeStatus status = BtNodeStatus::kIdle;
  BtNodeStatus prev_status = BtNodeStatus::kIdle;
  /** SubTree node is visually expanded with inlined children. */
  bool subtree_expanded = false;
  /** Node was inlined from an expanded SubTree (view-only, excluded on save). */
  bool expanded_inline = false;
  /** Owning SubTree node uid when expanded_inline is true. */
  int expanded_owner_uid = -1;
  /** Groot2 / BT.CPP4 script preconditions. */
  QString skip_if;
  QString success_if;
  QString failure_if;
  QString while_script;
  /** Groot2 / BT.CPP4 script postconditions. */
  QString on_success;
  QString on_failure;
  QString on_halted;
  QString post_script;
  /** Node comment shown as yellow bubble when non-empty. */
  QString description;
  QVector<int> children;
};

struct BtAbsTree {
  QString tree_id;
  int root_uid = -1;
  QHash<int, BtAbsNode> nodes;
};

struct BtDocument {
  QString main_tree_id;
  QString source_path;
  QHash<QString, BtAbsTree> trees;
  QHash<QString, BtNodeModel> models;
};

BtNodeKind BtNodeKindFromTag(const QString& tag);
QString TagFromBtNodeKind(BtNodeKind kind);

QString StatusToString(BtNodeStatus status);
std::optional<BtNodeStatus> StatusFromString(const QString& text);

QString KindDisplayName(BtNodeKind kind);

QColor ColorForStatus(BtNodeStatus status);
QColor ColorForKind(BtNodeKind kind);

QHash<QString, BtNodeModel> BuiltinNodeModels();

bool IsBuiltinModel(const QString& registration_id);
QHash<QString, BtNodeModel> MergeWithBuiltinModels(const QHash<QString, BtNodeModel>& models);
/** Re-apply model kinds after manifests load (XML parse may lack TreeNodesModel). */
void ReresolveTreeNodeKinds(BtAbsTree& tree, const QHash<QString, BtNodeModel>& models);
QSet<QString> CustomModelsToRemoveOnImport(QWidget* parent,
                                             const QHash<QString, BtNodeModel>& current_models,
                                             const QHash<QString, BtNodeModel>& imported_models);
QString FindTreeUsingModel(const QHash<QString, BtAbsTree>& trees, const QString& model_id);
void RenameModelInAllTrees(QHash<QString, BtAbsTree>& trees, const QString& old_id,
                           const QString& new_id);
BtNodeModel MakeSubTreeModel(const QString& registration_id);
BtAbsTree MakeEmptySubTree(const QString& tree_id);

bool IsValidBehaviorTree(const BtAbsTree& tree);
QString SubTreeIdFromNode(const BtAbsNode& node);
BtAbsTree StripExpandedInlineNodes(const BtAbsTree& tree);
QVector<int> CollectExpandedInlineNodes(const BtAbsTree& tree, int owner_uid);

int NextUid(BtAbsTree& tree);
QVector<int> CollectSubtree(const BtAbsTree& tree, int uid);
int FindParentUid(const BtAbsTree& tree, int uid);
bool KindAcceptsChildren(BtNodeKind kind);
int MaxChildCount(BtNodeKind kind);

}  // namespace behavior_tree
}  // namespace autoviz
