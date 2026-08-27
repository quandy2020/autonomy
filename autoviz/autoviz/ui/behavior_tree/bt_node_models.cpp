/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QMessageBox>

#include <algorithm>

namespace autoviz {
namespace behavior_tree {
namespace {

BtPortModel InputPort(const QString& name, const QString& type_name = QStringLiteral("string"),
                      const QString& description = {}, const QString& default_value = {}) {
  return BtPortModel{name, type_name, description, default_value, BtPortDirection::kInput};
}

BtPortModel OutputPort(const QString& name, const QString& type_name = QStringLiteral("string"),
                       const QString& description = {}) {
  return BtPortModel{name, type_name, description, {}, BtPortDirection::kOutput};
}

BtNodeModel ControlModel(const QString& id, const QVector<BtPortModel>& ports = {}) {
  return BtNodeModel{BtNodeKind::kControl, id, ports};
}

BtNodeModel DecoratorModel(const QString& id, const QVector<BtPortModel>& ports = {}) {
  return BtNodeModel{BtNodeKind::kDecorator, id, ports};
}

void AddModel(QHash<QString, BtNodeModel>& models, BtNodeModel model) {
  models.insert(model.registration_id, std::move(model));
}

}  // namespace

BtNodeKind BtNodeKindFromTag(const QString& tag) {
  if (tag == QLatin1String("Action")) {
    return BtNodeKind::kAction;
  }
  if (tag == QLatin1String("Condition")) {
    return BtNodeKind::kCondition;
  }
  if (tag == QLatin1String("Control")) {
    return BtNodeKind::kControl;
  }
  if (tag == QLatin1String("Decorator")) {
    return BtNodeKind::kDecorator;
  }
  if (tag == QLatin1String("SubTree")) {
    return BtNodeKind::kSubTree;
  }
  if (tag == QLatin1String("root")) {
    return BtNodeKind::kRoot;
  }
  return BtNodeKind::kUndefined;
}

QString TagFromBtNodeKind(BtNodeKind kind) {
  switch (kind) {
    case BtNodeKind::kAction:
      return QStringLiteral("Action");
    case BtNodeKind::kCondition:
      return QStringLiteral("Condition");
    case BtNodeKind::kControl:
      return QStringLiteral("Control");
    case BtNodeKind::kDecorator:
      return QStringLiteral("Decorator");
    case BtNodeKind::kSubTree:
      return QStringLiteral("SubTree");
    case BtNodeKind::kRoot:
      return QStringLiteral("root");
    case BtNodeKind::kUndefined:
      break;
  }
  return QStringLiteral("Action");
}

QString StatusToString(BtNodeStatus status) {
  switch (status) {
    case BtNodeStatus::kIdle:
      return QStringLiteral("IDLE");
    case BtNodeStatus::kRunning:
      return QStringLiteral("RUNNING");
    case BtNodeStatus::kSuccess:
      return QStringLiteral("SUCCESS");
    case BtNodeStatus::kFailure:
      return QStringLiteral("FAILURE");
  }
  return QStringLiteral("IDLE");
}

std::optional<BtNodeStatus> StatusFromString(const QString& text) {
  const QString upper = text.trimmed().toUpper();
  if (upper == QLatin1String("IDLE")) {
    return BtNodeStatus::kIdle;
  }
  if (upper == QLatin1String("RUNNING")) {
    return BtNodeStatus::kRunning;
  }
  if (upper == QLatin1String("SUCCESS")) {
    return BtNodeStatus::kSuccess;
  }
  if (upper == QLatin1String("FAILURE")) {
    return BtNodeStatus::kFailure;
  }
  return std::nullopt;
}

QString KindDisplayName(BtNodeKind kind) {
  switch (kind) {
    case BtNodeKind::kAction:
      return QStringLiteral("Action");
    case BtNodeKind::kCondition:
      return QStringLiteral("Condition");
    case BtNodeKind::kControl:
      return QStringLiteral("Control");
    case BtNodeKind::kDecorator:
      return QStringLiteral("Decorator");
    case BtNodeKind::kSubTree:
      return QStringLiteral("SubTree");
    case BtNodeKind::kRoot:
      return QStringLiteral("Root");
    case BtNodeKind::kUndefined:
    default:
      return QStringLiteral("Undefined");
  }
}

QColor ColorForStatus(BtNodeStatus status) {
  switch (status) {
    case BtNodeStatus::kIdle:
      return QColor(160, 160, 160);
    case BtNodeStatus::kRunning:
      return QColor(0, 200, 83);
    case BtNodeStatus::kSuccess:
      return QColor(76, 175, 80);
    case BtNodeStatus::kFailure:
      return QColor(244, 67, 54);
  }
  return QColor(160, 160, 160);
}

QColor ColorForKind(BtNodeKind kind) {
  switch (kind) {
    case BtNodeKind::kAction:
      return QColor(33, 150, 243);
    case BtNodeKind::kCondition:
      return QColor(156, 39, 176);
    case BtNodeKind::kControl:
      return QColor(63, 81, 181);
    case BtNodeKind::kDecorator:
      return QColor(0, 150, 136);
    case BtNodeKind::kSubTree:
      return QColor(121, 85, 72);
    case BtNodeKind::kRoot:
      return QColor(96, 125, 139);
    case BtNodeKind::kUndefined:
      break;
  }
  return QColor(158, 158, 158);
}

QHash<QString, BtNodeModel> BuiltinNodeModels() {
  QHash<QString, BtNodeModel> models;

  AddModel(models, ControlModel(QStringLiteral("Sequence")));
  AddModel(models, ControlModel(QStringLiteral("ReactiveSequence")));
  AddModel(models, ControlModel(QStringLiteral("SequenceWithMemory")));
  AddModel(models, ControlModel(QStringLiteral("Fallback")));
  AddModel(models, ControlModel(QStringLiteral("ReactiveFallback")));
  AddModel(models, ControlModel(QStringLiteral("IfThenElse")));
  AddModel(models, ControlModel(QStringLiteral("WhileDoElse")));
  AddModel(models, ControlModel(QStringLiteral("Parallel"), {
      InputPort(QStringLiteral("success_threshold"), QStringLiteral("int"),
                QStringLiteral("number of child successes required")),
      InputPort(QStringLiteral("failure_threshold"), QStringLiteral("int"),
                QStringLiteral("number of child failures required")),
  }));
  AddModel(models, ControlModel(QStringLiteral("ParallelAll"), {
      InputPort(QStringLiteral("max_failures"), QStringLiteral("int"),
                QStringLiteral("max allowed child failures")),
  }));

  AddModel(models, DecoratorModel(QStringLiteral("ForceSuccess")));
  AddModel(models, DecoratorModel(QStringLiteral("ForceFailure")));
  AddModel(models, DecoratorModel(QStringLiteral("Inverter")));
  AddModel(models, DecoratorModel(QStringLiteral("RetryUntilSuccessful"), {
      InputPort(QStringLiteral("num_attempts"), QStringLiteral("int"),
                QStringLiteral("max retry attempts"), QStringLiteral("1")),
  }));
  AddModel(models, DecoratorModel(QStringLiteral("Retry"), {
      InputPort(QStringLiteral("num_attempts"), QStringLiteral("int"),
                QStringLiteral("max retry attempts"), QStringLiteral("1")),
  }));
  AddModel(models, DecoratorModel(QStringLiteral("KeepRunningUntilFailure")));
  AddModel(models, DecoratorModel(QStringLiteral("Repeat"), {
      InputPort(QStringLiteral("num_cycles"), QStringLiteral("int"),
                QStringLiteral("repeat count"), QStringLiteral("1")),
  }));
  AddModel(models, DecoratorModel(QStringLiteral("Delay"), {
      InputPort(QStringLiteral("delay_msec"), QStringLiteral("int"),
                QStringLiteral("delay in milliseconds"), QStringLiteral("0")),
  }));
  AddModel(models, DecoratorModel(QStringLiteral("RunOnce")));
  AddModel(models, DecoratorModel(QStringLiteral("Timeout"), {
      InputPort(QStringLiteral("msec"), QStringLiteral("int"),
                QStringLiteral("timeout in milliseconds"), QStringLiteral("0")),
  }));

  AddModel(models, BtNodeModel{
      BtNodeKind::kCondition,
      QStringLiteral("BlackboardCheckBool"),
      {
          InputPort(QStringLiteral("value_A"), QStringLiteral("bool")),
          InputPort(QStringLiteral("value_B"), QStringLiteral("bool")),
      },
  });
  AddModel(models, BtNodeModel{
      BtNodeKind::kCondition,
      QStringLiteral("BlackboardCheckString"),
      {
          InputPort(QStringLiteral("value_A"), QStringLiteral("string")),
          InputPort(QStringLiteral("value_B"), QStringLiteral("string")),
      },
  });
  AddModel(models, BtNodeModel{
      BtNodeKind::kCondition,
      QStringLiteral("BlackboardCheckInt"),
      {
          InputPort(QStringLiteral("value_A"), QStringLiteral("int")),
          InputPort(QStringLiteral("value_B"), QStringLiteral("int")),
      },
  });
  AddModel(models, BtNodeModel{
      BtNodeKind::kCondition,
      QStringLiteral("BlackboardCheckDouble"),
      {
          InputPort(QStringLiteral("value_A"), QStringLiteral("double")),
          InputPort(QStringLiteral("value_B"), QStringLiteral("double")),
      },
  });

  AddModel(models, BtNodeModel{
      BtNodeKind::kAction,
      QStringLiteral("Script"),
      {InputPort(QStringLiteral("code"), QStringLiteral("string"),
                 QStringLiteral("script body"))},
  });
  AddModel(models, BtNodeModel{
      BtNodeKind::kAction,
      QStringLiteral("SetBlackboard"),
      {
          InputPort(QStringLiteral("value"), QStringLiteral("string")),
          OutputPort(QStringLiteral("output_key"), QStringLiteral("string")),
      },
  });
  AddModel(models, BtNodeModel{BtNodeKind::kAction, QStringLiteral("AlwaysSuccess")});
  AddModel(models, BtNodeModel{BtNodeKind::kAction, QStringLiteral("AlwaysFailure")});

  AddModel(models, BtNodeModel{
      BtNodeKind::kSubTree,
      QStringLiteral("SubTree"),
      {InputPort(QStringLiteral("ID"), QStringLiteral("string"),
                 QStringLiteral("referenced behavior tree id"))},
  });

  return models;
}

int NextUid(BtAbsTree& tree) {
  int max_uid = -1;
  for (auto it = tree.nodes.constBegin(); it != tree.nodes.constEnd(); ++it) {
    max_uid = qMax(max_uid, it.key());
  }
  return max_uid + 1;
}

QVector<int> CollectSubtree(const BtAbsTree& tree, int uid) {
  QVector<int> collected;
  if (!tree.nodes.contains(uid)) {
    return collected;
  }

  QVector<int> stack{uid};
  while (!stack.isEmpty()) {
    const int current = stack.takeLast();
    if (collected.contains(current)) {
      continue;
    }
    collected.push_back(current);
    const BtAbsNode& node = tree.nodes.value(current);
    for (int child_uid : node.children) {
      stack.push_back(child_uid);
    }
  }
  return collected;
}

int FindParentUid(const BtAbsTree& tree, int uid) {
  for (auto it = tree.nodes.constBegin(); it != tree.nodes.constEnd(); ++it) {
    if (it.value().children.contains(uid)) {
      return it.key();
    }
  }
  return -1;
}

bool KindAcceptsChildren(BtNodeKind kind) {
  return kind == BtNodeKind::kControl || kind == BtNodeKind::kDecorator ||
         kind == BtNodeKind::kRoot;
}

int MaxChildCount(BtNodeKind kind) {
  switch (kind) {
    case BtNodeKind::kDecorator:
      return 1;
    case BtNodeKind::kControl:
    case BtNodeKind::kRoot:
      return 1024;
    default:
      return 0;
  }
}

bool IsBuiltinModel(const QString& registration_id) {
  return BuiltinNodeModels().contains(registration_id);
}

QHash<QString, BtNodeModel> MergeWithBuiltinModels(const QHash<QString, BtNodeModel>& models) {
  QHash<QString, BtNodeModel> merged = BuiltinNodeModels();
  for (auto it = models.constBegin(); it != models.constEnd(); ++it) {
    if (!IsBuiltinModel(it.key())) {
      merged.insert(it.key(), it.value());
    }
  }
  return merged;
}

void ReresolveTreeNodeKinds(BtAbsTree& tree, const QHash<QString, BtNodeModel>& models) {
  for (auto it = tree.nodes.begin(); it != tree.nodes.end(); ++it) {
    BtAbsNode& node = it.value();
    if (node.kind == BtNodeKind::kRoot ||
        node.registration_id == QLatin1String("Root")) {
      node.kind = BtNodeKind::kRoot;
      continue;
    }
    if (node.registration_id == QLatin1String("SubTree") ||
        node.kind == BtNodeKind::kSubTree) {
      node.kind = BtNodeKind::kSubTree;
      continue;
    }
    auto model_it = models.constFind(node.registration_id);
    if (model_it != models.constEnd()) {
      node.kind = model_it.value().kind;
    }
  }
}

QSet<QString> CustomModelsToRemoveOnImport(QWidget* parent,
                                           const QHash<QString, BtNodeModel>& current_models,
                                           const QHash<QString, BtNodeModel>& imported_models) {
  QSet<QString> stale_custom_models;
  for (auto it = current_models.constBegin(); it != current_models.constEnd(); ++it) {
    const QString& model_name = it.key();
    if (!IsBuiltinModel(model_name) && !imported_models.contains(model_name)) {
      stale_custom_models.insert(model_name);
    }
  }

  if (!stale_custom_models.isEmpty() && parent != nullptr) {
    const int ret = QMessageBox::question(
        parent, QStringLiteral("Clear Palette?"),
        QStringLiteral("Do you want to remove the previously loaded custom nodes?"),
        QMessageBox::No | QMessageBox::Yes, QMessageBox::No);
    if (ret != QMessageBox::Yes) {
      stale_custom_models.clear();
    }
  }
  return stale_custom_models;
}

QString FindTreeUsingModel(const QHash<QString, BtAbsTree>& trees, const QString& model_id) {
  for (auto tree_it = trees.constBegin(); tree_it != trees.constEnd(); ++tree_it) {
    for (auto node_it = tree_it->nodes.constBegin(); node_it != tree_it->nodes.constEnd();
         ++node_it) {
      if (node_it->registration_id == model_id) {
        return tree_it.key();
      }
      if (node_it->kind == BtNodeKind::kSubTree &&
          node_it->port_remap.value(QStringLiteral("ID")) == model_id) {
        return tree_it.key();
      }
    }
  }
  return {};
}

void RenameModelInAllTrees(QHash<QString, BtAbsTree>& trees, const QString& old_id,
                           const QString& new_id) {
  for (auto tree_it = trees.begin(); tree_it != trees.end(); ++tree_it) {
    for (auto node_it = tree_it->nodes.begin(); node_it != tree_it->nodes.end(); ++node_it) {
      if (node_it->registration_id == old_id) {
        node_it->registration_id = new_id;
        if (node_it->instance_name == old_id) {
          node_it->instance_name = new_id;
        }
      }
      if (node_it->kind == BtNodeKind::kSubTree &&
          node_it->port_remap.value(QStringLiteral("ID")) == old_id) {
        node_it->port_remap.insert(QStringLiteral("ID"), new_id);
        if (node_it->instance_name == old_id) {
          node_it->instance_name = new_id;
        }
      }
    }
  }
}

BtNodeModel MakeSubTreeModel(const QString& registration_id) {
  BtNodeModel model;
  model.kind = BtNodeKind::kSubTree;
  model.registration_id = registration_id;
  model.ports.push_back(BtPortModel{
      QStringLiteral("__shared_blackboard"), QStringLiteral("bool"),
      QStringLiteral("If false (default), the Subtree has an isolated blackboard and needs port "
                     "remapping"),
      QStringLiteral("false"), BtPortDirection::kInput});
  return model;
}

BtAbsTree MakeEmptySubTree(const QString& tree_id) {
  BtAbsTree tree;
  tree.tree_id = tree_id;
  BtAbsNode root;
  root.uid = NextUid(tree);
  root.registration_id = QStringLiteral("Root");
  root.kind = BtNodeKind::kRoot;
  root.instance_name = QStringLiteral("RootTree");
  tree.nodes.insert(root.uid, root);
  tree.root_uid = root.uid;
  return tree;
}

bool IsValidBehaviorTree(const BtAbsTree& tree) {
  if (tree.root_uid < 0 || tree.nodes.isEmpty()) {
    return false;
  }
  const BtAbsNode& root = tree.nodes.value(tree.root_uid);
  if (root.registration_id == QLatin1String("Root")) {
    return !root.children.isEmpty();
  }
  return true;
}

QString SubTreeIdFromNode(const BtAbsNode& node) {
  if (node.kind != BtNodeKind::kSubTree &&
      node.registration_id != QLatin1String("SubTree")) {
    return {};
  }
  return node.port_remap.value(QStringLiteral("ID")).trimmed();
}

QVector<int> CollectExpandedInlineNodes(const BtAbsTree& tree, int owner_uid) {
  QVector<int> result;
  for (auto it = tree.nodes.constBegin(); it != tree.nodes.constEnd(); ++it) {
    if (it.value().expanded_inline && it.value().expanded_owner_uid == owner_uid) {
      result.push_back(it.key());
    }
  }
  return result;
}

BtAbsTree StripExpandedInlineNodes(const BtAbsTree& tree) {
  BtAbsTree result = tree;
  QSet<int> inline_uids;
  for (auto it = result.nodes.constBegin(); it != result.nodes.constEnd(); ++it) {
    if (it.value().expanded_inline) {
      inline_uids.insert(it.key());
    }
  }

  for (auto it = result.nodes.begin(); it != result.nodes.end(); ++it) {
    if (it.value().subtree_expanded) {
      it.value().subtree_expanded = false;
      QVector<int> kept;
      kept.reserve(it.value().children.size());
      for (int child_uid : it.value().children) {
        if (!inline_uids.contains(child_uid)) {
          kept.push_back(child_uid);
        }
      }
      it.value().children = kept;
    }
    QVector<int>& children = it.value().children;
    children.erase(std::remove_if(children.begin(), children.end(),
                                    [&inline_uids](int child_uid) {
                                      return inline_uids.contains(child_uid);
                                    }),
                   children.end());
  }

  for (int uid : inline_uids) {
    result.nodes.remove(uid);
  }
  return result;
}

}  // namespace behavior_tree
}  // namespace autoviz
