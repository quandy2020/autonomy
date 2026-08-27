/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QHash>
#include <QString>
#include <QVector>

namespace autoviz {
namespace behavior_tree {

/** Aligns with BT::Monitor::Hook::Mode (Groot2 protocol). */
enum class BtHookMode {
  kNone = -1,
  kBreakpoint = 0,
  kReplace = 1,
};

/** Aligns with BT::Monitor::Hook::Position. */
enum class BtHookPosition {
  kPre = 0,
  kPost = 1,
};

/** Local hook/breakpoint entry (Groot2 Hook Config semantics). */
struct BtHook {
  int node_uid = -1;
  QString node_name;
  BtHookMode mode = BtHookMode::kBreakpoint;
  BtHookPosition position = BtHookPosition::kPost;
  BtNodeStatus desired_status = BtNodeStatus::kSuccess;
  bool once = false;
  bool enabled = true;
};

inline QString HookModeToString(BtHookMode mode) {
  switch (mode) {
    case BtHookMode::kBreakpoint:
      return QStringLiteral("Breakpoint");
    case BtHookMode::kReplace:
      return QStringLiteral("Replace");
    case BtHookMode::kNone:
    default:
      return QStringLiteral("None");
  }
}

inline QString HookPositionToString(BtHookPosition position) {
  return position == BtHookPosition::kPre ? QStringLiteral("PRE")
                                          : QStringLiteral("POST");
}

inline QString HookSummary(const BtHook& hook) {
  return QStringLiteral("%1 [%2] %3%4")
      .arg(hook.node_name,
           HookModeToString(hook.mode),
           HookPositionToString(hook.position),
           hook.once ? QStringLiteral(" once") : QString());
}

/** Hooks keyed by node instance name (matches BehaviorTreeLog.node_name). */
using BtHookMap = QHash<QString, BtHook>;

bool SaveHooksToFile(const BtHookMap& hooks, const QString& path);
bool LoadHooksFromFile(const QString& path, BtHookMap* hooks);

}  // namespace behavior_tree
}  // namespace autoviz
