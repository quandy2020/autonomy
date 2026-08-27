/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_hook.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace autoviz {
namespace behavior_tree {
namespace {

int ModeToInt(BtHookMode mode) {
  return static_cast<int>(mode);
}

BtHookMode ModeFromInt(int value) {
  if (value == static_cast<int>(BtHookMode::kReplace)) {
    return BtHookMode::kReplace;
  }
  if (value == static_cast<int>(BtHookMode::kBreakpoint)) {
    return BtHookMode::kBreakpoint;
  }
  return BtHookMode::kNone;
}

}  // namespace

bool SaveHooksToFile(const BtHookMap& hooks, const QString& path) {
  QJsonArray array;
  for (auto it = hooks.constBegin(); it != hooks.constEnd(); ++it) {
    const BtHook& hook = it.value();
    if (hook.mode == BtHookMode::kNone || hook.node_name.isEmpty()) {
      continue;
    }
    QJsonObject obj;
    obj.insert(QStringLiteral("enabled"), hook.enabled);
    obj.insert(QStringLiteral("uid"), hook.node_uid);
    obj.insert(QStringLiteral("node_name"), hook.node_name);
    obj.insert(QStringLiteral("mode"), ModeToInt(hook.mode));
    obj.insert(QStringLiteral("once"), hook.once);
    obj.insert(QStringLiteral("desired_status"), StatusToString(hook.desired_status));
    obj.insert(QStringLiteral("position"), static_cast<int>(hook.position));
    array.append(obj);
  }

  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }
  file.write(QJsonDocument(array).toJson(QJsonDocument::Indented));
  return true;
}

bool LoadHooksFromFile(const QString& path, BtHookMap* hooks) {
  if (hooks == nullptr) {
    return false;
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  if (!doc.isArray()) {
    return false;
  }

  BtHookMap loaded;
  const QJsonArray array = doc.array();
  for (const QJsonValue& value : array) {
    if (!value.isObject()) {
      continue;
    }
    const QJsonObject obj = value.toObject();
    BtHook hook;
    hook.enabled = obj.value(QStringLiteral("enabled")).toBool(true);
    hook.node_uid = obj.value(QStringLiteral("uid")).toInt(-1);
    hook.node_name = obj.value(QStringLiteral("node_name")).toString();
    hook.mode = ModeFromInt(obj.value(QStringLiteral("mode")).toInt(0));
    hook.once = obj.value(QStringLiteral("once")).toBool(false);
    hook.position = static_cast<BtHookPosition>(
        obj.value(QStringLiteral("position")).toInt(static_cast<int>(BtHookPosition::kPost)));
    const auto status =
        StatusFromString(obj.value(QStringLiteral("desired_status")).toString());
    if (status.has_value()) {
      hook.desired_status = *status;
    }
    if (hook.node_name.isEmpty() || hook.mode == BtHookMode::kNone) {
      continue;
    }
    loaded.insert(hook.node_name, hook);
  }
  *hooks = std::move(loaded);
  return true;
}

}  // namespace behavior_tree
}  // namespace autoviz
