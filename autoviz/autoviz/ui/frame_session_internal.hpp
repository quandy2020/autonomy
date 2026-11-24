/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_session_internal.hpp
 * @brief TU-private helpers shared by the FrameSession translation units.
 *
 * These live in a header (instead of an anonymous namespace) so the config /
 * timer / drag-and-drop TUs can use one definition without duplicating logic.
 */

#pragma once

#include <algorithm>

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QSet>
#include <QString>

#include "autoviz/ui/app/preferences.hpp"

namespace autoviz {
namespace frame_session_detail {

/** Force one paint after this many skipped ticks while unexposed. */
inline constexpr int kForcePaintAfterOccludedTicks = 90;

/** Clamps an idle render divisor into the persisted preference range. */
inline int SanitizedIdleDivisor(int divisor) {
  return std::clamp(divisor, kMinIdleRenderDivisor, kMaxIdleRenderDivisor);
}

/** Collects every leaf dock name from a mosaic:v1 root object. */
inline void CollectMosaicLeafNames(const QJsonObject& node,
                                   QSet<QString>* names) {
  if (names == nullptr || node.isEmpty()) {
    return;
  }
  const QString type = node.value(QStringLiteral("type")).toString();
  if (type == QLatin1String("leaf")) {
    const QString name = node.value(QStringLiteral("name")).toString();
    if (!name.isEmpty()) {
      names->insert(name);
    }
    return;
  }
  if (type != QLatin1String("split")) {
    return;
  }
  const QJsonArray children = node.value(QStringLiteral("children")).toArray();
  for (const QJsonValue& child : children) {
    if (child.isObject()) {
      CollectMosaicLeafNames(child.toObject(), names);
    }
  }
}

/** True for @c ViewportDock / @c ViewportDock_N object names. */
inline bool IsViewportDockObjectName(const QString& object_name) {
  return object_name == QLatin1String("ViewportDock") ||
         object_name.startsWith(QLatin1String("ViewportDock_"));
}

}  // namespace frame_session_detail
}  // namespace autoviz

