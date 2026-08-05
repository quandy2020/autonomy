/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

namespace autoviz {

/** Where a panel lives in the Foxglove-style shell. */
inline QString PanelRoleMain() { return QStringLiteral("main"); }
inline QString PanelRoleSidebar() { return QStringLiteral("sidebar"); }
inline QString PanelRoleBottom() { return QStringLiteral("bottom"); }

inline bool IsMainPanelRole(const QString& role) {
  return role == PanelRoleMain();
}

inline bool IsSidebarPanelRole(const QString& role) {
  return role == PanelRoleSidebar();
}

}  // namespace autoviz
