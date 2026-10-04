/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file role.hpp
 * @brief Panel placement roles in the Foxglove-style shell (main / sidebar /
 *        bottom).
 *
 * Role strings are stored on docks (dynamic property or session config) and
 * consulted by @ref FrameLayout when attaching panels to
 * @ref MainPanelHost vs left/right dock areas vs the bottom Time strip.
 *
 * @see FrameLayout::isMainPanel()
 * @see FrameLayout::addMainPanelDock()
 * @see FrameLayout::addSidebarDock()
 */

#pragma once

#include <QString>

namespace autoviz {

/**
 * @brief Role string for main-column panels (3D View, Image, Plot, …).
 * @return @c "main"
 */
inline QString PanelRoleMain() { return QStringLiteral("main"); }

/**
 * @brief Role string for sidebar panels (Displays, Views, Properties, …).
 * @return @c "sidebar"
 */
inline QString PanelRoleSidebar() { return QStringLiteral("sidebar"); }

/**
 * @brief Role string for bottom strip panels (Time).
 * @return @c "bottom"
 */
inline QString PanelRoleBottom() { return QStringLiteral("bottom"); }

/**
 * @brief @c true if @p role is the main-column role.
 * @param role Role string to test.
 */
inline bool IsMainPanelRole(const QString& role) {
  return role == PanelRoleMain();
}

/**
 * @brief @c true if @p role is the sidebar role.
 * @param role Role string to test.
 */
inline bool IsSidebarPanelRole(const QString& role) {
  return role == PanelRoleSidebar();
}

}  // namespace autoviz
