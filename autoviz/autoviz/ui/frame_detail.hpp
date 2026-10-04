/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_detail.hpp
 * @brief Internal helpers shared by VisualizationFrame collaborators.
 *
 * Free functions in @c autoviz::detail keep small shared UI strings out of the
 * public collaborator headers.
 *
 * @see FrameChrome
 * @see PanelDockWidget
 */

#pragma once

#include <QString>

namespace autoviz::detail {

/**
 * @brief Shared panel menu title used by chrome toggles and panel docks.
 *
 * Resolves a localized / catalog display title for @p type_id when known,
 * otherwise returns @p fallback (typically the dock window title).
 *
 * @param type_id Panel type / dock object-name id (e.g. @c "DisplaysDock").
 * @param fallback Title to use when the catalog has no match.
 * @return Display string suitable for Panels menu toggles and dock chrome.
 *
 * @see PanelCatalog()
 * @see FrameChrome::registerPanelMenuToggle()
 */
QString PanelsMenuDisplayTitle(const QString& type_id, const QString& fallback);

}  // namespace autoviz::detail
