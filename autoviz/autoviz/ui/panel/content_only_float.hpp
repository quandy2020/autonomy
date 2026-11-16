/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file content_only_float.hpp
 * @brief Chrome-less floating dock helpers (drag / resize / hover restore bar).
 */

#pragma once

#include <functional>

#include <QString>

class QWidget;

namespace autoviz {

class PanelDockWidget;

/**
 * @brief Installs or removes content-only float interaction on @p dock.
 *
 * When enabled: hides the title bar, removes visible borders/grip, adds a top
 * hover restore bar, top-edge window drag, and edge/corner resize hit zones.
 * Does not engage Qt dock snap.
 *
 * @param dock Floating main-panel dock (must already be intentionalFloat).
 * @param enabled @c true to enter content-only; @c false to restore chrome.
 * @param on_show_chrome Callback for "Show chrome" (exit content-only).
 * @param on_dock Callback for "Dock" (exit content-only then redock).
 * @param title Optional title shown on the hover bar.
 */
void SetContentOnlyFloatChrome(
    PanelDockWidget* dock, bool enabled,
    std::function<void()> on_show_chrome = {},
    std::function<void()> on_dock = {},
    const QString& title = {});

/** @return @c true when @p dock is in content-only float mode. */
bool IsContentOnlyFloat(const PanelDockWidget* dock);

}  // namespace autoviz
