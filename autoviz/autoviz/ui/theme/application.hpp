/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file application.hpp
 * @brief Application-wide Fusion theme: palette, QSS, and menu chrome helpers.
 *
 * Entry point is @ref ApplyAppTheme(). Layering:
 *  1. @c QPalette — semantic colors (@ref glass::Shell)
 *  2. QSS — spacing, borders, panel @c #objectName rules
 *  3. Overlays — @ref glass::Overlay for 3D HUD / floating tools
 *
 * @see glass::ShellTokens
 * @see ProxyStyle
 * @see AppThemeIds
 * @see style::sheet()
 */

#pragma once

#include <QColor>
#include <QString>

class QApplication;
class QMenu;
class QPalette;

namespace autoviz {

/**
 * @brief Qt Fusion highlight accent — frosted-glass Shell cyan.
 * @return Accent @c QColor (@ref glass::ShellTokens::accent).
 */
QColor AppThemeAccentColor();

/**
 * @brief Default 3D viewport background (48,48,48) — contrast for glass overlays.
 * @return Suggested clear color for render windows.
 */
QColor AppThemeSuggestedViewportBackground();

/**
 * @brief Builds the cool-light @c QPalette used by @ref ApplyAppTheme().
 * @return Palette filled from @ref glass::Shell().
 */
QPalette BuildThemePalette();

/**
 * @brief Apply frosted-glass theme: Fusion base + cool light palette + cyan accents.
 *
 * Sets Fusion style with @ref ProxyStyle, applies @ref BuildThemePalette(), and
 * installs @ref BuildAppThemeStylesheet().
 *
 * @param app Application instance (typically @c qApp).
 * @see BuildAppThemeStylesheet()
 * @see PrepareAppMenu()
 */
void ApplyAppTheme(QApplication& app);

/**
 * @brief Assembles the global application stylesheet (chrome + widgets + panels).
 * @return QSS string suitable for @c QApplication::setStyleSheet().
 * @see style::sheet()
 */
QString BuildAppThemeStylesheet();

/**
 * @brief Checkbox-style indicators for Panels menu (applied per @c QMenu via
 *        @c setStyleSheet).
 * @return QSS fragment for @ref PreparePanelsMenu().
 */
QString BuildPanelsMenuStylesheet();

/**
 * @brief Ink color for Panels menu monochrome icons (light/dark).
 * @return Fill color passed to @ref IconLoader::monochromeMenuIcon().
 */
QColor PanelsMenuIconInk();

/**
 * @brief Frosted popup chrome for app menus (File / Help / …).
 *
 * Tags the menu for @ref ProxyStyle teal hover pills + milky panel.
 *
 * @param menu Menu to prepare (non-null).
 * @see PreparePanelsMenu()
 * @see IsActiveGlassMenuPopup()
 */
void PrepareAppMenu(QMenu* menu);

/**
 * @brief @ref PrepareAppMenu() + checkbox/icon column layout (Panels catalog).
 * @param menu Panels menu to prepare (non-null).
 * @see BuildPanelsMenuStylesheet()
 * @see IsActivePanelsMenuPopup()
 */
void PreparePanelsMenu(QMenu* menu);

/**
 * @brief @c true while a @ref PreparePanelsMenu()-tagged @c QMenu popup is visible.
 *
 * Used by @ref ProxyStyle to special-case checkbox / icon painting.
 */
bool IsActivePanelsMenuPopup();

/**
 * @brief @c true while a @ref PrepareAppMenu() / Panels-tagged glass menu popup
 *        is visible.
 */
bool IsActiveGlassMenuPopup();

}  // namespace autoviz
