/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file icon_loader.hpp
 * @brief Load themed icons and cursors from Qt resources (RViz-style paths).
 *
 * Centralizes brand, toolbar, panel, menu, and status glyphs so chrome and
 * docks share the same slate / teal ink language.
 *
 * @see FrameChrome
 * @see PanelDockWidget
 * @see PanelsMenuIconInk()
 * @see display::DisplayStatusLevel
 */

#pragma once

#include <QCursor>
#include <QIcon>
#include <QString>

#include "autoviz/common/display_status.hpp"

namespace autoviz {

class PanelDockWidget;

/**
 * @class IconLoader
 * @brief Static helpers for Autoviz themed icons and tool cursors.
 *
 * All methods are free of instance state; icons are loaded from the Qt resource
 * system (paths aligned with RViz conventions where applicable).
 *
 * @note Prefer @ref dockPanelIcon() / @ref applyDockPanelChrome() over ad-hoc
 *       resource paths when wiring docks.
 */
class IconLoader {
 public:
  /**
   * @brief Window / taskbar icon (squirrel brand mark).
   * @return Application @c QIcon.
   */
  static QIcon applicationIcon();

  /**
   * @brief Loads an icon from a Qt resource path.
   * @param resource_path Resource path (e.g. @c ":/icons/...").
   * @return Loaded icon (may be null if missing).
   */
  static QIcon load(const QString& resource_path);

  /**
   * @brief Icon for a display type in the Displays tree.
   * @param display_type Display class / type id.
   * @return Type icon.
   */
  static QIcon displayIcon(const QString& display_type);

  /**
   * @brief Toolbar tool glyph — slate off, teal when active.
   * @param tool_id Tool registry id (Interact, Select, …).
   * @return Tool icon.
   */
  static QIcon toolIcon(const QString& tool_id);

  /**
   * @brief Monochrome toolbar chrome glyph (plus / dock / etc.).
   * @param resource_base Base resource name without state suffix.
   * @return Chrome glyph icon.
   */
  static QIcon toolbarGlyph(const QString& resource_base);

  /**
   * @brief RViz-style viewport cursor: base arrow + tool icon overlay.
   * @param tool_id Tool registry id.
   * @return Composed cursor.
   * @see makeIconCursor()
   */
  static QCursor toolCursor(const QString& tool_id);

  /**
   * @brief Default viewport arrow cursor (no tool overlay).
   * @return Default cursor.
   */
  static QCursor defaultCursor();

  /**
   * @brief Composite @p icon (16×16) onto @c cursor.svg like
   *        @c rviz_common::makeIconCursor.
   *
   * @param icon Overlay pixmap.
   * @param cache_key Cache key for the composed cursor.
   * @return Composed cursor.
   */
  static QCursor makeIconCursor(const QPixmap& icon, const QString& cache_key);

  /**
   * @brief Icon for a panel catalog id.
   * @param panel_id Panel type / catalog id.
   * @return Panel icon.
   */
  static QIcon panelIcon(const QString& panel_id);

  /**
   * @brief Icon for a dock @c objectName / panelTypeId.
   * @param dock_type_id Dock type id.
   * @return Dock panel icon.
   */
  static QIcon dockPanelIcon(const QString& dock_type_id);

  /**
   * @brief Sync dock title bar icon from @p dock_type_id.
   *
   * @param dock Target dock (non-null).
   * @param dock_type_id Type id used for @ref dockPanelIcon().
   */
  static void applyDockPanelChrome(PanelDockWidget* dock,
                                   const QString& dock_type_id);

  /**
   * @brief Icon for a top-level menu id (File, Help, …).
   * @param menu_id Menu identifier.
   * @return Menu icon.
   */
  static QIcon menuIcon(const QString& menu_id);

  /**
   * @brief Panels menu dock toggle icon — same 16px normalized slate as
   *        @ref menuIcon().
   * @param dock_type_id Dock type id.
   * @return Menu-sized dock icon.
   */
  static QIcon panelsMenuDockIcon(const QString& dock_type_id);

  /**
   * @brief Flatten @p source into a 16×16 linear monochrome icon for Panels
   *        menus.
   *
   * Preserves alpha; fills visible pixels with @p ink.
   *
   * @param source Source icon.
   * @param ink Fill color (default slate).
   * @return Monochrome icon.
   * @see PanelsMenuIconInk()
   */
  static QIcon monochromeMenuIcon(const QIcon& source,
                                  const QColor& ink = QColor(0x2D, 0x37, 0x48));

  /**
   * @brief RViz-style panel title-bar icons (viewport tools, expand, more, …).
   * @param role Title-tool role id.
   * @return Title-bar icon.
   */
  static QIcon panelTitleIcon(const QString& role);

  /**
   * @brief Expand (off) / collapse (on) toggle for panel maximize.
   * @return Dual-state expand icon.
   */
  static QIcon panelExpandIcon();

  /**
   * @brief Status icon for a display tree row.
   *
   * @param level Status severity.
   * @param enabled Whether the display is enabled (may dim the icon).
   * @return Status icon.
   */
  static QIcon statusIcon(display::DisplayStatusLevel level, bool enabled);
};

}  // namespace autoviz
