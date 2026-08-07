/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QCursor>
#include <QIcon>
#include <QString>

#include "autoviz/common/display_status.hpp"

namespace autoviz {

class PanelDockWidget;

/** Load themed icons from Qt resources (RViz-style paths). */
class IconLoader {
 public:
  /** Window / taskbar icon (aviz.svg). */
  static QIcon applicationIcon();
  static QIcon load(const QString& resource_path);
  static QIcon displayIcon(const QString& display_type);
  static QIcon toolIcon(const QString& tool_id);
  /** RViz-style viewport cursor: base arrow + tool icon overlay. */
  static QCursor toolCursor(const QString& tool_id);
  static QCursor defaultCursor();
  /** Composite @p icon (16×16) onto cursor.svg like rviz_common::makeIconCursor. */
  static QCursor makeIconCursor(const QPixmap& icon, const QString& cache_key);
  static QIcon panelIcon(const QString& panel_id);
  /** Icon for a dock objectName / panelTypeId. */
  static QIcon dockPanelIcon(const QString& dock_type_id);
  /** Sync dock title bar icon from @p dock_type_id. */
  static void applyDockPanelChrome(PanelDockWidget* dock,
                                   const QString& dock_type_id);
  static QIcon menuIcon(const QString& menu_id);
  /** RViz-style panel title-bar icons (viewport tools, expand, more, …). */
  static QIcon panelTitleIcon(const QString& role);
  /** Expand (off) / collapse (on) toggle for panel maximize. */
  static QIcon panelExpandIcon();
  static QIcon statusIcon(display::DisplayStatusLevel level, bool enabled);
};

}  // namespace autoviz
