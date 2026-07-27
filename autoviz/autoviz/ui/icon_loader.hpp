/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QCursor>
#include <QIcon>
#include <QString>

#include "autoviz/common/display_status.hpp"

namespace autoviz {

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
  static QIcon statusIcon(display::DisplayStatusLevel level, bool enabled);
};

}  // namespace autoviz
