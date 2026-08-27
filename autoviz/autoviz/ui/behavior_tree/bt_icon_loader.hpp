/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QColor>
#include <QIcon>
#include <QPixmap>
#include <QString>

#include <optional>

namespace autoviz {
namespace behavior_tree {

/** Groot icon resources and NodesStyle.json lookups for the BT panel. */
class BtIconLoader {
 public:
  static QIcon icon(const QString& resource_path, int size = 16);
  static QPixmap pixmap(const QString& resource_path, int size = 16);
  /** Groot SVG/PNG icons recolored for autoviz light panels. */
  static QIcon toolbarIcon(const QString& relative_groot_path, int size = 18);
  static QIcon toolbarIconFromResource(const QString& resource_path, int size = 18);

  static QString nodeResource(const QString& registration_id, BtNodeKind kind);
  static QIcon nodeIcon(const QString& registration_id, BtNodeKind kind, int size = 16);
  static QPixmap nodePixmap(const QString& registration_id, BtNodeKind kind, int size = 16);
  static std::optional<QColor> nodeCaptionColor(const QString& registration_id,
                                                BtNodeKind kind);
  static QString nodeCaptionLabel(const QString& registration_id, BtNodeKind kind);

  static QIcon panelIcon(int size = 16);
  /** @p mode_name: editor | monitor | replay */
  static QIcon modeIcon(const QString& mode_name, int size = 16);

  static QString grootResource(const QString& relative_path);

 private:
  static void ensureStyleLoaded();
};

}  // namespace behavior_tree
}  // namespace autoviz
