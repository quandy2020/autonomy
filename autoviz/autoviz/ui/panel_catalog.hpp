/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector>

namespace autoviz {

struct PanelCatalogEntry {
  /** Dock objectName; empty when not implemented in autoviz yet. */
  const char* object_name;
  const char* icon_id;
  const char* label;
  const char* description;

  bool isImplemented() const {
    return object_name != nullptr && object_name[0] != '\0';
  }
};

/** Foxglove-compatible panel list (Add Panel menu order). */
QVector<PanelCatalogEntry> PanelCatalog();

}  // namespace autoviz
