/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector>

namespace autoviz {

struct PanelCatalogEntry {
  const char* object_name;
  const char* icon_id;
  const char* label;
  const char* description;
};

QVector<PanelCatalogEntry> PanelCatalog();

}  // namespace autoviz
