/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file catalog.hpp
 * @brief Foxglove-compatible panel catalog (Add Panel / Change panel order).
 *
 * Entries describe dock object names, icons, labels, and descriptions. An empty
 * @c object_name marks a catalog row that is not yet implemented in Autoviz.
 *
 * @see AddPanelDialog
 * @see ChangePanelMenuWidget
 * @see FramePanels
 * @see PanelCatalogEntry::isImplemented()
 */

#pragma once

#include <QVector>

namespace autoviz {

/**
 * @struct PanelCatalogEntry
 * @brief One row in the panel catalog shown by Add / Change UI.
 *
 * String fields are compile-time @c const char* literals (not owned heap
 * strings) so the catalog can be a static table.
 */
struct PanelCatalogEntry {
  /**
   * Dock @c objectName; empty when not implemented in Autoviz yet.
   */
  const char* object_name;

  /** Icon id passed to @ref IconLoader::panelIcon() / menu helpers. */
  const char* icon_id;

  /** English display label (translated at UI sites when needed). */
  const char* label;

  /** Short description for tooltips / subtitle rows. */
  const char* description;

  /**
   * @brief @c true when @c object_name is non-null and non-empty.
   * @return Whether Autoviz can create this panel type.
   */
  bool isImplemented() const {
    return object_name != nullptr && object_name[0] != '\0';
  }
};

/**
 * @brief Foxglove-compatible panel list (Add Panel menu order).
 * @return Catalog entries in display order.
 * @see PanelCatalogEntry
 */
QVector<PanelCatalogEntry> PanelCatalog();

}  // namespace autoviz
