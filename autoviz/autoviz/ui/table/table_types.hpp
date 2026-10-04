/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file table_types.hpp
 * @brief Runtime config for the Table panel (array-of-messages browser).
 */

#pragma once

#include <QString>

namespace autoviz {
namespace table_panel {

/**
 * @struct TablePanelConfig
 * @brief Channel + repeated-field path shown as rows/columns.
 */
struct TablePanelConfig {
  /** Panel title. */
  QString title = QStringLiteral("Table");

  /** Source Autolink channel. */
  QString channel;

  /**
   * Path to a repeated protobuf field (e.g. @c detections or @c pose.covariance).
   * Empty → wait for a path drop / edit.
   */
  QString field_path;

  /** Case-insensitive substring filter applied to visible row cells. */
  QString row_filter;
};

/**
 * @brief Default empty table config.
 */
inline TablePanelConfig DefaultTablePanelConfig() {
  return TablePanelConfig{};
}

}  // namespace table_panel
}  // namespace autoviz
