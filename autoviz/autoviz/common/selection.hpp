/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file selection.hpp
 * @brief Data types describing a single selected object in the 3D view.
 *
 * @see SelectionManager
 * @see SelectionHandler
 * @see PickHandle
 */

#pragma once

#include <string>
#include <utility>
#include <vector>

#include <QVector3D>

#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {

/**
 * @struct SelectionEntry
 * @brief One item in the global selection set (position + display identity).
 *
 * Produced by tools / handlers after a pick and shown in the Selection panel.
 */
struct SelectionEntry {
  /** World position of the selection in the fixed frame. */
  QVector3D position;

  /** Owning display instance name. */
  std::string display_name;

  /** Display type id. */
  std::string display_type;

  /** Pick handle used for highlight / deselect callbacks (−1 invalid). */
  PickHandle pick_handle = kInvalidPickHandle;

  /** Optional point index within a cloud (−1 if N/A). */
  int point_index = -1;

  /**
   * Key/value property pairs for the Selection panel
   * (e.g. @c "Index" → @c "42").
   */
  std::vector<std::pair<std::string, std::string>> properties;
};

}  // namespace common
}  // namespace autoviz
