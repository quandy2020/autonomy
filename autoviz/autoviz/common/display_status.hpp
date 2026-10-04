/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_status.hpp
 * @brief Status level and message reported by a Display to the UI tree.
 *
 * Used by the Displays panel to show Ok / Warn / Error icons and tooltips
 * next to each display row (RViz StatusProperty equivalent).
 *
 * @see display::Display
 */

#pragma once

#include <string>

namespace autoviz {
namespace display {

/**
 * @enum DisplayStatusLevel
 * @brief Severity of a display's current status.
 */
enum class DisplayStatusLevel {
  kOk,    /**< Healthy / no issues. */
  kWarn,  /**< Degraded but still rendering (e.g. missing TF). */
  kError, /**< Failed (e.g. bad channel, transform unavailable). */
};

/**
 * @struct DisplayStatus
 * @brief Combined severity + human-readable message for one display.
 */
struct DisplayStatus {
  /** Current severity (defaults to @ref DisplayStatusLevel::kOk). */
  DisplayStatusLevel level = DisplayStatusLevel::kOk;

  /** Short message shown in the Displays tree / tooltip. */
  std::string message;
};

}  // namespace display
}  // namespace autoviz
