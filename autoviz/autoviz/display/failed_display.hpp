/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file failed_display.hpp
 * @brief Placeholder display when a catalog plugin fails to instantiate
 *        (@c rviz_common::FailedDisplay analogue).
 *
 * Used by the display factory so the Displays tree still shows the intended
 * type id and an error status with the failure reason, instead of dropping
 * the session entry silently.
 *
 * @see Display
 * @see DisplayGroup
 */

#pragma once

#include <string>

#include "autoviz/display/display.hpp"

namespace autoviz {
namespace display {

/**
 * @class FailedDisplay
 * @brief Non-drawing stand-in for a display type that could not be loaded.
 *
 * @ref onEnable sets error status from the stored reason. @ref onDraw is a
 * no-op so the 3D view remains usable.
 *
 * @see Display
 */
class FailedDisplay : public Display {
 public:
  /**
   * @brief Constructs a failed-plugin placeholder.
   *
   * @param type Intended catalog type id (returned by @ref typeId()).
   * @param reason Human-readable failure explanation for status text.
   */
  FailedDisplay(std::string type, std::string reason);

  /**
   * @brief Returns the intended plugin type id.
   *
   * @return @c type_ stored at construction.
   */
  std::string typeId() const override { return type_; }

 protected:
  /**
   * @brief Sets error status to @c reason_.
   */
  void onEnable() override;

  /**
   * @brief No-op draw (placeholder has no geometry).
   *
   * @param scene Unused.
   */
  void onDraw(rendering::SceneOverlay& /*scene*/) override {}

 private:
  /** Intended catalog type id. */
  std::string type_;

  /** Failure explanation shown as error status. */
  std::string reason_;
};

}  // namespace display
}  // namespace autoviz
