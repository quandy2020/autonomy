/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file axes_display.hpp
 * @brief Static RGB coordinate axes at the origin (or configured pose) —
 *        RViz Axes display analogue.
 *
 * Not channel-backed; draws three orthogonal arrows each update based on
 * properties such as @c length.
 *
 * @see Display
 * @see GridDisplay
 */

#pragma once

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"

namespace autoviz {
namespace display {

/**
 * @class AxesDisplay
 * @brief Draws fixed-frame RGB axes for spatial reference.
 *
 * @ref onDraw reads properties (e.g. length) and appends axis geometry to
 * the scene overlay. No subscription lifecycle.
 *
 * @see Display
 * @see GridDisplay
 */
class AxesDisplay : public Display {
 public:
  /**
   * @brief Constructs an Axes display with default properties.
   */
  AxesDisplay();

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Axes".
   */
  std::string typeId() const override { return "Axes"; }

  /**
   * @brief Property schema (axis length, etc.).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Emits RGB axis arrows into @p scene.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;
};

}  // namespace display
}  // namespace autoviz
