/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file grid_display.hpp
 * @brief Static reference ground grid (RViz Grid display analogue).
 *
 * Not channel-backed. Draws a planar cell grid in the fixed frame using
 * @ref rendering::GridRenderer settings derived from display properties
 * (cell count, size, color, offset, …).
 *
 * @see Display
 * @see AxesDisplay
 * @see rendering::GridRenderer
 */

#pragma once

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"

namespace autoviz {
namespace display {

/**
 * @class GridDisplay
 * @brief Draws a configurable XY (or plane) reference grid.
 *
 * @ref onDraw pushes grid geometry into the scene overlay / render path.
 * No subscription lifecycle.
 *
 * @see Display
 * @see AxesDisplay
 */
class GridDisplay : public Display {
 public:
  /**
   * @brief Constructs a Grid display with default cell properties.
   */
  GridDisplay();

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Grid".
   */
  std::string typeId() const override { return "Grid"; }

  /**
   * @brief Property schema (plane, cell size/count, color, line style, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Emits grid lines / cells into @p scene.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;
};

}  // namespace display
}  // namespace autoviz
