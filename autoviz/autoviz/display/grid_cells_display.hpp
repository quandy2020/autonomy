/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file grid_cells_display.hpp
 * @brief Displays @c map_msgs/GridCells as filled rectangles in the map
 *        frame (RViz GridCells).
 *
 * Each cell center/size from the message is cached and drawn as a colored
 * quad / box in the fixed frame.
 *
 * @see ChannelDisplay
 * @see MapDisplay
 * @see GridMapDisplay
 */

#pragma once

#include <vector>

#include <QVector3D>

#include <automsgs/msgs/map_msgs/grid_cells.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class GridCellsDisplay
 * @brief Channel display for sparse grid cell overlays (costmap footprints,
 *        etc.).
 *
 * @see MapDisplay
 * @see ChannelDisplay
 */
class GridCellsDisplay
    : public ChannelDisplay<automsgs::msgs::map_msgs::GridCells> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for GridCells messages.
   */
  explicit GridCellsDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "GridCells".
   */
  std::string typeId() const override { return "GridCells"; }

  /**
   * @brief Property schema (color, alpha, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Rebuilds @c cells_ from the latest GridCells message.
   *
   * @param message map_msgs/GridCells protobuf.
   */
  void processMessage(const automsgs::msgs::map_msgs::GridCells& message)
      override;

  /**
   * @brief Draws cached cells into @p scene.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears @c cells_.
   */
  void clearReceivedData() override;

 private:
  /**
   * @struct Cell
   * @brief One grid cell rectangle in the message / fixed frame.
   */
  struct Cell {
    QVector3D center;   /**< Cell center position. */
    float width = 0.f;  /**< Cell extent along X (meters). */
    float height = 0.f; /**< Cell extent along Y (meters). */
  };

  /** Cached cells from the latest message. */
  std::vector<Cell> cells_;
};

}  // namespace display
}  // namespace autoviz
