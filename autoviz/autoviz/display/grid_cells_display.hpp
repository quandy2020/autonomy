/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QVector3D>

#include <automsgs/msgs/map_msgs/grid_cells.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class GridCellsDisplay
    : public ChannelDisplay<automsgs::msgs::map_msgs::GridCells> {
 public:
  explicit GridCellsDisplay(std::string channel);

  std::string typeId() const override { return "GridCells"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(const automsgs::msgs::map_msgs::GridCells& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct Cell {
    QVector3D center;
    float width = 0.f;
    float height = 0.f;
  };
  std::vector<Cell> cells_;
};

}  // namespace display
}  // namespace autoviz
