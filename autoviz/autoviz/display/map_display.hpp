/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class MapDisplay
    : public ChannelDisplay<automsgs::msgs::map_msgs::OccupancyGrid> {
 public:
  explicit MapDisplay(std::string channel);

  std::string typeId() const override { return "Map"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::map_msgs::OccupancyGrid& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct MapCell {
    QVector3D position;
    QColor color;
  };
  std::vector<MapCell> cells_;
};

}  // namespace display
}  // namespace autoviz
