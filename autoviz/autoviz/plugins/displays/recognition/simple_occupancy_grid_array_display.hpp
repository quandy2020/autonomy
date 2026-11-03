/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/simple_occupancy_grid_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class SimpleOccupancyGridArrayDisplay
    : public ChannelDisplay<
          automsgs::msgs::visualization_msgs::SimpleOccupancyGridArray> {
 public:
  explicit SimpleOccupancyGridArrayDisplay(std::string channel);

  std::string typeId() const override { return "SimpleOccupancyGridArray"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::SimpleOccupancyGridArray&
          message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct CellVisual {
    QVector3D center;
    QVector3D half_size;
    QColor color;
  };

  std::vector<CellVisual> cells_;
};

}  // namespace display
}  // namespace autoviz
