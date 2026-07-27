/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class PolygonDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PolygonStamped> {
 public:
  explicit PolygonDisplay(std::string channel);

  std::string typeId() const override { return "Polygon"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::PolygonStamped& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  std::vector<QVector3D> points_;
};

}  // namespace display
}  // namespace autoviz
