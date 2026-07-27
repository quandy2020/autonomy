/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/point_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class PointStampedDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PointStamped> {
 public:
  explicit PointStampedDisplay(std::string channel);

  std::string typeId() const override { return "PointStamped"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::PointStamped& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_point_ = false;
  QVector3D position_;
};

}  // namespace display
}  // namespace autoviz
