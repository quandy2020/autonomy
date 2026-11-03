/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Inspired by RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/bounding_box.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class BoundingBoxDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::BoundingBox> {
 public:
  explicit BoundingBoxDisplay(std::string channel);

  std::string typeId() const override { return "BoundingBox"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::BoundingBox& message)
      override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct BoxVisual {
    QVector3D center;
    QVector3D size;
    QQuaternion orientation;
    QColor color;
  };

  std::vector<BoxVisual> boxes_;
};

}  // namespace display
}  // namespace autoviz
