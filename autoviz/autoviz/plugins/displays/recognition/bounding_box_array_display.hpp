/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Inspired by RViz visualization plugins (BSD).
 * vision_msgs/BoundingBox3DArray.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/vision_msgs/bounding_box3d_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class BoundingBox3DArrayDisplay
 * @brief Draws @c vision_msgs/BoundingBox3DArray as wireframe / solid boxes.
 */
class BoundingBox3DArrayDisplay
    : public ChannelDisplay<automsgs::msgs::vision_msgs::BoundingBox3DArray> {
 public:
  explicit BoundingBox3DArrayDisplay(std::string channel);

  std::string typeId() const override { return "BoundingBox3DArray"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::vision_msgs::BoundingBox3DArray& message)
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
