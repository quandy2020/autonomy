/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Native visualization_msgs/BoundingBoxArray (label/value fields).
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/bounding_box_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class RecognitionBoundingBoxArrayDisplay
    : public ChannelDisplay<
          automsgs::msgs::visualization_msgs::BoundingBoxArray> {
 public:
  explicit RecognitionBoundingBoxArrayDisplay(std::string channel);

  std::string typeId() const override {
    return "RecognitionBoundingBoxArray";
  }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::BoundingBoxArray& message)
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
