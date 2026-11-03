/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/footstep_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class FootstepDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::FootstepArray> {
 public:
  explicit FootstepDisplay(std::string channel);

  std::string typeId() const override { return "Footstep"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::FootstepArray& message)
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

  std::vector<BoxVisual> footsteps_;
};

}  // namespace display
}  // namespace autoviz
