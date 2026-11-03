/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class TargetVisualizerDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PoseStamped> {
 public:
  explicit TargetVisualizerDisplay(std::string channel);

  std::string typeId() const override { return "TargetVisualizer"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseStamped& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_pose_ = false;
  QVector3D position_;
  QQuaternion orientation_;
};

}  // namespace display
}  // namespace autoviz
