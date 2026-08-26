/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class PoseArrayDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PoseArray> {
 public:
  explicit PoseArrayDisplay(std::string channel);

  std::string typeId() const override { return "PoseArray"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseArray& message)
      override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredPose {
    QVector3D position;
    float yaw = 0.f;
  };

  std::vector<StoredPose> poses_;
};

}  // namespace display
}  // namespace autoviz
