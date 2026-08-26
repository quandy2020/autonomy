/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class PoseDisplay
    : public ChannelDisplay<
          automsgs::msgs::geometry_msgs::PoseStamped> {
 public:
  explicit PoseDisplay(std::string channel);

  std::string typeId() const override { return "Pose"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseStamped& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void clearReceivedData() override;

 private:
  bool have_pose_ = false;
  QVector3D position_;
  float yaw_ = 0.f;
};

}  // namespace display
}  // namespace autoviz
