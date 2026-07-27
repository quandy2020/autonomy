/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>
#include <vector>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class OdometryDisplay
    : public ChannelDisplay<automsgs::msgs::nav_msgs::Odometry> {
 public:
  explicit OdometryDisplay(std::string channel);

  std::string typeId() const override { return "Odometry"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::nav_msgs::Odometry& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct OdometryVisual {
    QVector3D position;
    QVector3D velocity;
    float yaw = 0.f;
  };
  OdometryVisual odom_;
  bool has_odom_ = false;
};

}  // namespace display
}  // namespace autoviz
