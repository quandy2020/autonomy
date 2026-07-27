/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <array>

#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class PoseWithCovarianceDisplay
    : public ChannelDisplay<
          automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped> {
 public:
  explicit PoseWithCovarianceDisplay(std::string channel);

  std::string typeId() const override { return "PoseWithCovariance"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_pose_ = false;
  QVector3D position_;
  QQuaternion pose_orientation_;
  QQuaternion frame_orientation_;
  float yaw_ = 0.f;
  std::array<double, 36> covariance_{};
};

}  // namespace display
}  // namespace autoviz
