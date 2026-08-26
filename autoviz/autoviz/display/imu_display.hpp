/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class ImuDisplay : public ChannelDisplay<automsgs::msgs::sensor_msgs::Imu> {
 public:
  explicit ImuDisplay(std::string channel);

  std::string typeId() const override { return "Imu"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(const automsgs::msgs::sensor_msgs::Imu& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_imu_ = false;
  QVector3D origin_;
  QVector3D linear_accel_;
  QVector3D angular_vel_;
  float roll_ = 0.f;
  float pitch_ = 0.f;
  float yaw_ = 0.f;
};

}  // namespace display
}  // namespace autoviz
