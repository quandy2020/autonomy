/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class CameraInfoDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::CameraInfo> {
 public:
  explicit CameraInfoDisplay(std::string channel);

  std::string typeId() const override { return "CameraInfo"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::CameraInfo& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_info_ = false;
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;
};

}  // namespace display
}  // namespace autoviz
