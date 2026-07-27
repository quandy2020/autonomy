/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class ImageDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::Image> {
 public:
  explicit ImageDisplay(std::string channel);

  std::string typeId() const override { return "Image"; }

  const QImage& image() const { return image_; }

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::Image& message) override;
  void onDraw(rendering::SceneOverlay& /*scene*/) override {}

 private:
  QImage image_;
};

}  // namespace display
}  // namespace autoviz
