/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/image_display.hpp"

#include "autoviz/display/image_utils.hpp"

namespace autoviz {
namespace display {

ImageDisplay::ImageDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::Image>(
          "Image", std::move(channel),
          "automsgs.msgs.sensor_msgs.Image") {}

void ImageDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::Image& message) {
  image_ = imageFromProto(message);
  if (context_ != nullptr && context_->image_updated && !image_.isNull()) {
    context_->image_updated(name(), image_);
  }
}

void ImageDisplay::clearReceivedData() {
  image_ = QImage();
  if (context_ != nullptr && context_->image_updated) {
    context_->image_updated(name(), image_);
  }
}

}  // namespace display
}  // namespace autoviz
