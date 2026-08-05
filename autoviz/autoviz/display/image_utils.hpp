/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>

#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autoviz {
namespace display {

QImage imageFromProto(
    const automsgs::msgs::sensor_msgs::Image& image);

QImage compressedImageFromProto(
    const automsgs::msgs::sensor_msgs::CompressedImage& image);

/** Returns true if the message type can be rendered in the Image panel. */
bool isImageMessageType(const std::string& message_type);

}  // namespace display
}  // namespace autoviz
