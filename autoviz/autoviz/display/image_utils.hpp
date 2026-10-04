/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_utils.hpp
 * @brief Converts @c sensor_msgs/Image and @c CompressedImage protobufs to
 *        @c QImage for Image / Camera panels and displays.
 *
 * Also provides a message-type predicate used by the Image panel to decide
 * whether a channel can be previewed.
 *
 * @see ImageDisplay
 * @see CameraDisplay
 * @see imageFromProto
 */

#pragma once

#include <QImage>

#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autoviz {
namespace display {

/**
 * @brief Decodes a raw @c sensor_msgs/Image protobuf into a @c QImage.
 *
 * Supports common encodings (RGB8, BGR8, mono, etc.) as implemented in the
 * corresponding .cpp.
 *
 * @param image Source protobuf image.
 * @return Converted @c QImage, or a null image on unsupported encoding /
 *         empty data.
 *
 * @see compressedImageFromProto
 * @see ImageDisplay
 */
QImage imageFromProto(
    const automsgs::msgs::sensor_msgs::Image& image);

/**
 * @brief Decodes a @c sensor_msgs/CompressedImage (JPEG/PNG/…) into a
 *        @c QImage.
 *
 * @param image Compressed image protobuf with @c format and @c data.
 * @return Decoded @c QImage, or null on failure.
 *
 * @see imageFromProto
 */
QImage compressedImageFromProto(
    const automsgs::msgs::sensor_msgs::CompressedImage& image);

/**
 * @brief Returns whether @p message_type can be rendered in the Image panel.
 *
 * @param message_type Wire / schema type string from channel discovery.
 * @return @c true for raw or compressed image message types known to Autoviz.
 *
 * @see imageFromProto
 * @see compressedImageFromProto
 */
bool isImageMessageType(const std::string& message_type);

}  // namespace display
}  // namespace autoviz
