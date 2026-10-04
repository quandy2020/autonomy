/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file camera_display.hpp
 * @brief RViz Camera display: image topic + CameraInfo frustum / image plane
 *        in 3D.
 *
 * Subscribes to an image channel and a companion CameraInfo channel, converts
 * frames via @ref imageFromProto, and draws frustum edges plus a textured
 * far plane using @ref camera_utils.hpp.
 *
 * @see CameraInfoDisplay
 * @see camera_utils.hpp
 * @see image_utils.hpp
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QImage>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader.hpp"
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {

/**
 * @class CameraDisplay
 * @brief Visualizes a camera image projected on its frustum far plane.
 *
 * Uses dedicated Autolink readers (not @ref ChannelDisplay) because two
 * channels (image + CameraInfo) must stay in sync. Properties include
 * @c camera_info_channel, near/far distances, and frustum color.
 *
 * @note @ref image() exposes the latest decoded frame for optional 2D UI
 *       consumers.
 *
 * @see CameraInfoDisplay
 * @see DepthCloudDisplay
 * @see buildCameraFarPlane
 */
class CameraDisplay : public Display {
 public:
  /**
   * @brief Constructs the display for @p image_channel.
   *
   * @param image_channel Primary @c sensor_msgs/Image Autolink channel.
   */
  explicit CameraDisplay(std::string image_channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Camera".
   */
  std::string typeId() const override { return "Camera"; }

  /**
   * @brief Primary image channel name.
   *
   * @return @c image_channel_.
   */
  std::string channel() const override { return image_channel_; }

  /**
   * @brief Changes the image channel and rebinds readers when enabled.
   *
   * @param channel New image channel name.
   */
  void setChannel(const std::string& channel) override;

  /**
   * @brief Property schema (CameraInfo channel, near/far, color, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  /**
   * @brief Latest decoded camera image (may be null if none received).
   *
   * @return Const reference to @c image_.
   */
  const QImage& image() const { return image_; }

 protected:
  /**
   * @brief Creates image and CameraInfo readers / queues.
   */
  void onEnable() override;

  /**
   * @brief Tears down readers and clears queues.
   */
  void onDisable() override;

  /**
   * @brief Clears cached image and CameraInfo state.
   */
  void reset() override;

  /**
   * @brief Drains image / CameraInfo queues and updates status.
   */
  void onUpdate() override;

  /**
   * @brief Draws frustum and textured far plane when CameraInfo is available.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @brief Decodes @p message into @c image_.
   *
   * @param message Raw sensor_msgs/Image protobuf.
   */
  void processImage(
      const automsgs::msgs::sensor_msgs::Image& message);

  /**
   * @brief Stores the latest CameraInfo for frustum projection.
   *
   * @param message sensor_msgs/CameraInfo protobuf.
   */
  void processCameraInfo(
      const automsgs::msgs::sensor_msgs::CameraInfo& message);

  /** Primary image topic. */
  std::string image_channel_;

  /** Companion CameraInfo topic (from property or convention). */
  std::string camera_info_channel_;

  /** Incoming image payload queue. */
  integration::MessageQueue image_queue_;

  /** Incoming CameraInfo payload queue. */
  integration::MessageQueue camera_info_queue_;

  /** Autolink reader for the image channel. */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> image_reader_;

  /** Autolink reader for the CameraInfo channel. */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      camera_info_reader_;

  /** Latest decoded image for texture / UI. */
  QImage image_;

  /** Latest CameraInfo used for frustum geometry. */
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;

  /** @c true after at least one CameraInfo was processed. */
  bool have_camera_info_ = false;
};

}  // namespace display
}  // namespace autoviz
