/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file depth_cloud_display.hpp
 * @brief Builds a colored point cloud from a depth image + CameraInfo
 *        (+ optional color image).
 *
 * RViz DepthCloud analogue. Subscribes to depth, CameraInfo, and optional
 * color channels; projects via @ref projectDepthImage; draws points with
 * configurable size/style.
 *
 * @see depth_cloud_utils.hpp
 * @see CameraDisplay
 * @see projectDepthImage
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QColor>
#include <QImage>
#include <QVector3D>

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
 * @class DepthCloudDisplay
 * @brief Multi-channel depth → point cloud visualization.
 *
 * Not based on @ref ChannelDisplay because three channels (depth,
 * CameraInfo, color) are coordinated. @ref rebuildPoints regenerates
 * @c points_ when inputs or properties change.
 *
 * @see projectDepthImage
 * @see DepthCloudPoint
 */
class DepthCloudDisplay : public Display {
 public:
  /**
   * @brief Constructs the display for @p depth_channel.
   *
   * @param depth_channel Primary depth @c sensor_msgs/Image channel.
   */
  explicit DepthCloudDisplay(std::string depth_channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "DepthCloud".
   */
  std::string typeId() const override { return "DepthCloud"; }

  /**
   * @brief Primary depth channel name.
   *
   * @return @c depth_channel_.
   */
  std::string channel() const override { return depth_channel_; }

  /**
   * @brief Changes the depth channel and rebinds readers when enabled.
   *
   * @param channel New depth channel name.
   */
  void setChannel(const std::string& channel) override;

  /**
   * @brief Property schema (CameraInfo/color channels, decimate, style, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Creates depth / CameraInfo / color readers.
   */
  void onEnable() override;

  /**
   * @brief Tears down readers and clears queues.
   */
  void onDisable() override;

  /**
   * @brief Clears depth, info, color, and projected points.
   */
  void reset() override;

  /**
   * @brief Drains queues and rebuilds points when new data arrives.
   */
  void onUpdate() override;

  /**
   * @brief Draws @c points_ as an overlay point set.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @struct ColoredPoint
   * @brief Internal projected point with display color (fixed/optical frame
   *        as produced by @ref rebuildPoints).
   */
  struct ColoredPoint {
    QVector3D position;  /**< 3D position for overlay drawing. */
    QColor color;        /**< Per-point RGBA. */
  };

  /**
   * @brief Reprojects @c depth_image_ with @c camera_info_ into @c points_.
   *
   * Reads decimation / flat color / color image from properties.
   */
  void rebuildPoints();

  /** Depth image topic. */
  std::string depth_channel_;

  /** CameraInfo topic (property-driven). */
  std::string camera_info_channel_;

  /** Optional color image topic. */
  std::string color_channel_;

  /** Depth payload queue. */
  integration::MessageQueue depth_queue_;

  /** CameraInfo payload queue. */
  integration::MessageQueue camera_info_queue_;

  /** Color image payload queue. */
  integration::MessageQueue color_queue_;

  /** Autolink reader for depth. */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> depth_reader_;

  /** Autolink reader for CameraInfo. */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      camera_info_reader_;

  /** Autolink reader for color image. */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> color_reader_;

  /** Latest depth image protobuf. */
  automsgs::msgs::sensor_msgs::Image depth_image_;

  /** Latest CameraInfo for projection. */
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;

  /** Latest decoded color image (may be null). */
  QImage color_image_;

  /** @c true when @c depth_image_ is valid. */
  bool have_depth_ = false;

  /** @c true when @c camera_info_ is valid. */
  bool have_camera_info_ = false;

  /** Cached projected cloud for @ref onDraw. */
  std::vector<ColoredPoint> points_;
};

}  // namespace display
}  // namespace autoviz
