/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file camera_info_display.hpp
 * @brief Draws a CameraInfo frustum wireframe in 3D (RViz CameraInfo display).
 *
 * Subscribes to @c sensor_msgs/CameraInfo and renders near/far edges via
 * @ref buildCameraFrustumSegments. Unlike @ref CameraDisplay, no image
 * texture is shown.
 *
 * @see CameraDisplay
 * @see camera_utils.hpp
 * @see ChannelDisplay
 */

#pragma once

#include <string>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class CameraInfoDisplay
 * @brief Channel display for CameraInfo frustum visualization.
 *
 * Properties typically include far distance, edge visibility, color, and
 * alpha.
 *
 * @see CameraDisplay
 * @see buildCameraFrustumSegments
 */
class CameraInfoDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::CameraInfo> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for CameraInfo messages.
   */
  explicit CameraInfoDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "CameraInfo".
   */
  std::string typeId() const override { return "CameraInfo"; }

  /**
   * @brief Property schema (far distance, edges, color, alpha, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches the latest CameraInfo message.
   *
   * @param message sensor_msgs/CameraInfo protobuf.
   */
  void processMessage(
      const automsgs::msgs::sensor_msgs::CameraInfo& message) override;

  /**
   * @brief Clears @c camera_info_ and @c have_info_.
   */
  void clearReceivedData() override;

  /**
   * @brief Draws frustum segments when @c have_info_ is set.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /** @c true after a successful @ref processMessage since reset. */
  bool have_info_ = false;

  /** Latest CameraInfo used for frustum geometry. */
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;
};

}  // namespace display
}  // namespace autoviz
