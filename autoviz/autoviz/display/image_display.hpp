/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_display.hpp
 * @brief Subscribes to @c sensor_msgs/Image and exposes a @c QImage for the
 *        Image panel (no 3D overlay).
 *
 * RViz Image display analogue for the 2D Image panel / dock. @ref onDraw is
 * intentionally empty; consumers call @ref image().
 *
 * @see ChannelDisplay
 * @see imageFromProto
 * @see CameraDisplay
 */

#pragma once

#include <QImage>

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class ImageDisplay
 * @brief Channel display that decodes images for 2D UI consumers.
 *
 * @note Does not contribute 3D geometry; the Image panel reads @ref image().
 *
 * @see imageFromProto
 * @see CameraDisplay
 */
class ImageDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::Image> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for Image messages.
   */
  explicit ImageDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Image".
   */
  std::string typeId() const override { return "Image"; }

  /**
   * @brief Latest decoded image (may be null).
   *
   * @return Const reference to @c image_.
   */
  const QImage& image() const { return image_; }

 protected:
  /**
   * @brief Decodes @p message into @c image_ via @ref imageFromProto.
   *
   * @param message sensor_msgs/Image protobuf.
   */
  void processMessage(
      const automsgs::msgs::sensor_msgs::Image& message) override;

  /**
   * @brief Clears @c image_.
   */
  void clearReceivedData() override;

  /**
   * @brief No-op; Image display has no 3D overlay.
   *
   * @param scene Unused.
   */
  void onDraw(rendering::SceneOverlay& /*scene*/) override {}

 private:
  /** Latest decoded frame for the Image panel. */
  QImage image_;
};

}  // namespace display
}  // namespace autoviz
