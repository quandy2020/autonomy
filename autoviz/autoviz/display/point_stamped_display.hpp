/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file point_stamped_display.hpp
 * @brief Channel display for @c geometry_msgs/PointStamped.
 *
 * Draws a sphere (or point marker) at the stamped point in the fixed frame —
 * aligned with RViz2 PointStamped.
 *
 * ## Properties
 *
 * - @c color / @c alpha — marker appearance
 * - @c radius — sphere radius in meters
 *
 * @see ChannelDisplay
 * @see PoseDisplay
 * @see PolygonDisplay
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/point_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class PointStampedDisplay
 * @brief Subscribes to PointStamped and draws a single colored point/sphere.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms the point into the fixed frame.
 * - **Out:** @ref onDraw() emits a sphere / point visual at @c position_.
 */
class PointStampedDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PointStamped> {
 public:
  /**
   * @brief Constructs a PointStamped display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c geometry_msgs/PointStamped.
   */
  explicit PointStampedDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "PointStamped".
   */
  std::string typeId() const override { return "PointStamped"; }

  /**
   * @brief Declares color, alpha, and radius properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches the fixed-frame position from a new message.
   *
   * @param message Incoming @c geometry_msgs/PointStamped.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::PointStamped& message) override;

  /**
   * @brief Clears @c have_point_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws the cached point into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_point_ = false; /**< Whether @c position_ is valid. */
  QVector3D position_;      /**< Fixed-frame point position. */
};

}  // namespace display
}  // namespace autoviz
