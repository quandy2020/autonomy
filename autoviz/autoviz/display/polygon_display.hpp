/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file polygon_display.hpp
 * @brief Channel display for @c geometry_msgs/PolygonStamped.
 *
 * Draws a closed polyline (and optional fill) for the stamped polygon in the
 * fixed frame — aligned with RViz2 Polygon.
 *
 * ## Properties
 *
 * - @c color / @c alpha — stroke / fill appearance
 * - @c line_width — outline width
 *
 * @see ChannelDisplay
 * @see PathDisplay
 * @see PointStampedDisplay
 */

#pragma once

#include <vector>

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class PolygonDisplay
 * @brief Subscribes to PolygonStamped and draws the outline in fixed frame.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms each vertex (≥ 2 required).
 * - **Out:** @ref onDraw() connects @c points_ as a closed loop.
 */
class PolygonDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PolygonStamped> {
 public:
  /**
   * @brief Constructs a Polygon display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c geometry_msgs/PolygonStamped.
   */
  explicit PolygonDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "Polygon".
   */
  std::string typeId() const override { return "Polygon"; }

  /**
   * @brief Declares color, alpha, and line-width properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Transforms polygon vertices into the fixed frame.
   *
   * @param message Incoming @c geometry_msgs/PolygonStamped.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::PolygonStamped& message) override;

  /**
   * @brief Clears @c points_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws the cached polygon outline into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  std::vector<QVector3D> points_; /**< Fixed-frame polygon vertices. */
};

}  // namespace display
}  // namespace autoviz
