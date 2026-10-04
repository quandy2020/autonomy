/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file twist_stamped_display.hpp
 * @brief Channel display for @c geometry_msgs/TwistStamped.
 *
 * Draws linear and angular components as an RViz ScrewVisual (or GL fallback)
 * at the message frame origin — aligned with RViz2 TwistStamped.
 *
 * ## Properties
 *
 * - @c linear_color / @c angular_color
 * - @c linear_scale / @c angular_scale
 * - @c arrow_width / @c hide_small_values
 *
 * @see ChannelDisplay
 * @see drawScrewOgreOrGl()
 * @see WrenchDisplay
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class TwistStampedDisplay
 * @brief Subscribes to TwistStamped and draws linear/angular screw arrows.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms the frame origin and rotates
 *   twist vectors into the fixed frame.
 * - **Out:** @ref onDraw() calls @ref drawScrewOgreOrGl.
 */
class TwistStampedDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::TwistStamped> {
 public:
  /**
   * @brief Constructs a TwistStamped display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c geometry_msgs/TwistStamped.
   */
  explicit TwistStampedDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "TwistStamped".
   */
  std::string typeId() const override { return "TwistStamped"; }

  /**
   * @brief Declares color, scale, width, and hide-small-values properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches origin + linear/angular vectors from @p message.
   *
   * @param message Incoming @c geometry_msgs/TwistStamped.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::TwistStamped& message) override;

  /**
   * @brief Clears @c have_twist_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws the cached twist as a screw visual into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_twist_ = false; /**< Whether twist fields are valid. */
  QVector3D origin_;        /**< Frame origin in fixed frame. */
  QVector3D linear_;        /**< Linear twist (fixed frame). */
  QVector3D angular_;       /**< Angular twist (fixed frame). */
};

}  // namespace display
}  // namespace autoviz
