/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file wrench_display.hpp
 * @brief Channel display for @c geometry_msgs/WrenchStamped.
 *
 * Draws force and torque at the message frame origin as an RViz WrenchVisual
 * (or GL fallback) — aligned with RViz2 Wrench.
 *
 * ## Properties
 *
 * - @c force_color / @c torque_color
 * - @c force_scale / @c torque_scale
 * - @c arrow_width
 *
 * @see ChannelDisplay
 * @see drawWrenchOgreOrGl()
 * @see TwistStampedDisplay
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/wrench_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class WrenchDisplay
 * @brief Subscribes to WrenchStamped and draws force/torque arrows.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms origin and rotates force /
 *   torque into the fixed frame.
 * - **Out:** @ref onDraw() calls @ref drawWrenchOgreOrGl.
 */
class WrenchDisplay
    : public ChannelDisplay<
          automsgs::msgs::geometry_msgs::WrenchStamped> {
 public:
  /**
   * @brief Constructs a Wrench display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c geometry_msgs/WrenchStamped.
   */
  explicit WrenchDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "Wrench".
   */
  std::string typeId() const override { return "Wrench"; }

  /**
   * @brief Declares force/torque color, scale, and width properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches origin + force/torque from @p message.
   *
   * @param message Incoming @c geometry_msgs/WrenchStamped.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::WrenchStamped& message)
      override;

  /**
   * @brief Clears @c have_wrench_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws the cached wrench into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_wrench_ = false; /**< Whether wrench fields are valid. */
  QVector3D origin_;         /**< Frame origin in fixed frame. */
  QVector3D force_;          /**< Force vector (fixed frame). */
  QVector3D torque_;         /**< Torque vector (fixed frame). */
};

}  // namespace display
}  // namespace autoviz
