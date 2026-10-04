/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file pose_display.hpp
 * @brief Channel display for @c geometry_msgs/PoseStamped.
 *
 * Draws a single pose as an arrow or axes in the fixed frame — aligned with
 * RViz2 Pose.
 *
 * ## Properties
 *
 * - @c shape — @c Arrow or @c Axes
 * - @c color / @c alpha — appearance
 * - Arrow: @c shaft_length, @c head_length, @c shaft_diameter, @c head_diameter
 * - Axes: @c axis_length, @c axis_radius
 *
 * @see ChannelDisplay
 * @see PoseArrayDisplay
 * @see PoseWithCovarianceDisplay
 * @see OdometryDisplay
 */

#pragma once

#include <optional>

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class PoseDisplay
 * @brief Subscribes to PoseStamped and draws one Arrow or Axes visual.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms position and yaw.
 * - **Out:** @ref onDraw() draws according to the Shape property.
 */
class PoseDisplay
    : public ChannelDisplay<
          automsgs::msgs::geometry_msgs::PoseStamped> {
 public:
  /**
   * @brief Constructs a Pose display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c geometry_msgs/PoseStamped.
   */
  explicit PoseDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "Pose".
   */
  std::string typeId() const override { return "Pose"; }

  /**
   * @brief Declares shape, color, and arrow/axes size properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches fixed-frame position and yaw from @p message.
   *
   * @param message Incoming @c geometry_msgs/PoseStamped.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseStamped& message)
      override;

  /**
   * @brief Draws the cached pose into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears @c have_pose_ (Time-panel Reset).
   */
  void clearReceivedData() override;

 private:
  bool have_pose_ = false; /**< Whether pose fields are valid. */
  QVector3D position_;     /**< Fixed-frame translation. */
  float yaw_ = 0.f;        /**< Yaw (rad) from orientation. */
};

}  // namespace display
}  // namespace autoviz
