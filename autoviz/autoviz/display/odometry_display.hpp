/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file odometry_display.hpp
 * @brief Channel display for @c nav_msgs/Odometry pose (and optional velocity).
 *
 * Renders the latest odometry pose in the fixed frame as an arrow or axes,
 * optionally with a velocity vector — aligned with RViz2 Odometry.
 *
 * ## Properties
 *
 * - @c shape — @c Arrow or @c Axes
 * - @c color / @c alpha — pose visual appearance
 * - @c keep — history length (kept for RViz parity; current draw uses latest)
 * - @c position_tolerance / @c angle_tolerance — update filtering thresholds
 * - @c show_velocity — draw twist linear as a secondary arrow
 *
 * @see ChannelDisplay
 * @see PoseDisplay
 * @see TwistStampedDisplay
 */

#pragma once

#include <QVector3D>
#include <vector>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class OdometryDisplay
 * @brief Subscribes to @c nav_msgs/Odometry and draws pose (+ velocity).
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms the pose into the fixed frame
 *   and caches @ref OdometryVisual.
 * - **Out:** @ref onDraw() emits arrow/axes (and velocity) via Ogre/GL helpers.
 *
 * @note Owns no TF buffer; uses @c context_->tf_buffer from DisplayContext.
 */
class OdometryDisplay
    : public ChannelDisplay<automsgs::msgs::nav_msgs::Odometry> {
 public:
  /**
   * @brief Constructs an Odometry display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c nav_msgs/Odometry.
   */
  explicit OdometryDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "Odometry".
   */
  std::string typeId() const override { return "Odometry"; }

  /**
   * @brief Declares UI property specs (shape, color, tolerances, velocity).
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches pose and velocity from a new odometry message.
   *
   * Looks up TF from @c header.frame_id to the fixed frame, then stores
   * position, yaw, and linear velocity for drawing.
   *
   * @param message Incoming @c nav_msgs/Odometry.
   */
  void processMessage(
      const automsgs::msgs::nav_msgs::Odometry& message) override;

  /**
   * @brief Draws the cached odometry visual into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears @c has_odom_ so Time-panel Reset drops the visual.
   */
  void clearReceivedData() override;

 private:
  /**
   * @struct OdometryVisual
   * @brief Fixed-frame snapshot of the latest odometry sample.
   */
  struct OdometryVisual {
    QVector3D position;  /**< Pose translation in fixed frame. */
    QVector3D velocity;  /**< Linear twist (for Show Velocity). */
    float yaw = 0.f;     /**< Yaw (rad) extracted from orientation. */
  };

  OdometryVisual odom_;   /**< Latest transformed sample. */
  bool has_odom_ = false; /**< Whether @c odom_ is valid for drawing. */
};

}  // namespace display
}  // namespace autoviz
