/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file pose_array_display.hpp
 * @brief Channel display for @c geometry_msgs/PoseArray.
 *
 * Draws each pose as an arrow or axes in the fixed frame — aligned with
 * RViz2 PoseArray.
 *
 * ## Properties
 *
 * - @c shape — @c Arrow or @c Axes
 * - @c color / @c alpha — appearance
 * - @c axis_length / @c axis_radius — size controls
 *
 * @see ChannelDisplay
 * @see PoseDisplay
 * @see PathDisplay
 */

#pragma once

#include <vector>

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class PoseArrayDisplay
 * @brief Subscribes to PoseArray and draws every pose as Arrow or Axes.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms each pose (position + yaw).
 * - **Out:** @ref onDraw() emits one visual per @ref StoredPose.
 */
class PoseArrayDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PoseArray> {
 public:
  /**
   * @brief Constructs a PoseArray display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c geometry_msgs/PoseArray.
   */
  explicit PoseArrayDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "PoseArray".
   */
  std::string typeId() const override { return "PoseArray"; }

  /**
   * @brief Declares shape, color, and size properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Replaces @c poses_ with TF-transformed samples from @p message.
   *
   * @param message Incoming @c geometry_msgs/PoseArray.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseArray& message)
      override;

  /**
   * @brief Clears @c poses_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws all stored poses into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @struct StoredPose
   * @brief Compact fixed-frame pose (position + yaw) for drawing.
   */
  struct StoredPose {
    QVector3D position; /**< Pose translation. */
    float yaw = 0.f;    /**< Yaw (rad) from orientation. */
  };

  std::vector<StoredPose> poses_; /**< Latest transformed pose array. */
};

}  // namespace display
}  // namespace autoviz
