/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file accel_stamped_display.hpp
 * @brief Displays @c geometry_msgs/AccelStamped as linear and angular
 *        acceleration arrows at the message frame origin.
 *
 * RViz AccelStamped analogue: two colored arrows (linear / angular) scaled by
 * display properties and drawn via @ref appendSolidArrowMeshes.
 *
 * @see ChannelDisplay
 * @see ImuDisplay
 * @see arrow_mesh_utils.hpp
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/accel_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class AccelStampedDisplay
 * @brief Channel display for @c geometry_msgs/AccelStamped acceleration
 *        arrows.
 *
 * Caches origin (TF of header frame) plus linear and angular vectors from the
 * latest message; @ref onDraw emits solid arrows into the scene overlay.
 *
 * @see ChannelDisplay
 * @see ImuDisplay
 */
class AccelStampedDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::AccelStamped> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for AccelStamped messages.
   */
  explicit AccelStampedDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "AccelStamped".
   */
  std::string typeId() const override { return "AccelStamped"; }

  /**
   * @brief Property schema (colors, scales, alpha, etc.).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches linear/angular accel and resolves the header frame origin.
   *
   * @param message Latest AccelStamped protobuf.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::AccelStamped& message) override;

  /**
   * @brief Clears cached accel vectors and @c have_accel_.
   */
  void clearReceivedData() override;

  /**
   * @brief Draws linear and angular acceleration arrows when data is valid.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /** @c true after a successful @ref processMessage since reset. */
  bool have_accel_ = false;

  /** Arrow origin in fixed frame (header frame pose). */
  QVector3D origin_;

  /** Linear acceleration vector (message frame / fixed, as processed). */
  QVector3D linear_;

  /** Angular acceleration vector. */
  QVector3D angular_;
};

}  // namespace display
}  // namespace autoviz
