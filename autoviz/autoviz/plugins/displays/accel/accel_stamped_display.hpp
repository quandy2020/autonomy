/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins AccelStampedDisplay
 * (Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file accel_stamped_display.hpp
 * @brief Channel display for @c geometry_msgs/AccelStamped (RViz AccelStamped).
 *
 * Thin subclass of @ref ScrewDisplay — processMessage forwards linear/angular
 * acceleration into the shared screw visual path.
 *
 * @see ScrewDisplay
 * @see TwistStampedDisplay
 */

#pragma once

#include <automsgs/msgs/geometry_msgs/accel_stamped.pb.h>
#include "autoviz/plugins/displays/screw/screw_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class AccelStampedDisplay
 * @brief ScrewDisplay specialization for AccelStamped messages.
 */
class AccelStampedDisplay
    : public ScrewDisplay<automsgs::msgs::geometry_msgs::AccelStamped> {
 public:
  /**
   * @brief Constructs an AccelStamped display bound to @p channel.
   *
   * @param channel Autolink channel for AccelStamped messages.
   */
  explicit AccelStampedDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   * @return Always @c "AccelStamped".
   */
  std::string typeId() const override { return "AccelStamped"; }

 protected:
  /**
   * @brief Forwards accel linear/angular into @ref processMessagePrivate.
   *
   * @param message Latest AccelStamped protobuf.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::AccelStamped& message) override;
};

}  // namespace display
}  // namespace autoviz
