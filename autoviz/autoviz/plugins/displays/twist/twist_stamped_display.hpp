/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins TwistStampedDisplay
 * (Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file twist_stamped_display.hpp
 * @brief Channel display for @c geometry_msgs/TwistStamped (RViz TwistStamped).
 *
 * Thin subclass of @ref ScrewDisplay — processMessage forwards linear/angular
 * into the shared screw visual path.
 *
 * @see ScrewDisplay
 * @see AccelStampedDisplay
 */

#pragma once

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include "autoviz/plugins/displays/screw/screw_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class TwistStampedDisplay
 * @brief ScrewDisplay specialization for TwistStamped messages.
 */
class TwistStampedDisplay
    : public ScrewDisplay<automsgs::msgs::geometry_msgs::TwistStamped> {
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

 protected:
  /**
   * @brief Forwards twist linear/angular into @ref processMessagePrivate.
   *
   * @param message Incoming @c geometry_msgs/TwistStamped.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::TwistStamped& message) override;
};

}  // namespace display
}  // namespace autoviz
