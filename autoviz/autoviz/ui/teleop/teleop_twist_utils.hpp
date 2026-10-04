/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_twist_utils.hpp
 * @brief Helpers to build @c geometry_msgs::Twist from teleop button configs.
 *
 * Used by @ref TeleopPanel when composing discrete-button commands and by
 * settings UI for field labels.
 *
 * @see TeleopButtonConfig
 * @see TeleopTwistField
 * @see TeleopPanel
 */

#pragma once

#include <automsgs/msgs/geometry_msgs/twist.pb.h>

#include "autoviz/ui/teleop/teleop_types.hpp"

namespace autoviz {
namespace teleop {

/**
 * @brief Returns a Twist with all linear and angular components set to zero.
 *
 * @return Zero velocity command.
 */
automsgs::msgs::geometry_msgs::Twist ZeroTwist();

/**
 * @brief Builds a Twist that sets a single field from @p button.
 *
 * All other components remain zero. The scalar at @ref TeleopButtonConfig::field
 * is set to @ref TeleopButtonConfig::value.
 *
 * @param button Field + value mapping for one discrete teleop button.
 * @return Twist with one non-zero component (unless @c value is 0).
 * @see ZeroTwist()
 */
automsgs::msgs::geometry_msgs::Twist TwistFromButton(
    const TeleopButtonConfig& button);

/**
 * @brief Human-readable label for a Twist field (e.g. for settings combos).
 *
 * @param field Twist scalar discriminator.
 * @return Localized / display string such as @c "linear.x".
 */
QString twistFieldLabel(TeleopTwistField field);

/**
 * @brief Maps a combo-box index to a @ref TeleopTwistField.
 *
 * @param index Zero-based index matching the settings field combo order.
 * @return Corresponding @ref TeleopTwistField (clamped / defaulted if invalid).
 */
TeleopTwistField twistFieldFromIndex(int index);

}  // namespace teleop
}  // namespace autoviz
