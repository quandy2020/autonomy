/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <automsgs/msgs/geometry_msgs/twist.pb.h>

#include "autoviz/ui/teleop/teleop_types.hpp"

namespace autoviz {
namespace teleop {

automsgs::msgs::geometry_msgs::Twist ZeroTwist();
automsgs::msgs::geometry_msgs::Twist TwistFromButton(
    const TeleopButtonConfig& button);

QString twistFieldLabel(TeleopTwistField field);
TeleopTwistField twistFieldFromIndex(int index);

}  // namespace teleop
}  // namespace autoviz
