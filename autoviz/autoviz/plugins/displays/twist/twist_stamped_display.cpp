/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins TwistStampedDisplay
 * (Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/twist/twist_stamped_display.hpp"

namespace autoviz {
namespace display {
namespace {

QVector3D ProtoVector3(
    const automsgs::msgs::geometry_msgs::Vector3& vector) {
  return QVector3D(static_cast<float>(vector.x()),
                   static_cast<float>(vector.y()),
                   static_cast<float>(vector.z()));
}

}  // namespace

TwistStampedDisplay::TwistStampedDisplay(std::string channel)
    : ScrewDisplay<automsgs::msgs::geometry_msgs::TwistStamped>(
          "TwistStamped", std::move(channel),
          "automsgs.msgs.geometry_msgs.TwistStamped") {}

void TwistStampedDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::TwistStamped& message) {
  processMessagePrivate(message.header().frame_id(),
                        ProtoVector3(message.twist().linear()),
                        ProtoVector3(message.twist().angular()));
}

}  // namespace display
}  // namespace autoviz
