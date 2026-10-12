/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins AccelStampedDisplay
 * (Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/accel/accel_stamped_display.hpp"

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

AccelStampedDisplay::AccelStampedDisplay(std::string channel)
    : ScrewDisplay<automsgs::msgs::geometry_msgs::AccelStamped>(
          "AccelStamped", std::move(channel),
          "automsgs.msgs.geometry_msgs.AccelStamped") {}

void AccelStampedDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::AccelStamped& message) {
  processMessagePrivate(message.header().frame_id(),
                        ProtoVector3(message.accel().linear()),
                        ProtoVector3(message.accel().angular()));
}

}  // namespace display
}  // namespace autoviz
