// `automsgs.msgs.pkg.Type` → `pkg/Type`（Foxglove / ROS 风格显示名）。
#pragma once

#include <string>

namespace autoviz {
namespace converter {

// Map an automsgs proto full name (e.g. "automsgs.msgs.sensor_msgs.Imu")
// to a human-friendly type string used in foxglove (e.g. "sensor_msgs/Imu").
std::string RosStyleTypeName(const std::string& automsgs_full_name);

}  // namespace converter
}  // namespace autoviz

