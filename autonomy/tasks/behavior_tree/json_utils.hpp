/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <set>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "behaviortree_cpp/json_export.h"

// The follow templates are required when using Groot 2 to visualize the BT.
// They convert the data types into JSON format easy for visualization.

namespace autonomy::commsgs::builtin_interfaces {

BT_JSON_CONVERTER(autonomy::commsgs::builtin_interfaces::Time, msg) {
    add_field("sec", &msg.sec);
    add_field("nanosec", &msg.nanosec);
}

}  // namespace autonomy::commsgs::builtin_interfaces

namespace autonomy::commsgs::std_msgs {

BT_JSON_CONVERTER(autonomy::commsgs::std_msgs::Header, msg) {
    add_field("stamp", &msg.stamp);
    add_field("frame_id", &msg.frame_id);
}

}  // namespace autonomy::commsgs::std_msgs

namespace autonomy::commsgs::geometry_msgs {

BT_JSON_CONVERTER(autonomy::commsgs::geometry_msgs::Point, msg) {
    add_field("x", &msg.x);
    add_field("y", &msg.y);
    add_field("z", &msg.z);
}

BT_JSON_CONVERTER(autonomy::commsgs::geometry_msgs::Quaternion, msg) {
    add_field("x", &msg.x);
    add_field("y", &msg.y);
    add_field("z", &msg.z);
    add_field("w", &msg.w);
}

BT_JSON_CONVERTER(autonomy::commsgs::geometry_msgs::Pose, msg) {
    add_field("position", &msg.position);
    add_field("orientation", &msg.orientation);
}

BT_JSON_CONVERTER(autonomy::commsgs::geometry_msgs::PoseStamped, msg) {
    add_field("header", &msg.header);
    add_field("pose", &msg.pose);
}

}  // namespace autonomy::commsgs::geometry_msgs

namespace autonomy::commsgs::planning_msgs {

BT_JSON_CONVERTER(autonomy::commsgs::planning_msgs::Path, msg) {
    add_field("header", &msg.header);
    add_field("poses", &msg.poses);
}

BT_JSON_CONVERTER(autonomy::commsgs::planning_msgs::Goals, msg) {
    add_field("header", &msg.header);
    add_field("goals", &msg.goals);
}

}  // namespace autonomy::commsgs::planning_msgs

namespace std {

inline void from_json(const nlohmann::json& js,
                      std::chrono::milliseconds& dest) {
    if (js.contains("ms")) {
        dest = std::chrono::milliseconds(js.at("ms").get<int>());
    } else {
        throw std::runtime_error("Invalid JSON for std::chrono::milliseconds");
    }
}

inline void to_json(nlohmann::json& js, const std::chrono::milliseconds& src) {
    js["__type"] = "std::chrono::milliseconds";
    js["ms"] = src.count();
}

}  // namespace std