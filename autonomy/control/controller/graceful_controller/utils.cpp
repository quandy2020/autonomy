/*
 * Copyright 2025 The Openbot Authors
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

#include "autonomy/control/controller/graceful_controller/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {

commsgs::visualization_msgs::Marker CreateSlowdownMarker(
    const commsgs::geometry_msgs::PoseStamped& motion_target,
    const double& slowdown_radius) {
    commsgs::visualization_msgs::Marker slowdown_marker;
    slowdown_marker.header = motion_target.header;
    slowdown_marker.ns = "slowdown";
    slowdown_marker.id = 0;
    slowdown_marker.type = 2;    // SPHERE
    slowdown_marker.action = 0;  // ADD
    slowdown_marker.pose = motion_target.pose;
    slowdown_marker.pose.position.z = 0.01;
    slowdown_marker.scale.x = slowdown_radius * 2.0;
    slowdown_marker.scale.y = slowdown_radius * 2.0;
    slowdown_marker.scale.z = 0.02;
    slowdown_marker.color.a = 0.2;
    slowdown_marker.color.r = 0.0;
    slowdown_marker.color.g = 1.0;
    slowdown_marker.color.b = 0.0;
    return slowdown_marker;
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy