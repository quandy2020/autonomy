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

automsgs::msgs::visualization_msgs::Marker CreateSlowdownMarker(
    const automsgs::msgs::geometry_msgs::PoseStamped& motion_target,
    const double& slowdown_radius) {
    automsgs::msgs::visualization_msgs::Marker slowdown_marker;
    *slowdown_marker.mutable_header() = motion_target.header();
    slowdown_marker.set_ns("slowdown");
    slowdown_marker.set_id(0);
    slowdown_marker.set_type(automsgs::msgs::visualization_msgs::Marker::SPHERE);    // SPHERE
    slowdown_marker.set_action(automsgs::msgs::visualization_msgs::Marker::ADD);  // ADD
    *slowdown_marker.mutable_pose() = motion_target.pose();
    slowdown_marker.mutable_pose()->mutable_position()->set_z(0.01);
    slowdown_marker.mutable_scale()->set_x(slowdown_radius * 2.0);
    slowdown_marker.mutable_scale()->set_y(slowdown_radius * 2.0);
    slowdown_marker.mutable_scale()->set_z(0.02);
    slowdown_marker.mutable_color()->set_a(0.2);
    slowdown_marker.mutable_color()->set_r(0.0);
    slowdown_marker.mutable_color()->set_g(1.0);
    slowdown_marker.mutable_color()->set_b(0.0);
    return slowdown_marker;
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy
