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

#include "autonomy/control/utils/conversions.hpp"

namespace autonomy {
namespace control {
namespace utils {

commsgs::geometry_msgs::Twist twist2Dto3D(const commsgs::planning_msgs::Twist2D& cmd_vel_2d) {
    commsgs::geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = cmd_vel_2d.x;
    cmd_vel.linear.y = cmd_vel_2d.y;
    cmd_vel.angular.z = cmd_vel_2d.theta;
    return cmd_vel;
}

commsgs::planning_msgs::Twist2D twist3Dto2D(const commsgs::geometry_msgs::Twist& cmd_vel) {
    commsgs::planning_msgs::Twist2D cmd_vel_2d;
    cmd_vel_2d.x = cmd_vel.linear.x;
    cmd_vel_2d.y = cmd_vel.linear.y;
    cmd_vel_2d.theta = cmd_vel.angular.z;
    return cmd_vel_2d;
}

}  // namespace utils
}  // namespace control
}  // namespace autonomy