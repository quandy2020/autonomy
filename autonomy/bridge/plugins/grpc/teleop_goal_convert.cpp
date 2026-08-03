/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/bridge/plugins/grpc/teleop_goal_convert.hpp"

#include <automsgs/msgs/vehicle_msgs/vehicle_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {

namespace tp = ::autonomy::task::proto;

tp::TeleopGoal ToTaskTeleopGoal(const proto::TeleopCommandRequest& request) {
    tp::TeleopGoal goal;
    goal.set_command(static_cast<tp::TeleopCommand>(request.command()));

    if (request.has_header()) {
        auto* header = goal.mutable_header();
        if (request.header().has_timestamp()) {
            *header->mutable_stamp() = request.header().timestamp();
        }
        header->set_task_id(request.header().cmd_id());
        header->set_client_id(request.header().client_id());
        header->set_task_type(
            ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_TELEOP);
    }

    if (request.has_velocity()) {
        *goal.mutable_velocity() = request.velocity();
    }
    if (request.has_session_timeout()) {
        *goal.mutable_session_timeout() = request.session_timeout();
    }
    if (request.max_linear_speed() > 0.f) {
        goal.set_max_linear_speed(request.max_linear_speed());
    }
    if (request.max_angular_speed() > 0.f) {
        goal.set_max_angular_speed(request.max_angular_speed());
    }
    if (request.watchdog_timeout_sec() > 0.f) {
        goal.set_watchdog_timeout_sec(request.watchdog_timeout_sec());
    }
    goal.set_disable_collision_checks(request.disable_collision_checks());
    return goal;
}

}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
