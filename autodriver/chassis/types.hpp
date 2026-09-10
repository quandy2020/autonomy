/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Chassis types = automsgs vehicle_msgs (+ TwistStamped command).
 *
 * Wire / driver message bodies stay identical to
 * `automsgs/msgs/vehicle_msgs/*` — no parallel ChassisCommand/State structs.
 */

#ifndef AUTODRIVER_CHASSIS_TYPES_HPP_
#define AUTODRIVER_CHASSIS_TYPES_HPP_

#include <string>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_event.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_state.pb.h>

namespace autodriver {
namespace chassis {

/** Stable chassis instance id, e.g. "chassis/base". */
using ChassisId = std::string;

/** Command body: same as RobotState.twist (geometry_msgs). */
using ChassisCommand = ::automsgs::msgs::geometry_msgs::TwistStamped;

/** State body: vehicle_msgs.RobotState. */
using ChassisState = ::automsgs::msgs::vehicle_msgs::RobotState;

/** Event body: vehicle_msgs.RobotEvent. */
using ChassisEvent = ::automsgs::msgs::vehicle_msgs::RobotEvent;

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_TYPES_HPP_
