/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file types.hpp
 * @brief Chassis wire types = automsgs vehicle_msgs (+ TwistStamped command).
 *
 * In-process abstractions (LocomotionModel, Capability, OperationalMode,
 * MotionCommand, SafetyGate, ToolCommand) live in sibling headers — they do
 * not duplicate RobotState fields.
 */

#ifndef AUTODRIVER_CHASSIS_TYPES_HPP_
#define AUTODRIVER_CHASSIS_TYPES_HPP_

#include <string>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_event.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_state.pb.h>

namespace autodriver {
namespace chassis {

/**
 * @brief Stable chassis instance id from YAML, e.g. "chassis/base".
 */
using ChassisId = std::string;

/**
 * @brief Velocity command body: geometry_msgs.TwistStamped
 *        (same shape as RobotState.twist).
 */
using ChassisCommand = ::automsgs::msgs::geometry_msgs::TwistStamped;

/**
 * @brief Feedback body: vehicle_msgs.RobotState (pose / twist / battery / flags).
 */
using ChassisState = ::automsgs::msgs::vehicle_msgs::RobotState;

/**
 * @brief Async event body: vehicle_msgs.RobotEvent (FAULT / E-STOP / BATTERY_LOW).
 */
using ChassisEvent = ::automsgs::msgs::vehicle_msgs::RobotEvent;

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_TYPES_HPP_
