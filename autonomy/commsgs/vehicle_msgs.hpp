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

#include "autonomy/commsgs/proto/vehicle_msgs.pb.h"

namespace autonomy {
namespace commsgs {
namespace vehicle_msgs {

struct RobotEvent
{

};

struct RobotState
{

};

// Converts 'data' to a proto::vehicle_msgs::RobotEvent.
proto::vehicle_msgs::RobotEvent ToProto(const RobotEvent& data);

// Converts 'proto' to RobotEvent.
RobotEvent FromProto(const proto::vehicle_msgs::RobotEvent& proto);

// Converts 'data' to a proto::vehicle_msgs::RobotState.
proto::vehicle_msgs::RobotState ToProto(const RobotState& data);

// Converts 'proto' to RobotState.
RobotState FromProto(const proto::vehicle_msgs::RobotState& proto);


}  // namespace vehicle_msgs
}  // namespace commsgs
}  // namespace autonomy