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

#include "autonomy/commsgs/proto/sensor_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"

#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace sensor {
 
using OdometryData = commsgs::planning_msgs::Odometry;
using OdometryProto = commsgs::proto::planning_msgs::Odometry;
 
// Converts 'data' to a proto::planning_msgs::Odometry.
OdometryProto ToProto(const OdometryData& data);

// Converts 'proto' to OdometryData.
OdometryData FromProto(const OdometryProto& proto);

}  // namespace sensor
}  // namespace autonomy