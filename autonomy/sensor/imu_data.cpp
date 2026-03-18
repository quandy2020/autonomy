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

#include "autonomy/sensor/imu_data.hpp"

namespace autonomy {
namespace sensor {

ImuProto ToProto(const ImuData& data) { return commsgs::sensor_msgs::ToProto(data); }

ImuData FromProto(const ImuProto& proto) { return commsgs::sensor_msgs::FromProto(proto); }

}  // namespace sensor
}  // namespace autonomy