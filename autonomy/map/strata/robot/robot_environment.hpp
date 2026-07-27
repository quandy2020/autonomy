/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <string>
#include <unordered_map>

namespace autonomy {
namespace map {
namespace strata {
namespace robot {

struct ElevatorState {
    std::string floor;
    std::string direction{"idle"};
    bool doors_open{false};
};

/** BICMap WaitContext 环境快照：传感器、区域、电梯、红绿灯。 */
struct RobotEnvironment {
    std::unordered_map<std::string, double> sensors;
    std::unordered_map<std::string, std::string> sensor_strings;
    std::unordered_map<std::string, std::string> zones;
    std::unordered_map<std::string, ElevatorState> elevators;
    std::unordered_map<std::string, std::string> traffic_lights;
};

}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
