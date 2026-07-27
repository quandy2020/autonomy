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

#include "autonomy/map/strata/robot/core/waypoint_types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace core {

std::string ToString(WaypointType type) {
    switch (type) {
        case WaypointType::kPatrol: return "patrol";
        case WaypointType::kPoi: return "poi";
        case WaypointType::kWaitElevator: return "wait_elevator";
        case WaypointType::kWaitTraffic: return "wait_traffic";
        case WaypointType::kWaitDoor: return "wait_door";
        case WaypointType::kWaitSignal: return "wait_signal";
        case WaypointType::kCharging: return "charging";
        case WaypointType::kDock: return "dock";
        case WaypointType::kFloorTransition: return "floor_transition";
        case WaypointType::kService: return "service";
        case WaypointType::kHold: return "hold";
        case WaypointType::kHandover: return "handover";
    }
    return "patrol";
}

void WaypointTypeRegistry::Register(WaypointType type, std::function<void(Waypoint&)> handler) {
    handlers_[type] = std::move(handler);
}

bool WaypointTypeRegistry::Invoke(WaypointType type, Waypoint& waypoint) const {
    const auto it = handlers_.find(type);
    if (it == handlers_.end()) {
        return false;
    }
    it->second(waypoint);
    return true;
}

}  // namespace core
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
