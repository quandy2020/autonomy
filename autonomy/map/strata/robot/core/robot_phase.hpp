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

#include <unordered_map>
#include <unordered_set>

#include "autonomy/map/strata/constants.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace core {

bool CanTransition(RobotPhase from, RobotPhase to);
bool IsTerminal(RobotPhase phase);
bool IsActive(RobotPhase phase);
RobotStatus PhaseToRobotStatus(RobotPhase phase);

const std::unordered_map<RobotPhase, std::unordered_set<RobotPhase>>& PhaseTransitions();

}  // namespace core
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
