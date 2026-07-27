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

#include "autonomy/map/strata/robot/core/robot_phase.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace robot {
namespace core {

const std::unordered_map<RobotPhase, std::unordered_set<RobotPhase>>& PhaseTransitions() {
    static const std::unordered_map<RobotPhase, std::unordered_set<RobotPhase>> transitions = {
        {RobotPhase::kIdle,
         {RobotPhase::kMoving, RobotPhase::kRotating, RobotPhase::kWaiting, RobotPhase::kCharging,
          RobotPhase::kDocking, RobotPhase::kError, RobotPhase::kManual}},
        {RobotPhase::kMoving,
         {RobotPhase::kArrived, RobotPhase::kRotating, RobotPhase::kWaiting, RobotPhase::kIdle,
          RobotPhase::kReturning, RobotPhase::kPaused, RobotPhase::kError}},
        {RobotPhase::kRotating, {RobotPhase::kMoving, RobotPhase::kIdle, RobotPhase::kError}},
        {RobotPhase::kArrived,
         {RobotPhase::kIdle, RobotPhase::kMoving, RobotPhase::kDwelling, RobotPhase::kWaiting,
          RobotPhase::kCharging, RobotPhase::kDocking, RobotPhase::kError, RobotPhase::kPaused}},
        {RobotPhase::kDwelling,
         {RobotPhase::kMoving, RobotPhase::kIdle, RobotPhase::kError, RobotPhase::kPaused}},
        {RobotPhase::kWaiting,
         {RobotPhase::kMoving, RobotPhase::kIdle, RobotPhase::kError, RobotPhase::kPaused}},
        {RobotPhase::kCharging,
         {RobotPhase::kIdle, RobotPhase::kMoving, RobotPhase::kError, RobotPhase::kPaused}},
        {RobotPhase::kDocking,
         {RobotPhase::kCharging, RobotPhase::kIdle, RobotPhase::kMoving, RobotPhase::kError,
          RobotPhase::kPaused}},
        {RobotPhase::kError, {RobotPhase::kRecovering, RobotPhase::kIdle, RobotPhase::kManual}},
        {RobotPhase::kRecovering, {RobotPhase::kMoving, RobotPhase::kError, RobotPhase::kIdle}},
        {RobotPhase::kReturning,
         {RobotPhase::kDocking, RobotPhase::kCharging, RobotPhase::kIdle, RobotPhase::kError,
          RobotPhase::kPaused}},
        {RobotPhase::kManual,
         {RobotPhase::kIdle, RobotPhase::kMoving, RobotPhase::kError, RobotPhase::kRecovering}},
        {RobotPhase::kPaused,
         {RobotPhase::kMoving, RobotPhase::kDwelling, RobotPhase::kWaiting, RobotPhase::kIdle}},
    };
    return transitions;
}

bool CanTransition(RobotPhase from, RobotPhase to) {
    const auto& transitions = PhaseTransitions();
    const auto it = transitions.find(from);
    return it != transitions.end() && it->second.count(to);
}

bool IsTerminal(RobotPhase phase) {
    return phase == RobotPhase::kIdle || phase == RobotPhase::kError ||
           phase == RobotPhase::kManual || phase == RobotPhase::kPaused;
}

bool IsActive(RobotPhase phase) { return !IsTerminal(phase); }

RobotStatus PhaseToRobotStatus(RobotPhase phase) {
    switch (phase) {
        case RobotPhase::kMoving:
        case RobotPhase::kRotating:
        case RobotPhase::kDocking:
        case RobotPhase::kManual:
            return RobotStatus::kRunning;
        case RobotPhase::kCharging:
            return RobotStatus::kCharging;
        case RobotPhase::kError:
        case RobotPhase::kRecovering:
            return RobotStatus::kError;
        case RobotPhase::kReturning:
            return RobotStatus::kReturning;
        default:
            return RobotStatus::kIdle;
    }
}

}  // namespace core
}  // namespace robot
}  // namespace strata
}  // namespace map
}  // namespace autonomy
