/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file trigger_rule.cpp
 * @brief Rising-edge flag match.
 */

#include "robot/data_agent/trigger_manager/trigger_rule.hpp"

namespace autoloop {
namespace robot {

TriggerRule::TriggerRule(TriggerKind kind, Priority prio, uint16_t required_flags)
    : kind_(kind), prio_(prio), required_flags_(required_flags) {}

bool TriggerRule::Match(const Sample& sample, TriggerEvent* event) {
    const bool hit = required_flags_ != 0 &&
                     (sample.flags & required_flags_) == required_flags_;
    if (!hit) {
        latched_ = false;
        return false;
    }
    if (latched_) {
        return false;
    }
    latched_ = true;
    if (event != nullptr) {
        event->kind = kind_;
        event->priority = prio_;
        event->mono_ns = sample.clock.mono_ns();
        event->reason = TriggerKindName(kind_);
    }
    return true;
}

}  // namespace robot
}  // namespace autoloop
