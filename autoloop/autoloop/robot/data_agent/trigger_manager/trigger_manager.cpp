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
 * @file trigger_manager.cpp
 * @brief Rule fan-in.
 */

#include "robot/data_agent/trigger_manager/trigger_manager.hpp"

#include "common/base/time_util.hpp"

namespace autoloop {
namespace robot {

void TriggerManager::AddRule(TriggerRule rule) { rules_.push_back(std::move(rule)); }

std::vector<TriggerEvent> TriggerManager::Observe(const Sample& sample) {
    std::vector<TriggerEvent> out;
    for (auto& r : rules_) {
        TriggerEvent ev;
        if (r.Match(sample, &ev)) {
            out.push_back(std::move(ev));
        }
    }
    return out;
}

TriggerEvent TriggerManager::Manual(const std::string& reason) {
    TriggerEvent ev;
    ev.kind = TriggerKind::kManual;
    ev.priority = Priority::kP2;
    ev.mono_ns = common::NowDualClock().mono_ns();
    ev.reason = reason;
    return ev;
}

}  // namespace robot
}  // namespace autoloop
