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

#pragma once

/**
 * @file trigger_rule.hpp
 * @brief Single trigger predicate over SampleFlag bits.
 */

#include "robot/data_agent/ring_buffer/ring_buffer.hpp"
#include "robot/data_agent/trigger_manager/trigger_types.hpp"

namespace autoloop {
namespace robot {

class TriggerRule {
public:
    TriggerRule(TriggerKind kind, Priority prio, uint16_t required_flags);

    /**
     * @brief Evaluate a sample against this rule.
     * @param [in] sample: Latest bypass sample.
     * @param [out] event: Filled on match.
     * @return True on rising-edge match.
     */
    bool Match(const Sample& sample, TriggerEvent* event);

private:
    TriggerKind kind_;
    Priority prio_;
    uint16_t required_flags_{0};
    bool latched_{false};
};

}  // namespace robot
}  // namespace autoloop
