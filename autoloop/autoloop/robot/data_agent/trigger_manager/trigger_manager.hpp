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
 * @file trigger_manager.hpp
 * @brief Evaluate configured rules; never publishes /cmd_vel.
 */

#include <string>
#include <vector>

#include "robot/data_agent/trigger_manager/trigger_rule.hpp"

namespace autoloop {
namespace robot {

class TriggerManager {
public:
    void AddRule(TriggerRule rule);

    /**
     * @brief Feed one sample; collect newly fired events.
     * @param [in] sample: Bypass sample.
     */
    std::vector<TriggerEvent> Observe(const Sample& sample);

    /**
     * @brief Force a manual snapshot request.
     * @param [in] reason: Operator note.
     */
    TriggerEvent Manual(const std::string& reason);

private:
    std::vector<TriggerRule> rules_;
};

}  // namespace robot
}  // namespace autoloop
