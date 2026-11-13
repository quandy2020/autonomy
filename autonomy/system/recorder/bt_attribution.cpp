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
 * @file bt_attribution.cpp
 * @brief L3: attribute failure to the last FAILURE BT transition.
 */

#include "autonomy/system/recorder/bt_attribution.hpp"

namespace autonomy {
namespace system {
namespace recorder {

std::optional<proto::BtFailure> AttributeBtFailure(
    const std::vector<BtTransition>& transitions) {
    for (auto it = transitions.rbegin(); it != transitions.rend(); ++it) {
        if (it->current != "FAILURE") {
            continue;
        }
        proto::BtFailure fail;
        fail.set_node_name(it->node_name);
        fail.set_previous_status(it->previous);
        fail.set_current_status(it->current);
        *fail.mutable_clock() = it->clock;
        fail.set_condition(it->previous == "RUNNING" ? "post" : "pre");
        return fail;
    }
    return std::nullopt;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
