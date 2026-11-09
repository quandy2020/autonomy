/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file node_context.hpp
 * @brief Inputs shared by the seven navigation behavior nodes.
 */

#pragma once

#include <functional>
#include <string_view>

#include "navigation/planning/interface/navigation_session.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation {
namespace behavior {

/** @brief Inputs shared by navigation behavior nodes. */
struct NodeContext {
    /** @brief Navigation session; borrowed, may be null. */
    planning::NavigationSession* session{nullptr};
    /** @brief Goal to submit. */
    GoalSpec goal;
    /** @brief Pose freshness/continuity check result. */
    bool pose_valid{true};
    /** @brief True when the safety layer has taken over. */
    bool safety_intervention{false};
    /** @brief Navigation/safety/memory health. */
    bool module_healthy{true};
    /** @brief Planner reachability result. */
    bool goal_reachable{true};
    /** @brief Localization/relocalization validity. */
    bool relocalized{true};
    /** @brief Callback used to request operator assistance. */
    std::function<void(std::string_view)> request_assist;
    /** @brief Latch set after the first assistance request. */
    bool assist_requested{false};
    /** @brief Current tick on the host's steady clock. */
    std::int64_t now_ns{0};
};

}  // namespace behavior
}  // namespace navigation
