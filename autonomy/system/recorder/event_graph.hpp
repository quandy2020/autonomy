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
 * @file event_graph.hpp
 * @brief L2 event graph: BT transitions + triggers linked by time and 2 s context.
 *
 * Edges are temporal + context, not physical causality.
 */

#include <vector>

#include "autonomy/system/recorder/episode.hpp"
#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Build an EventGraph from BT transitions and trigger events.
 *
 * Nodes are transitions and triggers. Edges connect predecessors within a 2 s
 * window (temporal) plus shared-channel context.
 *
 * @param [in] transitions: BT status transitions in the episode.
 * @param [in] triggers: Trigger events in the episode.
 * @return EventGraph proto.
 */
proto::EventGraph BuildEventGraph(
    const std::vector<BtTransition>& transitions,
    const std::vector<proto::TriggerEvent>& triggers);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
