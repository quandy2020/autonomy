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
 * @file observation.cpp
 * @brief Accept observations bound to evidence_node_id; JSONL append.
 */

#include "autonomy/system/recorder/observation.hpp"

#include <fstream>

#include "nlohmann/json.hpp"

namespace autonomy {
namespace system {
namespace recorder {

bool AcceptObservation(const proto::Observation& observation) {
    return observation.evidence_node_ids_size() > 0 &&
           !observation.description().empty();
}

bool AppendObservationJsonl(const std::string& path,
                            const proto::Observation& observation) {
    if (!AcceptObservation(observation)) {
        return false;
    }
    nlohmann::json ids = nlohmann::json::array();
    for (const auto& id : observation.evidence_node_ids()) {
        ids.push_back(id);
    }
    nlohmann::json j = {{"observation_id", observation.observation_id()},
                        {"description", observation.description()},
                        {"bt_node", observation.bt_node()},
                        {"start_monotonic_ns", observation.start_monotonic_ns()},
                        {"end_monotonic_ns", observation.end_monotonic_ns()},
                        {"evidence_node_ids", ids}};
    std::ofstream out(path, std::ios::app);
    if (!out) {
        return false;
    }
    out << j.dump() << '\n';
    return true;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
