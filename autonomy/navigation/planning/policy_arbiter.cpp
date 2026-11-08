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

#include "navigation/planning/policy_arbiter.hpp"

#include <algorithm>

namespace navigation
{
namespace planning
{

std::string PolicyArbiter::Select(const std::vector<PolicyCandidate>& candidates)
{
    const PolicyCandidate* best = nullptr;
    for (const auto& candidate : candidates)
    {
        if (!candidate.healthy || candidate.name.empty())
        {
            continue;
        }
        if (best == nullptr || candidate.priority > best->priority ||
            (candidate.priority == best->priority &&
             candidate.confidence > best->confidence) ||
            (candidate.priority == best->priority &&
             candidate.confidence == best->confidence &&
             candidate.last_success_ns > best->last_success_ns))
        {
            best = &candidate;
        }
    }
    return best != nullptr ? best->name : std::string{};
}

}  // namespace planning
}  // namespace navigation
