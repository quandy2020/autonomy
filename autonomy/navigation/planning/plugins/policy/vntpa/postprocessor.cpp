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

#include "navigation/planning/plugins/policy/vntpa/postprocessor.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace planning
{
namespace vntpa
{

Postprocessor::Postprocessor(PostprocessorOptions options) : options_(options)
{
}

bool Postprocessor::Build(const std::vector<float>& actions, std::uint32_t chunk_length,
                          std::uint64_t sequence, std::int64_t stamp_ns,
                          ActionChunk* chunk,
                          const std::vector<std::uint32_t>* kinds) const
{
    if (chunk == nullptr || chunk_length == 0 || actions.size() < 3U * chunk_length)
    {
        return false;
    }
    for (float value : actions)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    const std::uint32_t count = std::min({chunk_length, options_.chunk_length,
                                          static_cast<std::uint32_t>(actions.size() / 3)});
    chunk->Clear();
    chunk->set_dt_s(options_.dt_s);
    chunk->set_sequence(sequence);
    chunk->set_produced_at_ns(stamp_ns);
    chunk->set_frame("base_link");
    for (std::uint32_t i = 0; i < count; ++i)
    {
        auto* action = chunk->add_actions();
        action->set_x(
            std::clamp(static_cast<double>(actions[3 * i]), -options_.max_linear_mps,
                       options_.max_linear_mps));
        action->set_y(
            std::clamp(static_cast<double>(actions[3 * i + 1]), -options_.max_lateral_mps,
                       options_.max_lateral_mps));
        action->set_theta(
            std::clamp(static_cast<double>(actions[3 * i + 2]), -options_.max_angular_radps,
                       options_.max_angular_radps));
        if (kinds != nullptr && i < kinds->size())
        {
            auto* primitive = chunk->add_primitives();
            primitive->set_kind(static_cast<ActionKind>((*kinds)[i]));
            if ((*kinds)[i] == 1U)
            {
                primitive->set_distance(options_.forward_step);
            }
            else if ((*kinds)[i] == 2U || (*kinds)[i] == 3U)
            {
                primitive->set_angle((*kinds)[i] == 2U ? options_.turn_step_rad
                                                       : -options_.turn_step_rad);
            }
        }
    }
    return true;
}

const PostprocessorOptions& Postprocessor::Options() const
{
    return options_;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
