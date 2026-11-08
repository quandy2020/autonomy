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

#include "navigation/planning/plugins/policy/vntpa/action_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace navigation
{
namespace planning
{
namespace vntpa
{

ActionDecoder::ActionDecoder(ActionDecoderOptions options) : options_(options)
{
    if (options_.action_count < 4)
    {
        options_.action_count = 4;
    }
    if (options_.dt_s <= 0.0)
    {
        options_.dt_s = 0.5;
    }
}

bool ActionDecoder::Decode(const std::vector<float>& raw_actions,
                           std::uint32_t chunk_length, std::vector<float>* actions,
                           std::vector<std::uint32_t>* kinds) const
{
    if (actions == nullptr || chunk_length == 0)
    {
        return false;
    }
    actions->assign(static_cast<std::size_t>(chunk_length) * 3U, 0.0F);
    if (kinds != nullptr)
    {
        kinds->clear();
    }

    if (raw_actions.size() >= static_cast<std::size_t>(chunk_length) * 3U &&
        raw_actions.size() < static_cast<std::size_t>(chunk_length) * options_.action_count)
    {
        // Legacy continuous output [chunk, 3].
        std::copy_n(raw_actions.begin(), static_cast<std::size_t>(chunk_length) * 3U,
                    actions->begin());
        return true;
    }
    if (raw_actions.size() < static_cast<std::size_t>(chunk_length) * options_.action_count)
    {
        return false;
    }

    if (kinds != nullptr)
    {
        kinds->resize(chunk_length);
    }
    for (std::uint32_t i = 0; i < chunk_length; ++i)
    {
        const auto begin = raw_actions.begin() + static_cast<std::ptrdiff_t>(
                               i * options_.action_count);
        const auto best = std::max_element(
            begin, begin + static_cast<std::ptrdiff_t>(options_.action_count));
        const auto kind = static_cast<std::uint32_t>(
            std::distance(begin, best));
        if (kinds != nullptr)
        {
            (*kinds)[i] = kind;
        }
        switch (kind)
        {
            case 1:  // FORWARD
                (*actions)[3U * i] =
                    static_cast<float>(options_.forward_step / options_.dt_s);
                break;
            case 2:  // TURN_LEFT
                (*actions)[3U * i + 2U] =
                    static_cast<float>(options_.turn_step_rad / options_.dt_s);
                break;
            case 3:  // TURN_RIGHT
                (*actions)[3U * i + 2U] =
                    static_cast<float>(-options_.turn_step_rad / options_.dt_s);
                break;
            default:  // STOP
                break;
        }
    }
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
