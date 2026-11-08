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

namespace navigation
{
namespace planning
{
namespace vntpa
{

bool ActionDecoder::Decode(const std::vector<float>& raw_actions,
                           std::uint32_t chunk_length, std::vector<float>* actions) const
{
    if (actions == nullptr || chunk_length == 0 ||
        raw_actions.size() < 3U * chunk_length)
    {
        return false;
    }
    actions->assign(raw_actions.begin(), raw_actions.begin() + 3U * chunk_length);
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
