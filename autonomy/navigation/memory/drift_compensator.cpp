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

#include "navigation/memory/drift_compensator.hpp"


namespace navigation
{
namespace memory
{

bool DriftCompensator::ApplyCorrection(const common::Pose2D& correction,
                                       std::vector<Keyframe>* keyframes,
                                       std::uint32_t* epoch) const
{
    if (keyframes == nullptr || epoch == nullptr || !common::IsFinite(correction))
    {
        return false;
    }
    for (auto& keyframe : *keyframes)
    {
        keyframe.pose = common::Compose(correction, keyframe.pose);
    }
    ++(*epoch);
    for (auto& keyframe : *keyframes)
    {
        keyframe.epoch = *epoch;
    }
    return true;
}

}  // namespace memory
}  // namespace navigation
