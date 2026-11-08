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
 * @file drift_compensator.hpp
 * @brief Applies a loop-closure correction to stored keyframes.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/memory/context_view.hpp"

namespace navigation
{
namespace memory
{

class DriftCompensator
{
public:
    /**
     * @brief Apply a loop-closure correction.
     *
     * @param[in] correction Map-frame correction transform.
     * @param[in,out] keyframes Poses are updated in place.
     * @param[in,out] epoch Incremented on success.
     * @return false on null/invalid input.
     */
    bool ApplyCorrection(const common::Pose2D& correction,
                         std::vector<Keyframe>* keyframes, std::uint32_t* epoch) const;
};

}  // namespace memory
}  // namespace navigation
