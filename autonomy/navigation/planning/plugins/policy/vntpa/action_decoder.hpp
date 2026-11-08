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
 * @file action_decoder.hpp
 * @brief Decodes VNT-PA logits into discrete primitives or legacy velocities.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace navigation
{
namespace planning
{
namespace vntpa
{

/** @brief Discrete action mapping used by the decoder. */
struct ActionDecoderOptions
{
    /** @brief Number of action classes (STOP/FORWARD/LEFT/RIGHT). */
    std::uint32_t action_count{4};
    /** @brief Forward step in metres. */
    double forward_step{0.25};
    /** @brief Turn step in radians. */
    double turn_step_rad{0.5235987756};
    /** @brief Seconds represented by one action. */
    double dt_s{0.5};
};

class ActionDecoder
{
public:
    /** @brief Construct with action mapping. */
    explicit ActionDecoder(ActionDecoderOptions options = {});

    /**
     * @brief Decode logits [chunk, 4] or legacy velocities [chunk, 3].
     *
     * @param[out] actions Three floats per action.
     * @param[out] kinds Optional discrete class per action (empty for legacy input).
     */
    bool Decode(const std::vector<float>& raw_actions, std::uint32_t chunk_length,
                std::vector<float>* actions,
                std::vector<std::uint32_t>* kinds = nullptr) const;

private:
    /** @brief Mapping options. */
    ActionDecoderOptions options_;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
