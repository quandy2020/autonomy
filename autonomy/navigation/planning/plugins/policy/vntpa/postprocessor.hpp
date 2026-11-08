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
 * @file postprocessor.hpp
 * @brief Clamps raw model actions and builds an ActionChunk.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{
namespace vntpa
{

/** @brief Chunk construction and clamping options. */
struct PostprocessorOptions
{
    /** @brief Actions per chunk. */
    std::uint32_t chunk_length{8};
    /** @brief Seconds per action. */
    double dt_s{0.1};
    /** @brief Forward speed limit in m/s. */
    double max_linear_mps{0.5};
    /** @brief Lateral speed limit in m/s. */
    double max_lateral_mps{0.0};
    /** @brief Yaw-rate limit in rad/s. */
    double max_angular_radps{1.0};
    /** @brief Forward primitive step in metres. */
    double forward_step{0.25};
    /** @brief Turn primitive angle in radians. */
    double turn_step_rad{0.5235987756};
};

class Postprocessor
{
public:
    /** @brief Construct with chunk/clamp options. */
    explicit Postprocessor(PostprocessorOptions options);

    /** @brief Clamp flat actions and build an ActionChunk. */
    bool Build(const std::vector<float>& actions, std::uint32_t chunk_length,
               std::uint64_t sequence, std::int64_t stamp_ns, ActionChunk* chunk,
               const std::vector<std::uint32_t>* kinds = nullptr) const;

    /** @brief Read-only options snapshot. */
    [[nodiscard]] const PostprocessorOptions& Options() const;

private:
    /** @brief Clamp/chunk options. */
    PostprocessorOptions options_;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
