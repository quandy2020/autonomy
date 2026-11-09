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
 * @file context_builder.hpp
 * @brief Packs a KeyframeBatch into fixed-shape context tensors.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/memory/context_view.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

/** @brief Row-major context tensors sent to the transformer. */
struct ContextTensors
{
    /** @brief K * embedding_dim features. */
    std::vector<float> embeddings;
    /** @brief K * 3 Habitat keyframe poses (x, z, yaw). */
    std::vector<float> poses;
    /** @brief K * P * D keyframe patch tokens. */
    std::vector<float> tokens;
    /** @brief K mask bytes; 1 means a valid entry. */
    std::vector<std::uint8_t> mask;
    /** @brief Context capacity K. */
    std::uint32_t k{0};
    /** @brief Patch-token count per keyframe. */
    std::uint32_t token_count{0};
    /** @brief Patch-token dimension. */
    std::uint32_t token_dim{0};
};

class ContextBuilder
{
public:
    /** @brief Construct with fixed tensor dimensions. */
    ContextBuilder(std::uint32_t k, std::uint32_t embedding_dim,
                   std::uint32_t pose_dim);

    /**
     * @brief Build context tensors from a memory view.
     *
     * @param[in] live_tokens Optional current-view tokens prepended as slot 0.
     */
    bool Build(const memory::ContextView& view, const common::Pose2D& current_pose,
               ContextTensors* tensors,
               const std::vector<float>* live_tokens = nullptr,
               std::uint32_t live_token_count = 0,
               std::uint32_t live_token_dim = 0) const;

private:
    /** @brief Context capacity. */
    std::uint32_t k_{32};
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
