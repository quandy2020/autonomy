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
#include "navigation/planning/plugins/policy/vntpa/pose_attention.hpp"

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
    /** @brief K * pose_dim Fourier pose features. */
    std::vector<float> poses;
    /** @brief K mask bytes; 1 means a valid entry. */
    std::vector<std::uint8_t> mask;
    /** @brief Context capacity K. */
    std::uint32_t k{0};
    /** @brief Feature dimension. */
    std::uint32_t embedding_dim{0};
    /** @brief Pose feature dimension. */
    std::uint32_t pose_dim{0};
};

class ContextBuilder
{
public:
    /** @brief Construct with fixed tensor dimensions. */
    ContextBuilder(std::uint32_t k, std::uint32_t embedding_dim,
                   std::uint32_t pose_dim);

    /** @brief Build context tensors from a memory view. */
    bool Build(const memory::ContextView& view, const common::Pose2D& current_pose,
               ContextTensors* tensors) const;

private:
    /** @brief Context capacity. */
    std::uint32_t k_{32};
    /** @brief Feature dimension. */
    std::uint32_t embedding_dim_{384};
    PoseAttention pose_attention_;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
