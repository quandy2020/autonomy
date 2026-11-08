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

#include "navigation/planning/plugins/policy/vntpa/context_builder.hpp"

#include <algorithm>

namespace navigation
{
namespace planning
{
namespace vntpa
{

ContextBuilder::ContextBuilder(std::uint32_t k, std::uint32_t /*embedding_dim*/,
                               std::uint32_t /*pose_dim*/)
    : k_(k)
{
}

bool ContextBuilder::Build(const memory::ContextView& view,
                           const common::Pose2D& /*current_pose*/,
                           ContextTensors* tensors) const
{
    if (tensors == nullptr)
    {
        return false;
    }
    tensors->k = k_;
    tensors->token_count = memory::kMaxTokenCount;
    tensors->token_dim = memory::kMaxTokenDim;
    tensors->tokens.assign(static_cast<std::size_t>(k_) * tensors->token_count *
                               tensors->token_dim,
                           0.0F);
    tensors->poses.assign(static_cast<std::size_t>(k_) * 3U, 0.0F);
    tensors->mask.assign(k_, 0);
    if (!view.Valid())
    {
        return true;
    }

    const auto& batch = view.Batch();
    const std::size_t count = std::min<std::size_t>(batch.size, k_);
    for (std::size_t i = 0; i < count; ++i)
    {
        const auto& entry = batch.entries[i];
        tensors->poses[i * 3U] = static_cast<float>(entry.pose.x());
        tensors->poses[i * 3U + 1U] = static_cast<float>(entry.pose.y());
        tensors->poses[i * 3U + 2U] = static_cast<float>(entry.pose.theta());
        if (entry.tokens != nullptr && entry.token_count > 0 && entry.token_dim > 0)
        {
            const std::size_t token_count =
                std::min<std::size_t>(entry.token_count, tensors->token_count);
            const std::size_t token_dim =
                std::min<std::size_t>(entry.token_dim, tensors->token_dim);
            for (std::size_t token = 0; token < token_count; ++token)
            {
                for (std::size_t dim = 0; dim < token_dim; ++dim)
                {
                    tensors->tokens[(i * tensors->token_count + token) *
                                        tensors->token_dim + dim] =
                        (*entry.tokens)[token * entry.token_dim + dim];
                }
            }
        }
        tensors->mask[i] = 1;
    }
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
