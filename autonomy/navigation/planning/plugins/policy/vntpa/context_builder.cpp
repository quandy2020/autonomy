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

ContextBuilder::ContextBuilder(std::uint32_t k, std::uint32_t embedding_dim,
                               std::uint32_t pose_dim)
    : k_(k), embedding_dim_(embedding_dim), pose_attention_(pose_dim)
{
}

bool ContextBuilder::Build(const memory::ContextView& view,
                           const common::Pose2D& current_pose,
                           ContextTensors* tensors) const
{
    if (tensors == nullptr)
    {
        return false;
    }
    tensors->k = k_;
    tensors->embedding_dim = embedding_dim_;
    tensors->pose_dim = pose_attention_.OutputDim();
    tensors->embeddings.assign(static_cast<std::size_t>(k_) * embedding_dim_, 0.0F);
    tensors->poses.assign(static_cast<std::size_t>(k_) * tensors->pose_dim, 0.0F);
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
        const std::size_t copy_dim =
            std::min<std::size_t>(entry.embedding_dim, embedding_dim_);
        for (std::size_t dim = 0; dim < copy_dim; ++dim)
        {
            tensors->embeddings[i * embedding_dim_ + dim] = entry.embedding[dim];
        }
        std::vector<float> pose_encoding;
        const common::Pose2D relative =
            common::Compose(common::Inverse(current_pose), entry.pose);
        if (pose_attention_.Encode(relative, &pose_encoding))
        {
            for (std::size_t dim = 0; dim < pose_encoding.size(); ++dim)
            {
                tensors->poses[i * tensors->pose_dim + dim] = pose_encoding[dim];
            }
        }
        tensors->mask[i] = 1;
    }
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
