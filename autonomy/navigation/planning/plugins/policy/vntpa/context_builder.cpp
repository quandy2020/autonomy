/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/planning/plugins/policy/vntpa/context_builder.hpp"

#include "navigation/common/habitat_pose.hpp"

#include <algorithm>

namespace navigation
{
namespace planning
{
namespace vntpa
{
namespace
{

void WritePose(std::vector<float>* poses, std::size_t index, const common::Pose2D& ros_pose)
{
    const common::Pose2D habitat = common::RosToHabitat(ros_pose);
    (*poses)[index * 3U] = static_cast<float>(habitat.x());
    (*poses)[index * 3U + 1U] = static_cast<float>(habitat.y());
    (*poses)[index * 3U + 2U] = static_cast<float>(habitat.theta());
}

void WriteTokens(std::vector<float>* tokens, std::size_t index, std::uint32_t token_count,
                 std::uint32_t token_dim, const std::vector<float>& source,
                 std::uint32_t source_count, std::uint32_t source_dim)
{
    const std::size_t count = std::min<std::size_t>(source_count, token_count);
    const std::size_t dim = std::min<std::size_t>(source_dim, token_dim);
    for (std::size_t token = 0; token < count; ++token)
    {
        for (std::size_t d = 0; d < dim; ++d)
        {
            (*tokens)[(index * token_count + token) * token_dim + d] =
                source[token * source_dim + d];
        }
    }
}

}  // namespace

ContextBuilder::ContextBuilder(std::uint32_t k, std::uint32_t /*embedding_dim*/,
                               std::uint32_t /*pose_dim*/)
    : k_(k)
{
}

bool ContextBuilder::Build(const memory::ContextView& view,
                           const common::Pose2D& current_pose, ContextTensors* tensors,
                           const std::vector<float>* live_tokens,
                           std::uint32_t live_token_count,
                           std::uint32_t live_token_dim) const
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

    std::size_t written = 0;
    const bool have_live = live_tokens != nullptr && live_token_count > 0 &&
                           live_token_dim > 0 &&
                           live_tokens->size() >= live_token_count * live_token_dim;
    if (have_live && written < k_)
    {
        WritePose(&tensors->poses, written, current_pose);
        WriteTokens(&tensors->tokens, written, tensors->token_count, tensors->token_dim,
                    *live_tokens, live_token_count, live_token_dim);
        tensors->mask[written] = 1;
        ++written;
    }
    if (!view.Valid())
    {
        return true;
    }

    const auto& batch = view.Batch();
    for (std::size_t i = 0; i < batch.size && written < k_; ++i)
    {
        const auto& entry = batch.entries[i];
        const double dx = entry.pose.x() - current_pose.x();
        const double dy = entry.pose.y() - current_pose.y();
        if (have_live && dx * dx + dy * dy < 0.05 * 0.05)
        {
            continue;
        }
        WritePose(&tensors->poses, written, entry.pose);
        if (entry.tokens != nullptr && entry.token_count > 0 && entry.token_dim > 0)
        {
            WriteTokens(&tensors->tokens, written, tensors->token_count, tensors->token_dim,
                        *entry.tokens, entry.token_count, entry.token_dim);
        }
        tensors->mask[written] = 1;
        ++written;
    }
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
