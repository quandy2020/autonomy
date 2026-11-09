/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/planning/plugins/policy/vntpa/preprocessor.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

Preprocessor::Preprocessor(std::uint32_t context_size, std::uint32_t embedding_dim,
                           std::uint32_t pose_dim)
    : context_builder_(context_size, embedding_dim, pose_dim)
{
}

bool Preprocessor::Build(const common::Pose2D& current_pose, const GoalSpec& goal,
                         const memory::ContextView& context, ModelInputs* inputs,
                         const observation::DepthFeature* live_feature,
                         bool include_live_view)
{
    if (inputs == nullptr)
    {
        return false;
    }
    bool has_goal = false;
    if (!query_builder_.Build(current_pose, goal, &inputs->query_pose, &inputs->goal,
                              &has_goal) ||
        !has_goal)
    {
        return false;
    }
    ContextTensors tensors;
    const std::vector<float>* live_tokens = nullptr;
    std::vector<float> live_buffer;
    std::uint32_t live_count = 0;
    std::uint32_t live_dim = 0;
    if (include_live_view && live_feature != nullptr && live_feature->token_count() > 0 &&
        live_feature->token_dim() > 0 &&
        static_cast<std::size_t>(live_feature->tokens_size()) >=
            live_feature->token_count() * live_feature->token_dim())
    {
        live_buffer.assign(live_feature->tokens().begin(), live_feature->tokens().end());
        live_tokens = &live_buffer;
        live_count = live_feature->token_count();
        live_dim = live_feature->token_dim();
    }
    if (!context_builder_.Build(context, current_pose, &tensors, live_tokens, live_count,
                                live_dim))
    {
        return false;
    }
    inputs->kf_tokens = std::move(tensors.tokens);
    inputs->kf_pose = std::move(tensors.poses);
    inputs->kf_valid = std::move(tensors.mask);
    inputs->token_count = tensors.token_count;
    inputs->token_dim = tensors.token_dim;
    inputs->context_size = tensors.k;
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
