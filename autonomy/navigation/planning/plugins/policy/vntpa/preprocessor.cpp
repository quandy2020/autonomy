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
                         const memory::ContextView& context, ModelInputs* inputs)
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
    if (!context_builder_.Build(context, current_pose, &tensors))
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
