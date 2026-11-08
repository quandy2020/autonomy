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
 * @file preprocessor.hpp
 * @brief Assembles VNT-PA keyframe/query/goal tensors.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/memory/context_view.hpp"
#include "navigation/planning/plugins/policy/vntpa/context_builder.hpp"
#include "navigation/planning/plugins/policy/vntpa/query_builder.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{
namespace vntpa
{

/** @brief Fixed-shape tensors passed to the single VNT-PA planner ONNX. */
struct ModelInputs
{
    /** @brief K * P * D keyframe patch tokens. */
    std::vector<float> kf_tokens;
    /** @brief K * 3 keyframe poses. */
    std::vector<float> kf_pose;
    /** @brief K mask bytes; 1 means a valid keyframe. */
    std::vector<std::uint8_t> kf_valid;
    /** @brief Query pose (x, y, yaw). */
    std::vector<float> query_pose;
    /** @brief Goal position (x, y). */
    std::vector<float> goal;
    /** @brief Patch-token count per keyframe. */
    std::uint32_t token_count{0};
    /** @brief Patch-token dimension. */
    std::uint32_t token_dim{0};
    /** @brief Context capacity K. */
    std::uint32_t context_size{0};
};

class Preprocessor
{
public:
    /** @brief Construct with fixed context size; other dimensions come from tokens. */
    Preprocessor(std::uint32_t context_size, std::uint32_t embedding_dim,
                 std::uint32_t pose_dim);

    /** @brief Assemble keyframe tokens, poses, query and goal. */
    bool Build(const common::Pose2D& current_pose, const GoalSpec& goal,
               const memory::ContextView& context, ModelInputs* inputs);

private:
    /** @brief Query pose/goal builder. */
    QueryBuilder query_builder_;
    /** @brief Context tensor builder. */
    ContextBuilder context_builder_;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
