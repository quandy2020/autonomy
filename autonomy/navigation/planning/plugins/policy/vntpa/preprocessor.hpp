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
 * @brief Assembles VNTPA model inputs from feature, goal and context.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/memory/context_view.hpp"
#include "navigation/planning/plugins/policy/vntpa/context_builder.hpp"
#include "navigation/planning/plugins/policy/vntpa/query_builder.hpp"
#include "navigation/proto/navigation.pb.h"
#include "navigation/proto/observation.pb.h"

namespace navigation
{
namespace planning
{
namespace vntpa
{

/** @brief Fixed-shape tensors passed to the VNTPA model. */
struct ModelInputs
{
    /** @brief N * D depth tokens. */
    std::vector<float> depth_feature;
    /** @brief Query token of length D. */
    std::vector<float> query;
    /** @brief K * D context embeddings. */
    std::vector<float> context;
    /** @brief K * P Fourier context poses. */
    std::vector<float> context_pose;
    /** @brief K context mask bytes. */
    std::vector<std::uint8_t> context_mask;
    /** @brief Number of depth tokens. */
    std::uint32_t depth_token_count{0};
    /** @brief Depth token dimension. */
    std::uint32_t depth_token_dim{0};
    /** @brief Context capacity K. */
    std::uint32_t context_size{0};
};

class Preprocessor
{
public:
    /** @brief Construct with fixed tensor dimensions. */
    Preprocessor(std::uint32_t context_size, std::uint32_t embedding_dim,
                 std::uint32_t pose_dim);

    /**
     * @brief Assemble all model inputs.
     *
     * @param[in] depth_feature Encoder output.
     * @param[in] current_pose Current map pose.
     * @param[in] goal Active goal.
     * @param[in] context Memory snapshot.
     * @param[out] inputs Fixed-shape tensors.
     */
    bool Build(const observation::DepthFeature& depth_feature,
               const common::Pose2D& current_pose, const GoalSpec& goal,
               const memory::ContextView& context, ModelInputs* inputs);

private:
    /** @brief Query token builder. */
    QueryBuilder query_builder_;
    /** @brief Context tensor builder. */
    ContextBuilder context_builder_;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
