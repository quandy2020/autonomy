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
 * @file query_builder.hpp
 * @brief Builds the pose-and-goal query token.
 */

#pragma once

#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/planning/plugins/policy/vntpa/pose_attention.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{
namespace vntpa
{

class QueryBuilder
{
public:
    /** @brief Construct with the query feature dimension. */
    explicit QueryBuilder(std::uint32_t output_dim = 64);

    /** @brief Build the query token from the goal relative to the robot. */
    bool Build(const common::Pose2D& current_pose, const GoalSpec& goal,
               std::vector<float>* query, bool* has_goal);

    /** @brief Configured output dimension. */
    [[nodiscard]] std::uint32_t OutputDim() const;

private:
    PoseAttention pose_attention_;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
