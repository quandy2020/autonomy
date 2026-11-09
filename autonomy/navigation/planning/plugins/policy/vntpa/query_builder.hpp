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
 * @brief Builds raw query pose and goal inputs; RoPE lives in the ONNX graph.
 */

#pragma once

#include <vector>

#include "navigation/common/pose2d.hpp"
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
    /** @brief Build Habitat query pose (x, z, yaw) and goal (x, z). */
    bool Build(const common::Pose2D& current_pose, const GoalSpec& goal,
               std::vector<float>* query_pose, std::vector<float>* goal_xy,
               bool* has_goal) const;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
