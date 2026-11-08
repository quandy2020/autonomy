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
 * @file pose_indexed_memory.hpp
 * @brief Default pose-indexed keyframe memory provider.
 */

#pragma once

#include <cstdint>

#include "navigation/memory/context_provider.hpp"
#include "navigation/memory/keyframe_filter.hpp"
#include "navigation/proto/plugins.pb.h"

namespace navigation
{
namespace planning
{

class PoseIndexedMemory final : public memory::ContextProvider
{
public:
    /** @brief Construct the pose-indexed KNN provider. */
    explicit PoseIndexedMemory(plugins::PoseIndexedMemoryOptions options);

    /** @brief Insert one keyframe. */
    common::Status Insert(const memory::Keyframe& keyframe) override;

    /** @brief Return the nearest context batch. */
    memory::ContextView View(const common::Pose2D& pose) const override;

    /** @brief Number of stored keyframes. */
    std::size_t Size() const override;

    /** @brief Drop stored frames; optionally keep the pose index. */
    void Reset(bool keep_pose_index) override;

    /** @brief Validate epoch/pose consistency. */
    ConsistencyReport CheckConsistency(const common::Pose2D& pose,
                                       std::uint32_t epoch) const override;

private:
    memory::KeyframeFilter filter_;
    std::uint32_t epoch_{1};
};

}  // namespace planning
}  // namespace navigation
