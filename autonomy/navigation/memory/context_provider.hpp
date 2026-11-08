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
 * @file context_provider.hpp
 * @brief Spatial memory boundary used by Runner.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "navigation/common/pose2d.hpp"
#include "navigation/common/status.hpp"
#include "navigation/memory/context_view.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace memory
{

/**
 * @brief Pose-indexed memory provider.
 *
 * @par Threading
 * Insert/Reset run on the navigation thread; View returns an immutable handle
 * that may be read by the policy worker.
 */
class ContextProvider
{
public:
    virtual ~ContextProvider() = default;

    /**
     * @brief Insert one keyframe.
     *
     * @param[in] keyframe Feature + pose + metadata.
     * @return kOk; providers may silently reject low-novelty frames.
     */
    virtual common::Status Insert(const Keyframe& keyframe) = 0;

    /**
     * @brief Query the K nearest keyframes around a pose.
     *
     * @param[in] pose Query pose.
     * @return Immutable batch; may be empty but valid.
     */
    virtual ContextView View(const common::Pose2D& pose) const = 0;

    /** @brief Number of stored keyframes. */
    [[nodiscard]] virtual std::size_t Size() const = 0;

    /**
     * @brief Drop stored keyframes.
     *
     * @param[in] keep_pose_index Keep the spatial index when true.
     */
    virtual void Reset(bool keep_pose_index) = 0;

    /**
     * @brief Validate epoch/frame/context consistency.
     *
     * @param[in] pose Current pose.
     * @param[in] epoch Caller epoch.
     */
    virtual ConsistencyReport CheckConsistency(const common::Pose2D& pose,
                                               std::uint32_t epoch) const = 0;
};

}  // namespace memory
}  // namespace navigation
