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
 * @file keyframe_filter.hpp
 * @brief Novelty, time and motion gating for keyframe insertion.
 */

#pragma once

#include <cstddef>
#include <vector>

#include "navigation/memory/context_view.hpp"

namespace navigation
{
namespace memory
{

/** @brief Keyframe acceptance thresholds. */
struct KeyframeFilterOptions
{
    /** @brief Hard capacity before importance eviction. */
    std::size_t max_entries{5000};
    /** @brief Minimum novelty (1 - cosine similarity). */
    double novelty_threshold_eta{0.08};
    /** @brief Minimum time between accepted keyframes in seconds. */
    double min_interval_s{0.2};
    /** @brief Minimum translation between accepted keyframes in metres. */
    double min_motion{0.05};
    /** @brief Minimum rotation between accepted keyframes in radians. */
    double min_motion_rad{0.05};
    /** @brief Neighbourhood radius for novelty comparison in metres. */
    double search_radius{2.0};
};

class KeyframeFilter
{
public:
    /** @brief Construct with acceptance thresholds. */
    explicit KeyframeFilter(KeyframeFilterOptions options = {});

    /**
     * @brief Try to accept a keyframe.
     *
     * @param[in] keyframe Candidate feature + pose.
     * @param[out] accepted True when the keyframe was stored.
     * @return false on malformed input.
     */
    bool TryAccept(const Keyframe& keyframe, bool* accepted);

    /** @brief Read-only view of stored keyframes. */
    [[nodiscard]] const std::vector<Keyframe>& Entries() const;

    /** @brief Number of stored keyframes. */
    [[nodiscard]] std::size_t Size() const;

    /** @brief Clear all keyframes. */
    void Reset();

private:
    /**
     * @brief Compute 1 - maximum cosine similarity to stored neighbours.
     *
     * @param[in] keyframe Candidate feature + pose.
     */
    double Novelty(const Keyframe& keyframe) const;

    /** @brief Acceptance thresholds. */
    KeyframeFilterOptions options_;
    /** @brief Stored keyframes. */
    std::vector<Keyframe> entries_;
    /** @brief Timestamp of the last accepted keyframe in seconds. */
    double last_accept_s_{-1e9};
};

}  // namespace memory
}  // namespace navigation
