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
 * @file loop_aligner.hpp
 * @brief Place-recognition candidate and drift correction proposal.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "navigation/memory/context_view.hpp"

namespace navigation
{
namespace memory
{

/** @brief Best loop-closure match. */
struct LoopCandidate
{
    /** @brief Matched keyframe id. */
    std::uint64_t id{0};
    /** @brief Cosine similarity in [-1,1]. */
    double similarity{0.0};
    /** @brief Correction to apply to the query pose. */
    common::Pose2D correction;
};

class LoopAligner
{
public:
    /**
     * @brief Construct with matching thresholds.
     *
     * @param[in] similarity_threshold Minimum cosine similarity.
     * @param[in] max_distance Maximum pose distance in metres.
     */
    LoopAligner(double similarity_threshold = 0.9, double max_distance = 2.0);

    /**
     * @brief Find the best loop candidate.
     *
     * @param[in] query Current keyframe.
     * @param[in] memory Candidate keyframes.
     * @return Best match, or nullopt when none qualifies.
     */
    [[nodiscard]] std::optional<LoopCandidate> Find(
        const Keyframe& query, const std::vector<Keyframe>& memory) const;

private:
    /** @brief Minimum cosine similarity. */
    double similarity_threshold_{0.9};
    /** @brief Maximum pose distance in metres. */
    double max_distance_{2.0};
};

}  // namespace memory
}  // namespace navigation
