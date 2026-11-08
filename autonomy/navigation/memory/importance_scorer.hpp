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
 * @file importance_scorer.hpp
 * @brief Eviction score for keyframe memory.
 */

#pragma once

#include <cstdint>

#include "navigation/common/pose2d.hpp"
#include "navigation/memory/context_view.hpp"

namespace navigation
{
namespace memory
{

/** @brief Weights for keyframe eviction scoring. */
struct ImportanceOptions
{
    /** @brief Weight of the recency term. */
    double recency_weight{0.5};
    /** @brief Weight of the novelty term. */
    double novelty_weight{0.3};
    /** @brief Weight of the goal-proximity term. */
    double goal_weight{0.2};
    /** @brief Recency half-life in seconds. */
    double recency_halflife_s{30.0};
};

class ImportanceScorer
{
public:
    /** @brief Construct with scoring weights. */
    explicit ImportanceScorer(ImportanceOptions options = {});

    /**
     * @brief Score one keyframe for eviction.
     *
     * @param[in] keyframe Candidate.
     * @param[in] now_ns Current monotonic time.
     * @param[in] goal Current goal pose.
     * @return Higher values are more important.
     */
    [[nodiscard]] double Score(const Keyframe& keyframe, std::int64_t now_ns,
                               const common::Pose2D& goal) const;

private:
    /** @brief Scoring weights. */
    ImportanceOptions options_;
};

}  // namespace memory
}  // namespace navigation
