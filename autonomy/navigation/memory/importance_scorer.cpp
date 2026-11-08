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

#include "navigation/memory/importance_scorer.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace memory
{

ImportanceScorer::ImportanceScorer(ImportanceOptions options) : options_(options)
{
}

double ImportanceScorer::Score(const Keyframe& keyframe, std::int64_t now_ns,
                               const common::Pose2D& goal) const
{
    const double age_s = std::max(0.0, static_cast<double>(now_ns - keyframe.stamp_ns) * 1e-9);
    const double halflife = std::max(1e-3, options_.recency_halflife_s);
    const double recency = std::exp(-age_s * std::log(2.0) / halflife);
    const double novelty = std::clamp(static_cast<double>(keyframe.novelty), 0.0, 1.0);
    const double distance = common::Distance(keyframe.pose, goal);
    const double goal_term = 1.0 / (1.0 + distance);
    return options_.recency_weight * recency + options_.novelty_weight * novelty +
           options_.goal_weight * goal_term;
}

}  // namespace memory
}  // namespace navigation
