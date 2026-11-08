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

#include "navigation/memory/loop_aligner.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace memory
{
namespace
{

double Cosine(const Keyframe& lhs, const Keyframe& rhs)
{
    const std::size_t dim = std::min(lhs.embedding_dim, rhs.embedding_dim);
    double dot = 0.0;
    double lhs_norm = 0.0;
    double rhs_norm = 0.0;
    for (std::size_t i = 0; i < dim; ++i)
    {
        dot += static_cast<double>(lhs.embedding[i]) * rhs.embedding[i];
        lhs_norm += static_cast<double>(lhs.embedding[i]) * lhs.embedding[i];
        rhs_norm += static_cast<double>(rhs.embedding[i]) * rhs.embedding[i];
    }
    const double denominator = std::sqrt(lhs_norm * rhs_norm);
    return denominator > 1e-12 ? dot / denominator : 0.0;
}

}  // namespace

LoopAligner::LoopAligner(double similarity_threshold, double max_distance)
    : similarity_threshold_(std::clamp(similarity_threshold, -1.0, 1.0)),
      max_distance_(std::max(0.0, max_distance))
{
}

std::optional<LoopCandidate> LoopAligner::Find(
    const Keyframe& query, const std::vector<Keyframe>& memory) const
{
    std::optional<LoopCandidate> best;
    for (const auto& candidate : memory)
    {
        if (candidate.id == query.id || candidate.embedding_dim == 0 ||
            common::Distance(candidate.pose, query.pose) > max_distance_)
        {
            continue;
        }
        const double similarity = Cosine(query, candidate);
        if (similarity < similarity_threshold_)
        {
            continue;
        }
        if (!best.has_value() || similarity > best->similarity)
        {
            LoopCandidate match;
            match.id = candidate.id;
            match.similarity = similarity;
            match.correction =
                common::Compose(common::Inverse(query.pose), candidate.pose);
            best = match;
        }
    }
    return best;
}

}  // namespace memory
}  // namespace navigation
