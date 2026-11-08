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

#include "navigation/memory/keyframe_filter.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace memory
{
namespace
{

double Cosine(const float* lhs, const float* rhs, std::size_t size)
{
    double dot = 0.0;
    double lhs_norm = 0.0;
    double rhs_norm = 0.0;
    for (std::size_t i = 0; i < size; ++i)
    {
        dot += static_cast<double>(lhs[i]) * rhs[i];
        lhs_norm += static_cast<double>(lhs[i]) * lhs[i];
        rhs_norm += static_cast<double>(rhs[i]) * rhs[i];
    }
    const double denominator = std::sqrt(lhs_norm * rhs_norm);
    return denominator > 1e-12 ? dot / denominator : 0.0;
}

}  // namespace

KeyframeFilter::KeyframeFilter(KeyframeFilterOptions options) : options_(options)
{
    if (options_.max_entries == 0)
    {
        options_.max_entries = 1;
    }
}

bool KeyframeFilter::TryAccept(const Keyframe& keyframe, bool* accepted)
{
    if (accepted == nullptr)
    {
        return false;
    }
    *accepted = false;
    if (keyframe.embedding_dim == 0)
    {
        return false;
    }
    const double stamp_s = static_cast<double>(keyframe.stamp_ns) * 1e-9;
    if (!entries_.empty())
    {
        const Keyframe& last = entries_.back();
        if (stamp_s - last_accept_s_ < options_.min_interval_s)
        {
            return true;
        }
        const double dx = keyframe.pose.x() - last.pose.x();
        const double dy = keyframe.pose.y() - last.pose.y();
        const double move = std::sqrt(dx * dx + dy * dy);
        const double turn = std::abs(keyframe.pose.theta() - last.pose.theta());
        if (move < options_.min_motion && turn < options_.min_motion_rad)
        {
            return true;
        }
    }

    const double novelty = Novelty(keyframe);
    if (novelty < options_.novelty_threshold_eta)
    {
        return true;
    }

    if (entries_.size() >= options_.max_entries)
    {
        const auto victim = std::min_element(
            entries_.begin(), entries_.end(), [](const Keyframe& lhs, const Keyframe& rhs) {
                return lhs.importance < rhs.importance;
            });
        if (victim == entries_.end() || keyframe.importance < victim->importance)
        {
            return true;
        }
        entries_.erase(victim);
    }

    Keyframe stored = keyframe;
    stored.novelty = static_cast<float>(novelty);
    entries_.push_back(stored);
    last_accept_s_ = stamp_s;
    *accepted = true;
    return true;
}

const std::vector<Keyframe>& KeyframeFilter::Entries() const
{
    return entries_;
}

std::size_t KeyframeFilter::Size() const
{
    return entries_.size();
}

void KeyframeFilter::Reset()
{
    entries_.clear();
    last_accept_s_ = -1e9;
}

double KeyframeFilter::Novelty(const Keyframe& keyframe) const
{
    double best_similarity = -1.0;
    for (const auto& entry : entries_)
    {
        if (entry.embedding_dim != keyframe.embedding_dim)
        {
            continue;
        }
        const double dx = entry.pose.x() - keyframe.pose.x();
        const double dy = entry.pose.y() - keyframe.pose.y();
        if (dx * dx + dy * dy > options_.search_radius * options_.search_radius)
        {
            continue;
        }
        const double similarity =
            Cosine(entry.embedding.data(), keyframe.embedding.data(), keyframe.embedding_dim);
        best_similarity = std::max(best_similarity, similarity);
    }
    return best_similarity < -0.5 ? 1.0 : 1.0 - best_similarity;
}

}  // namespace memory
}  // namespace navigation
