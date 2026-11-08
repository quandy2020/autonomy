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

#include "navigation/planning/plugins/memory/pose_indexed_memory.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

namespace navigation
{
namespace planning
{
namespace
{

memory::KeyframeFilterOptions ToFilterOptions(
    const plugins::PoseIndexedMemoryOptions& options)
{
    memory::KeyframeFilterOptions filter;
    filter.max_entries = options.max_entries() == 0 ? 5000 : options.max_entries();
    filter.novelty_threshold_eta =
        options.novelty_threshold_eta() > 0.0 ? options.novelty_threshold_eta() : 0.08;
    filter.min_interval_s =
        options.min_insert_interval_s() > 0.0 ? options.min_insert_interval_s() : 0.2;
    filter.search_radius = options.query_radius() > 0.0 ? options.query_radius() : 2.0;
    return filter;
}

}  // namespace

PoseIndexedMemory::PoseIndexedMemory(plugins::PoseIndexedMemoryOptions options)
    : filter_(ToFilterOptions(options))
{
}

common::Status PoseIndexedMemory::Insert(const memory::Keyframe& keyframe)
{
    memory::Keyframe stored = keyframe;
    stored.epoch = epoch_;
    bool accepted = false;
    if (!filter_.TryAccept(stored, &accepted))
    {
        return common::Status::InvalidArgument("invalid keyframe");
    }
    return common::Status{};
}

memory::ContextView PoseIndexedMemory::View(const common::Pose2D& pose) const
{
    auto batch = std::make_shared<memory::KeyframeBatch>();
    batch->epoch = epoch_;
    const auto& entries = filter_.Entries();
    std::vector<std::size_t> indices(entries.size());
    std::iota(indices.begin(), indices.end(), 0U);
    std::sort(indices.begin(), indices.end(), [&entries, &pose](std::size_t lhs,
                                                                std::size_t rhs) {
        const auto distance = [&pose, &entries](std::size_t index) {
            const double dx = entries[index].pose.x() - pose.x();
            const double dy = entries[index].pose.y() - pose.y();
            return dx * dx + dy * dy;
        };
        return distance(lhs) < distance(rhs);
    });

    const std::size_t count =
        std::min({indices.size(), memory::kMaxContextEntries, batch->entries.size()});
    for (std::size_t i = 0; i < count; ++i)
    {
        batch->entries[i] = entries[indices[i]];
    }
    batch->size = count;
    return memory::ContextView(std::move(batch));
}

std::size_t PoseIndexedMemory::Size() const
{
    return filter_.Size();
}

void PoseIndexedMemory::Reset(bool keep_pose_index)
{
    filter_.Reset();
    if (!keep_pose_index)
    {
        ++epoch_;
    }
}

ConsistencyReport PoseIndexedMemory::CheckConsistency(const common::Pose2D& pose,
                                                     std::uint32_t epoch) const
{
    (void)pose;
    ConsistencyReport report;
    if (epoch != epoch_)
    {
        report.set_compatible(false);
        report.set_code(CONSISTENCY_CODE_EPOCH_MISMATCH);
        report.set_epoch(epoch_);
        report.set_reason("epoch mismatch");
        return report;
    }
    report.set_compatible(true);
    report.set_code(CONSISTENCY_CODE_OK);
    report.set_epoch(epoch_);
    return report;
}

}  // namespace planning
}  // namespace navigation
