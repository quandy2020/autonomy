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

#include "navigation/planning/plugins/memory/frame_queue_memory.hpp"

#include <algorithm>
#include <memory>

namespace navigation
{
namespace planning
{

FrameQueueMemory::FrameQueueMemory(plugins::FrameQueueMemoryOptions options)
    : capacity_(options.capacity() == 0 ? 16 : options.capacity())
{
}

common::Status FrameQueueMemory::Insert(const memory::Keyframe& keyframe)
{
    memory::Keyframe stored = keyframe;
    stored.epoch = epoch_;
    frames_.push_back(stored);
    while (frames_.size() > capacity_)
    {
        frames_.pop_front();
    }
    return common::Status{};
}

memory::ContextView FrameQueueMemory::View(const common::Pose2D& pose) const
{
    (void)pose;
    auto batch = std::make_shared<memory::KeyframeBatch>();
    batch->epoch = epoch_;
    const std::size_t count = std::min(frames_.size(), memory::kMaxContextEntries);
    for (std::size_t i = 0; i < count; ++i)
    {
        batch->entries[i] = frames_[frames_.size() - count + i];
    }
    batch->size = count;
    return memory::ContextView(std::move(batch));
}

std::size_t FrameQueueMemory::Size() const
{
    return frames_.size();
}

void FrameQueueMemory::Reset(bool keep_pose_index)
{
    frames_.clear();
    if (!keep_pose_index)
    {
        ++epoch_;
    }
}

ConsistencyReport FrameQueueMemory::CheckConsistency(const common::Pose2D& pose,
                                                     std::uint32_t epoch) const
{
    (void)pose;
    ConsistencyReport report;
    report.set_compatible(epoch == epoch_);
    report.set_code(epoch == epoch_ ? CONSISTENCY_CODE_OK
                                    : CONSISTENCY_CODE_EPOCH_MISMATCH);
    report.set_epoch(epoch_);
    return report;
}

}  // namespace planning
}  // namespace navigation
