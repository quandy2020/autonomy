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

#include "navigation/planning/plugins/memory/single_frame_memory.hpp"

#include <memory>

namespace navigation
{
namespace planning
{

common::Status SingleFrameMemory::Insert(const memory::Keyframe& keyframe)
{
    frame_ = keyframe;
    frame_.epoch = epoch_;
    has_frame_ = true;
    return common::Status{};
}

memory::ContextView SingleFrameMemory::View(const common::Pose2D& pose) const
{
    (void)pose;
    auto batch = std::make_shared<memory::KeyframeBatch>();
    batch->epoch = epoch_;
    if (has_frame_)
    {
        batch->entries[0] = frame_;
        batch->size = 1;
    }
    return memory::ContextView(std::move(batch));
}

std::size_t SingleFrameMemory::Size() const
{
    return has_frame_ ? 1 : 0;
}

void SingleFrameMemory::Reset(bool keep_pose_index)
{
    has_frame_ = false;
    if (!keep_pose_index)
    {
        ++epoch_;
    }
}

ConsistencyReport SingleFrameMemory::CheckConsistency(const common::Pose2D& pose,
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
