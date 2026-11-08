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

#include "navigation/planning/chunk_blender.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace planning
{

ChunkBlender::ChunkBlender(Options options) : options_(options)
{
    if (!std::isfinite(options_.action_dt_s) || options_.action_dt_s <= 0.0)
    {
        options_.action_dt_s = 0.1;
    }
    if (!std::isfinite(options_.blend_tau) || options_.blend_tau <= 0.0)
    {
        options_.blend_tau = 2.0;
    }
    options_.chunk_length = std::min(options_.chunk_length, kMaxChunkLength);
    action_dt_s_ = options_.action_dt_s;
}

bool ChunkBlender::Push(const ActionChunk& chunk, std::int64_t now_ns)
{
    const std::size_t count = static_cast<std::size_t>(chunk.actions_size());
    if (count == 0 || count > kMaxChunkLength)
    {
        return false;
    }
    if (chunk.dt_s() > 0.0 && std::isfinite(chunk.dt_s()))
    {
        action_dt_s_ = chunk.dt_s();
    }

    std::size_t start = static_cast<std::size_t>(std::ceil(position_));
    if (position_ <= 0.0 || start >= chunk_length_)
    {
        position_ = 0.0;
        start = 0;
        chunk_length_ = 0;
    }

    // Keep the remaining actions of the old chunk only when the new chunk
    // starts before the previous horizon ends.
    for (std::size_t index = 0; index < count && start + index < kMaxChunkLength; ++index)
    {
        const double weight = std::exp(-static_cast<double>(index) / options_.blend_tau);
        Slot& slot = slots_[start + index];
        const common::Twist2D incoming = navigation::common::MakeTwist2D(
            chunk.actions(static_cast<int>(index)).x(),
            chunk.actions(static_cast<int>(index)).y(),
            chunk.actions(static_cast<int>(index)).theta());
        if (slot.valid)
        {
            slot.action.set_x(weight * incoming.x() + (1.0 - weight) * slot.action.x());
            slot.action.set_y(weight * incoming.y() + (1.0 - weight) * slot.action.y());
            slot.action.set_theta(weight * incoming.theta() +
                (1.0 - weight) * slot.action.theta());
            slot.weight = weight;
        }
        else
        {
            slot.action = incoming;
            slot.weight = weight;
            slot.valid = true;
        }
    }

    chunk_length_ = std::min(kMaxChunkLength, start + count);
    last_push_ns_ = now_ns;
    return true;
}

bool ChunkBlender::Consume(double dt_s, common::Twist2D* command)
{
    if (command == nullptr || chunk_length_ == 0 || !std::isfinite(dt_s) || dt_s <= 0.0)
    {
        return false;
    }
    if (position_ >= static_cast<double>(chunk_length_) - 1.0)
    {
        return false;
    }

    const std::size_t index = static_cast<std::size_t>(position_);
    const double fraction = position_ - static_cast<double>(index);
    const Slot& current = slots_[index];
    if (!current.valid)
    {
        return false;
    }
    const Slot& next = slots_[std::min(index + 1, kMaxChunkLength - 1)];
    common::Twist2D output = current.action;
    if (next.valid)
    {
        output.set_x((current.action.x() + (next.action.x() - current.action.x()) * fraction));
        output.set_y((current.action.y() + (next.action.y() - current.action.y()) * fraction));
        output.set_theta((current.action.theta() +
             (next.action.theta() - current.action.theta()) * fraction));
    }

    if (has_last_command_ && options_.max_acceleration_mps2 > 0.0)
    {
        const double max_delta = options_.max_acceleration_mps2 * dt_s;
        output.set_x(last_command_.x() + autonomy::common::Clamp(static_cast<double>(output.x()) -
                                                           last_command_.x(),
                                                       -max_delta, max_delta));
        output.set_y(last_command_.y() + autonomy::common::Clamp(static_cast<double>(output.y()) -
                                                           last_command_.y(),
                                                       -max_delta, max_delta));
        output.set_theta(last_command_.theta() +
            autonomy::common::Clamp(static_cast<double>(output.theta()) - last_command_.theta(),
                          -max_delta, max_delta));
    }

    *command = output;
    last_command_ = output;
    has_last_command_ = true;
    position_ += dt_s / action_dt_s_;
    return true;
}

void ChunkBlender::Reset()
{
    slots_.fill(Slot{});
    chunk_length_ = 0;
    position_ = 0.0;
    has_last_command_ = false;
    last_command_ = common::Twist2D{};
}

bool ChunkBlender::HasActions() const
{
    return chunk_length_ > 0 &&
           position_ < static_cast<double>(chunk_length_) - 1.0;
}

double ChunkBlender::Position() const
{
    return position_;
}

std::size_t ChunkBlender::ChunkLength() const
{
    return chunk_length_;
}

}  // namespace planning
}  // namespace navigation
