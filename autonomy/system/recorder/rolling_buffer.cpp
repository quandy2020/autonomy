/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file rolling_buffer.cpp
 * @brief L1 rolling window: push, prune, Snapshot copy, TakeAll move-out.
 */

#include "autonomy/system/recorder/rolling_buffer.hpp"

namespace autonomy {
namespace system {
namespace recorder {

RollingBuffer::RollingBuffer(int64_t window_ns)
    : window_ns_(window_ns > 0 ? window_ns : 30000000000LL) {}

void RollingBuffer::Push(Sample sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int64_t now = sample.clock.monotonic_ns();
    samples_.push_back(std::move(sample));
    PruneLocked(now);
}

std::vector<Sample> RollingBuffer::Snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {samples_.begin(), samples_.end()};
}

std::vector<Sample> RollingBuffer::TakeAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Sample> out;
    out.reserve(samples_.size());
    for (auto& sample : samples_) {
        out.push_back(std::move(sample));
    }
    samples_.clear();
    return out;
}

void RollingBuffer::DropThrough(int64_t monotonic_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!samples_.empty() &&
           samples_.front().clock.monotonic_ns() <= monotonic_ns) {
        samples_.pop_front();
    }
}

void RollingBuffer::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
}

size_t RollingBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

void RollingBuffer::PruneLocked(int64_t now_ns) {
    const int64_t cutoff = now_ns - window_ns_;
    while (!samples_.empty() &&
           samples_.front().clock.monotonic_ns() < cutoff) {
        samples_.pop_front();
    }
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
