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
 * @file ring_buffer.cpp
 * @brief Mutex-protected ring with monotonic prune.
 */

#include "robot/data_agent/ring_buffer/ring_buffer.hpp"

#include <algorithm>

namespace autoloop {
namespace robot {

RingBuffer::RingBuffer(int64_t window_ns) : window_ns_(window_ns) {}

void RingBuffer::Push(Sample sample) {
    std::lock_guard<std::mutex> lock(mu_);
    const int64_t now = sample.clock.mono_ns();
    q_.push_back(std::move(sample));
    PruneLocked(now);
}

std::vector<Sample> RingBuffer::Snapshot() const {
    std::lock_guard<std::mutex> lock(mu_);
    return {q_.begin(), q_.end()};
}

std::vector<Sample> RingBuffer::TakeAll() {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<Sample> out;
    out.reserve(q_.size());
    while (!q_.empty()) {
        out.push_back(std::move(q_.front()));
        q_.pop_front();
    }
    return out;
}

size_t RingBuffer::size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return q_.size();
}

void RingBuffer::PruneLocked(int64_t now_ns) {
    while (!q_.empty() && window_ns_ > 0 &&
           (now_ns - q_.front().clock.mono_ns()) > window_ns_) {
        q_.pop_front();
    }
}

}  // namespace robot
}  // namespace autoloop
