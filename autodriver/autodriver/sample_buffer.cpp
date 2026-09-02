/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief SampleBuffer ring buffer implementation.
 */

#include "autodriver/sample_buffer.hpp"

namespace autodriver {

/**
 * @brief Constructs a ring buffer with at least one slot of capacity.
 */
SampleBuffer::SampleBuffer(std::size_t capacity)
    : capacity_(capacity > 0 ? capacity : 1) {}

/**
 * @brief Appends a sample and evicts the oldest when over capacity.
 */
void SampleBuffer::Push(std::shared_ptr<SensorSample> sample) {
    if (!sample) {
        return;
    }
    samples_.push_back(std::move(sample));
    while (samples_.size() > capacity_) {
        samples_.pop_front();
    }
}

/**
 * @brief Returns the newest sample whose host time is at or before time.
 */
std::shared_ptr<SensorSample> SampleBuffer::LatestAtOrBefore(
    const autolink::Time& time) const {
    for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
        if ((*it)->host_time() <= time) {
            return *it;
        }
    }
    return nullptr;
}

/**
 * @brief Returns the most recently pushed sample, or nullptr if empty.
 */
std::shared_ptr<SensorSample> SampleBuffer::Latest() const {
    if (samples_.empty()) {
        return nullptr;
    }
    return samples_.back();
}

/**
 * @brief Removes all buffered samples.
 */
void SampleBuffer::Clear() { samples_.clear(); }

/**
 * @brief Returns the number of samples currently stored.
 */
std::size_t SampleBuffer::Size() const { return samples_.size(); }

}  // namespace autodriver
