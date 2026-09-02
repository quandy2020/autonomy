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
 * @brief Fixed-capacity per-sensor sample ring buffer.
 */

#ifndef AUTODRIVER_SAMPLE_BUFFER_HPP_
#define AUTODRIVER_SAMPLE_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <memory>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {

/**
 * @class autodriver::SampleBuffer
 * @brief Time-ordered deque of samples; not internally synchronized.
 *
 * SensorHub holds buffers_lock_ around all access.
 */
class SampleBuffer {
public:
    /**
     * @brief Construct a ring buffer with a fixed capacity.
     * @param capacity Maximum number of samples to retain.
     */
    explicit SampleBuffer(std::size_t capacity = 32);

    /**
     * @brief Append a sample; drops the oldest entry when at capacity.
     * @param sample Shared sample to store in time order.
     */
    void Push(std::shared_ptr<SensorSample> sample);

    /**
     * @brief Return the newest sample at or before the given host time.
     * @param time Upper bound on sample host time.
     * @return Matching sample, or nullptr when the buffer is empty.
     */
    std::shared_ptr<SensorSample> LatestAtOrBefore(
        const autolink::Time& time) const;

    /**
     * @brief Return the most recently pushed sample.
     * @return Latest sample, or nullptr when the buffer is empty.
     */
    std::shared_ptr<SensorSample> Latest() const;

    /** @brief Remove all buffered samples. */
    void Clear();

    /**
     * @brief Number of samples currently stored.
     * @return Current buffer size.
     */
    std::size_t Size() const;

private:
    /** @brief Maximum samples retained before dropping the oldest. */
    std::size_t capacity_;
    /** @brief Time-ordered deque of shared samples. */
    std::deque<std::shared_ptr<SensorSample>> samples_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SAMPLE_BUFFER_HPP_
