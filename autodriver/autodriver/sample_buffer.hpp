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

#ifndef AUTODRIVER_SAMPLE_BUFFER_HPP_
#define AUTODRIVER_SAMPLE_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <memory>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {

// Not internally synchronized. SensorHub holds buffers_lock_ around access.
class SampleBuffer {
public:
    explicit SampleBuffer(std::size_t capacity = 32);

    void Push(std::shared_ptr<SensorSample> sample);
    std::shared_ptr<SensorSample> LatestAtOrBefore(
        const autolink::Time& time) const;
    std::shared_ptr<SensorSample> Latest() const;
    void Clear();
    std::size_t Size() const;

private:
    std::size_t capacity_;
    std::deque<std::shared_ptr<SensorSample>> samples_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SAMPLE_BUFFER_HPP_
