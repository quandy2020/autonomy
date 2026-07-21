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
 * @brief Implements SampleBuffer.
 */

#include "autodriver/sync/sample_buffer.hpp"

namespace autodriver {

SampleBuffer::SampleBuffer(std::size_t capacity)
: capacity_(capacity > 0 ? capacity : 1)
{
}

void SampleBuffer::Push(std::unique_ptr<SensorSample> sample)
{
  if (!sample) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  samples_.push_back(std::move(sample));
  while (samples_.size() > capacity_) {
    samples_.pop_front();
  }
}

std::unique_ptr<SensorSample> SampleBuffer::LatestAtOrBefore(
  Timestamp reference_time) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
    if ((*it)->host_time() <= reference_time) {
      return (*it)->Clone();
    }
  }
  return nullptr;
}

std::unique_ptr<SensorSample> SampleBuffer::Latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (samples_.empty()) {
    return nullptr;
  }
  return samples_.back()->Clone();
}

void SampleBuffer::Clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

std::size_t SampleBuffer::Size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

}  // namespace autodriver
