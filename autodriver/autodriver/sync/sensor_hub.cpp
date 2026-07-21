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
 * @brief Implements SensorHub.
 */

#include "autodriver/sync/sensor_hub.hpp"

#include <chrono>
#include <utility>

namespace autodriver {

SensorHub::SensorHub()
: SensorHub(SensorHubOptions{})
{
}

SensorHub::SensorHub(SensorHubOptions options)
: options_(options)
{
}

SensorHub::~SensorHub()
{
  Stop();
}

void SensorHub::RegisterDriver(std::shared_ptr<SensorDriver> driver)
{
  if (!driver) {
    return;
  }

  driver->SetSampleCallback(
    [this](std::unique_ptr<SensorSample> sample) {
      OnSample(std::move(sample));
    });

  std::lock_guard<std::mutex> lock(drivers_mutex_);
  drivers_.push_back(std::move(driver));
}

void SensorHub::SetAlignedCallback(AlignedCallback callback)
{
  aligned_callback_ = std::move(callback);
}

void SensorHub::SetRawSampleCallback(RawSampleCallback callback)
{
  raw_sample_callback_ = std::move(callback);
}

bool SensorHub::Start()
{
  if (running_.exchange(true)) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(drivers_mutex_);
    for (const auto & driver : drivers_) {
      if (!driver->Start()) {
        running_ = false;
        for (const auto & started : drivers_) {
          if (started->IsRunning()) {
            started->Stop();
          }
        }
        return false;
      }
    }
  }

  alignment_thread_ = std::thread([this]() { AlignmentLoop(); });
  return true;
}

void SensorHub::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(drivers_mutex_);
    for (const auto & driver : drivers_) {
      driver->Stop();
    }
  }

  if (alignment_thread_.joinable()) {
    alignment_thread_.join();
  }
}

bool SensorHub::IsRunning() const
{
  return running_.load();
}

void SensorHub::OnSample(std::unique_ptr<SensorSample> sample)
{
  if (!sample) {
    return;
  }

  const Timestamp host_receive_time = Now();
  time_sync_.Update(sample->sensor_id(), sample->device_time(), host_receive_time);
  sample->set_host_time(
    time_sync_.ToHostTime(sample->sensor_id(), sample->device_time()));

  if (raw_sample_callback_) {
    raw_sample_callback_(*sample);
  }

  std::lock_guard<std::mutex> lock(buffers_mutex_);
  auto [it, inserted] = buffers_.try_emplace(
    sample->type(), options_.buffer_capacity);
  it->second.Push(std::move(sample));
}

void SensorHub::AlignmentLoop()
{
  while (running_.load()) {
    const Timestamp reference_time = Now();
    AlignedSnapshot snapshot = BuildSnapshot(reference_time);

    if (aligned_callback_ && !snapshot.samples.empty()) {
      aligned_callback_(snapshot);
    }

    std::this_thread::sleep_for(options_.publish_period);
  }
}

AlignedSnapshot SensorHub::BuildSnapshot(Timestamp reference_time) const
{
  AlignedSnapshot snapshot;
  snapshot.reference_time = reference_time;

  static constexpr SensorType kAllTypes[] = {
    SensorType::kImu,
    SensorType::kGps,
    SensorType::kWheelOdometry,
    SensorType::kCamera,
    SensorType::kLidar,
    SensorType::kRangeFinder,
  };

  std::lock_guard<std::mutex> lock(buffers_mutex_);
  for (SensorType type : kAllTypes) {
    const auto buffer_it = buffers_.find(type);
    if (buffer_it == buffers_.end()) {
      continue;
    }

    std::unique_ptr<SensorSample> sample =
      buffer_it->second.LatestAtOrBefore(reference_time);
    if (!sample) {
      continue;
    }

    const int64_t skew_ns = ToNanoseconds(reference_time) -
      ToNanoseconds(sample->host_time());
    if (skew_ns < 0 ||
      skew_ns > options_.alignment_window.count())
    {
      continue;
    }

    snapshot.samples[type] = std::move(sample);
  }

  return snapshot;
}

}  // namespace autodriver
