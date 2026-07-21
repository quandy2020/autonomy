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
 * @brief Central hub: driver registration, time sync, and aligned output.
 */

#ifndef AUTODRIVER_SYNC_SENSOR_HUB_HPP_
#define AUTODRIVER_SYNC_SENSOR_HUB_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autodriver/common/time.hpp"
#include "autodriver/hal/sensor_driver.hpp"
#include "autodriver/sync/aligned_snapshot.hpp"
#include "autodriver/sync/sample_buffer.hpp"
#include "autodriver/sync/time_sync.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @struct SensorHubOptions
 * @brief Configuration for multi-sensor alignment and publish rate.
 */
struct SensorHubOptions
{
  /** Max host-time skew when picking samples for one snapshot. */
  Duration alignment_window{std::chrono::milliseconds(50)};
  /** Period between aligned snapshot callbacks. */
  Duration publish_period{std::chrono::milliseconds(20)};
  /** Per-sensor ring buffer capacity. */
  std::size_t buffer_capacity{32};
};

/**
 * @class SensorHub
 * @brief Registers drivers, synchronizes timestamps, and emits aligned frames.
 *
 * Typical usage:
 * 1. RegisterDriver() for each sensor backend.
 * 2. SetAlignedCallback() for perception/localization consumers.
 * 3. Start() / Stop() to control acquisition and alignment thread.
 */
class SensorHub
{
public:
  using AlignedCallback = std::function<void(const AlignedSnapshot & snapshot)>;
  using RawSampleCallback = std::function<void(const SensorSample & sample)>;

  SensorHub();
  explicit SensorHub(SensorHubOptions options);
  ~SensorHub();

  SensorHub(const SensorHub &) = delete;
  SensorHub & operator=(const SensorHub &) = delete;

  /** @brief Attaches a sensor driver; must be called before Start(). */
  void RegisterDriver(std::shared_ptr<SensorDriver> driver);

  /** @brief Callback invoked for each time-aligned multi-sensor snapshot. */
  void SetAlignedCallback(AlignedCallback callback);

  /** @brief Optional callback for every individual sample (post sync). */
  void SetRawSampleCallback(RawSampleCallback callback);

  /** @brief Starts all registered drivers and the alignment thread. */
  bool Start();

  /** @brief Stops drivers and the alignment thread. */
  void Stop();

  /** @brief Returns true while drivers and alignment are active. */
  bool IsRunning() const;

  /** @brief Access to the shared time synchronizer (read-only use). */
  const TimeSync & time_sync() const { return time_sync_; }

private:
  void OnSample(std::unique_ptr<SensorSample> sample);
  void AlignmentLoop();
  AlignedSnapshot BuildSnapshot(Timestamp reference_time) const;

  SensorHubOptions options_;
  TimeSync time_sync_;
  mutable std::mutex drivers_mutex_;
  std::vector<std::shared_ptr<SensorDriver>> drivers_;
  mutable std::mutex buffers_mutex_;
  std::unordered_map<SensorType, SampleBuffer> buffers_;
  AlignedCallback aligned_callback_;
  RawSampleCallback raw_sample_callback_;
  std::atomic<bool> running_{false};
  std::thread alignment_thread_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SYNC_SENSOR_HUB_HPP_
