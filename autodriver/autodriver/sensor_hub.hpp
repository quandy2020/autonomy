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

#ifndef AUTODRIVER_SENSOR_HUB_HPP_
#define AUTODRIVER_SENSOR_HUB_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autodriver/sample_buffer.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autodriver/aligned_snapshot.hpp"
#include "autodriver/time_synchronization.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/base/rw_lock_guard.hpp"
#include "autolink/base/signal.hpp"
#include "autolink/time/duration.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {

class SensorHub {
public:
    struct Options {
        autolink::Duration alignment_window{50'000'000};
        autolink::Duration publish_period{20'000'000};
        std::size_t buffer_capacity{32};
    };

    using AlignedCallback =
        autolink::base::Signal<const AlignedSnapshot&>::Callback;
    using RawSampleCallback =
        autolink::base::Signal<const SensorSample&>::Callback;

    SensorHub();
    explicit SensorHub(Options options);
    ~SensorHub();

    SensorHub(const SensorHub&) = delete;
    SensorHub& operator=(const SensorHub&) = delete;

    void RegisterDriver(std::shared_ptr<SensorDriver> driver);
    void SetAlignedCallback(AlignedCallback callback);
    void SetRawSampleCallback(RawSampleCallback callback);
    void PushSample(std::shared_ptr<SensorSample> sample);
    void DropBuffer(const SensorId& id);
    bool Start();
    void Stop();
    bool IsRunning() const;
    const TimeSync& time_sync() const { return time_sync_; }

private:
    void OnSample(std::shared_ptr<SensorSample> sample);
    void AlignmentLoop();
    AlignedSnapshot BuildSnapshot(const autolink::Time& time) const;

    Options options_;
    TimeSync time_sync_;
    mutable autolink::base::AtomicRWLock drivers_lock_;
    std::vector<std::shared_ptr<SensorDriver>> drivers_;
    mutable autolink::base::AtomicRWLock buffers_lock_;
    std::unordered_map<SensorId, std::unique_ptr<SampleBuffer>> buffers_;
    autolink::base::Signal<const AlignedSnapshot&> aligned_;
    autolink::base::Signal<const SensorSample&> raw_sample_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> seq_{0};
    std::thread alignment_thread_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_HUB_HPP_
