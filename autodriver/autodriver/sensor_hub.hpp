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
 * @brief Buffers samples, estimates per-sensor clock offsets, and emits aligned snapshots.
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

/**
 * @class autodriver::SensorHub
 * @brief Central sample router with optional multi-sensor time alignment.
 */
class SensorHub {
public:
    /**
     * @brief Tuning knobs for ring buffers and the alignment publisher loop.
     */
    struct Options {
        // Maximum host-time span for samples in one aligned snapshot.
        autolink::Duration alignment_window{50'000'000};

        // Period between aligned snapshot publications.
        autolink::Duration publish_period{20'000'000};

        // Maximum samples retained per sensor ring buffer.
        std::size_t buffer_capacity{32};
    };

    // Callback invoked when a multi-sensor aligned snapshot is ready.
    using AlignedCallback =
        autolink::base::Signal<const AlignedSnapshot&>::Callback;

    // Callback invoked for each raw sample before alignment.
    using RawSampleCallback =
        autolink::base::Signal<const SensorSample&>::Callback;

    /**
     * @brief Construct with default Options.
     */
    SensorHub();
    /**
     * @brief Construct with explicit buffer and alignment tuning.
     * @param options Hub configuration for buffers and the alignment loop.
     */
    explicit SensorHub(Options options);
    /**
     * @brief Stop the alignment thread and release resources.
     */
    ~SensorHub();

    /**
     * @brief Copy construction is disabled.
     */
    SensorHub(const SensorHub&) = delete;
    /**
     * @brief Copy assignment is disabled.
     */
    SensorHub& operator=(const SensorHub&) = delete;

    /**
     * @brief Register a driver whose samples will be buffered on PushSample().
     * @param driver Shared pointer to a sensor driver instance.
     */
    void RegisterDriver(std::shared_ptr<SensorDriver> driver);

    /**
     * @brief Register a callback for aligned multi-sensor snapshots.
     * @param callback Invoked on the alignment thread for each snapshot.
     */
    void SetAlignedCallback(AlignedCallback callback);

    /**
     * @brief Register a callback for every raw ingested sample.
     * @param callback Invoked synchronously from PushSample().
     */
    void SetRawSampleCallback(RawSampleCallback callback);

    /**
     * @brief Ingest a sample, update time sync, and fan out to raw listeners.
     * @param sample Shared sample to buffer and broadcast.
     */
    void PushSample(std::shared_ptr<SensorSample> sample);

    /**
     * @brief Drop buffered history for a detached sensor.
     * @param id Sensor identifier whose buffer should be cleared.
     */
    void DropBuffer(const SensorId& id);

    /**
     * @brief Start the background alignment publisher loop.
     * @return True when the hub starts successfully.
     */
    bool Start();

    /**
     * @brief Stop the alignment publisher loop.
     */
    void Stop();

    /**
     * @brief Whether the alignment publisher loop is active.
     * @return True after Start() and before Stop().
     */
    bool IsRunning() const;

    /**
     * @brief Access the per-sensor clock offset estimator.
     * @return Const reference to the owned TimeSync instance.
     */
    const TimeSync& time_sync() const { return time_sync_; }

private:
    /**
     * @brief Internal handler that buffers a sample and notifies raw listeners.
     * @param sample Shared sample received from a driver callback.
     */
    void OnSample(std::shared_ptr<SensorSample> sample);

    /**
     * @brief Background loop that publishes aligned snapshots periodically.
     */
    void AlignmentLoop();

    /**
     * @brief Build a snapshot from the latest samples near a host timestamp.
     * @param time Target host time for sample selection.
     * @return Aligned snapshot containing one sample per sensor when available.
     */
    AlignedSnapshot BuildSnapshot(const autolink::Time& time) const;

    // Hub tuning for buffers and alignment publishing.
    Options options_;

    // Estimates device-to-host clock offsets per sensor.
    TimeSync time_sync_;

    // Protects the registered drivers vector.
    mutable autolink::base::AtomicRWLock drivers_lock_;

    // Registered sensor drivers observed by the hub.
    std::vector<std::shared_ptr<SensorDriver>> drivers_;

    // Protects per-sensor sample buffers.
    mutable autolink::base::AtomicRWLock buffers_lock_;

    // Ring buffers keyed by sensor id.
    std::unordered_map<SensorId, std::unique_ptr<SampleBuffer>> buffers_;

    // Signal for aligned snapshot subscribers.
    autolink::base::Signal<const AlignedSnapshot&> aligned_;

    // Signal for raw sample subscribers.
    autolink::base::Signal<const SensorSample&> raw_sample_;

    // True while the alignment thread is running.
    std::atomic<bool> running_{false};

    // Monotonic sequence number assigned to each aligned snapshot.
    std::atomic<std::uint64_t> seq_{0};

    // Thread that periodically builds and publishes aligned snapshots.
    std::thread alignment_thread_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_HUB_HPP_
