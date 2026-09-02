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
 * @brief SensorHub implementation: buffering, time sync, and alignment loop.
 */

#include "autodriver/sensor_hub.hpp"

#include <utility>

#include "autolink/time/rate.hpp"

namespace autodriver {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using ReadLock = autolink::base::ReadLockGuard<AtomicRWLock>;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

}  // namespace

/**
 * @brief Default-constructs a hub with default alignment options.
 */
SensorHub::SensorHub() : SensorHub(Options{}) {}

/**
 * @brief Constructs a hub with alignment and buffer options.
 */
SensorHub::SensorHub(Options options) : options_(options) {}

/**
 * @brief Stops the hub if still running.
 */
SensorHub::~SensorHub() { Stop(); }

/**
 * @brief Registers a driver and wires its samples into this hub.
 */
void SensorHub::RegisterDriver(std::shared_ptr<SensorDriver> driver) {
    if (!driver) {
        return;
    }
    driver->SetSampleCallback([this](std::unique_ptr<SensorSample> sample) {
        OnSample(std::shared_ptr<SensorSample>(std::move(sample)));
    });
    WriteLock lock(drivers_lock_);
    drivers_.push_back(std::move(driver));
}

/**
 * @brief Sets the callback invoked with aligned multi-sensor snapshots.
 */
void SensorHub::SetAlignedCallback(AlignedCallback callback) {
    aligned_.DisconnectAllSlots();
    if (callback) {
        aligned_.Connect(callback);
    }
}

/**
 * @brief Sets the callback invoked for every raw sample after time sync.
 */
void SensorHub::SetRawSampleCallback(RawSampleCallback callback) {
    raw_sample_.DisconnectAllSlots();
    if (callback) {
        raw_sample_.Connect(callback);
    }
}

/**
 * @brief Starts all registered drivers and the alignment publish thread.
 */
bool SensorHub::Start() {
    if (running_.exchange(true)) {
        return true;
    }
    {
        WriteLock lock(drivers_lock_);
        for (const auto& driver : drivers_) {
            if (!driver->Start()) {
                running_ = false;
                for (const auto& started : drivers_) {
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

/**
 * @brief Stops drivers, the alignment thread, and clears the running flag.
 */
void SensorHub::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    {
        WriteLock lock(drivers_lock_);
        for (const auto& driver : drivers_) {
            driver->Stop();
        }
    }
    if (alignment_thread_.joinable()) {
        alignment_thread_.join();
    }
}

/**
 * @brief Returns true while the hub and alignment loop are active.
 */
bool SensorHub::IsRunning() const { return running_.load(); }

/**
 * @brief Ingests an externally produced sample into buffering and callbacks.
 */
void SensorHub::PushSample(std::shared_ptr<SensorSample> sample) {
    OnSample(std::move(sample));
}

/**
 * @brief Removes the per-sensor buffer when a driver detaches.
 */
void SensorHub::DropBuffer(const SensorId& id) {
    WriteLock lock(buffers_lock_);
    buffers_.erase(id);
}

/**
 * @brief Time-syncs, stores, and forwards a sample to raw callbacks.
 */
void SensorHub::OnSample(std::shared_ptr<SensorSample> sample) {
    if (!sample) {
        return;
    }
    const autolink::Time now = autolink::Time::Now();
    sample->set_host_time(
        time_sync_.Observe(sample->id(), sample->device_time(), now));
    raw_sample_(*sample);

    WriteLock lock(buffers_lock_);
    auto& buffer = buffers_[sample->id()];
    if (!buffer) {
        buffer = std::make_unique<SampleBuffer>(options_.buffer_capacity);
    }
    buffer->Push(std::move(sample));
    seq_.fetch_add(1, std::memory_order_relaxed);
}

/**
 * @brief Periodically publishes aligned snapshots when new samples arrive.
 */
void SensorHub::AlignmentLoop() {
    autolink::Rate rate(options_.publish_period);

    // Sequence counter value at the last published snapshot.
    std::uint64_t published = 0;
    while (running_.load()) {
        const std::uint64_t seq = seq_.load(std::memory_order_relaxed);
        if (seq != published) {
            published = seq;
            AlignedSnapshot snapshot = BuildSnapshot(autolink::Time::Now());
            if (!snapshot.samples.empty()) {
                aligned_(snapshot);
            }
        }
        rate.Sleep();
    }
}

/**
 * @brief Builds a time-aligned snapshot from the latest in-window samples.
 */
AlignedSnapshot SensorHub::BuildSnapshot(const autolink::Time& time) const {
    AlignedSnapshot snapshot;
    snapshot.time = time;

    ReadLock lock(buffers_lock_);
    for (const auto& entry : buffers_) {
        if (!entry.second) {
            continue;
        }
        std::shared_ptr<SensorSample> sample =
            entry.second->LatestAtOrBefore(time);
        if (!sample) {
            continue;
        }
        const autolink::Duration skew = time - sample->host_time();
        if (skew < autolink::Duration(0) || skew > options_.alignment_window) {
            continue;
        }
        snapshot.samples[entry.first] = std::move(sample);
    }
    return snapshot;
}

}  // namespace autodriver
