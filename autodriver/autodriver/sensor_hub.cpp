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

#include "autodriver/sensor_hub.hpp"

#include <utility>

#include "autolink/time/rate.hpp"

namespace autodriver {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using ReadLock = autolink::base::ReadLockGuard<AtomicRWLock>;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

}  // namespace

SensorHub::SensorHub() : SensorHub(Options{}) {}

SensorHub::SensorHub(Options options) : options_(options) {}

SensorHub::~SensorHub() { Stop(); }

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

void SensorHub::SetAlignedCallback(AlignedCallback callback) {
    aligned_.DisconnectAllSlots();
    if (callback) {
        aligned_.Connect(callback);
    }
}

void SensorHub::SetRawSampleCallback(RawSampleCallback callback) {
    raw_sample_.DisconnectAllSlots();
    if (callback) {
        raw_sample_.Connect(callback);
    }
}

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

bool SensorHub::IsRunning() const { return running_.load(); }

void SensorHub::PushSample(std::shared_ptr<SensorSample> sample) {
    OnSample(std::move(sample));
}

void SensorHub::DropBuffer(const SensorId& id) {
    WriteLock lock(buffers_lock_);
    buffers_.erase(id);
}

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

void SensorHub::AlignmentLoop() {
    autolink::Rate rate(options_.publish_period);
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
