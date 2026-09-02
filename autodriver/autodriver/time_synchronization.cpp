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

#include "autodriver/time_synchronization.hpp"

#include <cmath>

#include "autolink/time/duration.hpp"

namespace autodriver {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using ReadLock = autolink::base::ReadLockGuard<AtomicRWLock>;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

}  // namespace

/**
 * @brief Updates and returns host-aligned time for a device timestamp.
 */
autolink::Time TimeSync::Observe(const SensorId& id, const autolink::Time& device,
                                 const autolink::Time& host) {
    /**
     * @brief Raw host-minus-device offset for this observation in nanoseconds.
     */
    const std::int64_t sample_ns = (host - device).ToNanosecond();
    WriteLock lock(lock_);
    Offset& offset = offsets_[id];
    if (!offset.ready) {
        offset.ns = sample_ns;
        offset.ready = true;
    } else {
        offset.ns = static_cast<std::int64_t>(std::llround(
            (1.0 - kAlpha) * static_cast<double>(offset.ns) +
            kAlpha * static_cast<double>(sample_ns)));
    }
    return device + autolink::Duration(offset.ns);
}

/**
 * @brief Alias for Observe; records a host-device time pair.
 */
void TimeSync::Update(const SensorId& id, const autolink::Time& device,
                      const autolink::Time& host) {
    Observe(id, device, host);
}

/**
 * @brief Converts a device timestamp to host time using the stored offset.
 */
autolink::Time TimeSync::ToHostTime(const SensorId& id,
                                    const autolink::Time& device) const {
    ReadLock lock(lock_);
    const auto it = offsets_.find(id);
    if (it == offsets_.end() || !it->second.ready) {
        return device;
    }
    return device + autolink::Duration(it->second.ns);
}

void TimeSync::Reset() {
    WriteLock lock(lock_);
    offsets_.clear();
}

std::int64_t TimeSync::OffsetNs(const SensorId& id) const {
    ReadLock lock(lock_);
    const auto it = offsets_.find(id);
    if (it == offsets_.end()) {
        return 0;
    }
    return it->second.ns;
}

}  // namespace autodriver
