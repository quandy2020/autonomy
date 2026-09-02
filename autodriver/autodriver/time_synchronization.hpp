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
 * @brief Per-sensor device-to-host clock offset estimation.
 */

#ifndef AUTODRIVER_TIME_SYNCHRONIZATION_HPP_
#define AUTODRIVER_TIME_SYNCHRONIZATION_HPP_

#include <cstdint>
#include <unordered_map>

#include "autodriver/sensor_id.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/base/rw_lock_guard.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {

/**
 * @class autodriver::TimeSync
 * @brief Tracks host = device + offset per sensor using exponential smoothing.
 */
class TimeSync {
public:
    /**
     * @brief Observe a device/host pair and return the mapped host time.
     * @param id Sensor identifier for the offset estimate.
     * @param device Timestamp reported by the device.
     * @param host Host receive time for the same sample.
     * @return Mapped host time using the updated offset.
     */
    autolink::Time Observe(const SensorId& id, const autolink::Time& device,
                           const autolink::Time& host);

    /**
     * @brief Update the offset estimate without returning a mapped time.
     * @param id Sensor identifier for the offset estimate.
     * @param device Timestamp reported by the device.
     * @param host Host receive time for the same sample.
     */
    void Update(const SensorId& id, const autolink::Time& device,
                const autolink::Time& host);

    /**
     * @brief Map a device timestamp to host time using the current offset.
     * @param id Sensor identifier whose offset should be applied.
     * @param device Device timestamp to convert.
     * @return Host time computed as device plus the stored offset.
     */
    autolink::Time ToHostTime(const SensorId& id,
                              const autolink::Time& device) const;

    /**
     * @brief Clear all per-sensor offset estimates.
     */
    void Reset();

    /**
     * @brief Current offset in nanoseconds (host - device).
     * @param id Sensor identifier to query.
     * @return Offset in nanoseconds; zero when no estimate exists yet.
     */
    std::int64_t OffsetNs(const SensorId& id) const;

private:
    /**
     * @brief Smoothed clock offset state for one sensor.
     */
    struct Offset {
        // Offset in nanoseconds (host - device).
        std::int64_t ns{0};

        // True after at least one observation has been applied.
        bool ready{false};
    };

    // Protects the offsets_ map.
    mutable autolink::base::AtomicRWLock lock_;

    // Per-sensor smoothed offset estimates.
    std::unordered_map<SensorId, Offset> offsets_;

    // Exponential smoothing factor for offset updates.
    static constexpr double kAlpha = 0.2;
};

}  // namespace autodriver

#endif  // AUTODRIVER_TIME_SYNCHRONIZATION_HPP_
