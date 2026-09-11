/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file time_synchronization.hpp
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
#include "autolink/common/macros.hpp"

namespace autodriver {

/**
 * @class autodriver::TimeSync
 * @brief Tracks host = device + offset per sensor using exponential smoothing.
 */
class TimeSync {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(TimeSync)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(TimeSync)
    /**
     * @brief Default constructor (DISALLOW_COPY suppresses the implicit one).
     */
    TimeSync() = default;

    /**
     * @brief Updates and returns host-aligned time for a device timestamp.
     * @param[in] id Sensor whose clock offset is estimated.
     * @param[in] device Timestamp reported by the device.
     * @param[in] host Corresponding host receive time.
     * @return Host-aligned time for @p device after updating the offset.
     */
    autolink::Time Observe(const SensorId& id, const autolink::Time& device,
                           const autolink::Time& host);

    /**
     * @brief Alias for Observe; records a host-device time pair.
     * @param[in] id Sensor whose clock offset is estimated.
     * @param[in] device Timestamp reported by the device.
     * @param[in] host Corresponding host receive time.
     */
    void Update(const SensorId& id, const autolink::Time& device,
                const autolink::Time& host);

    /**
     * @brief Converts a device timestamp to host time using the stored offset.
     * @param[in] id Sensor whose offset should be applied.
     * @param[in] device Device timestamp to convert.
     * @return Host time = device + stored offset (identity when never observed).
     */
    autolink::Time ToHostTime(const SensorId& id,
                              const autolink::Time& device) const;

    /**
     * @brief Clears all per-sensor offset estimates.
     */
    void Reset();

    /**
     * @brief Returns the stored host-minus-device offset in nanoseconds.
     * @param[in] id Sensor whose offset is queried.
     * @return Offset in nanoseconds (0 when never observed).
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
