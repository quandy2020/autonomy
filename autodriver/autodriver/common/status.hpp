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
 * @brief Lightweight device health snapshot.
 */

#ifndef AUTODRIVER_COMMON_STATUS_HPP_
#define AUTODRIVER_COMMON_STATUS_HPP_

#include <cstdint>
#include <string>

#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace diagnostics {

/**
 * @brief Coarse connectivity / health for a transport or driver.
 */
enum class DeviceStatus : std::uint8_t {
    kOk = 0,
    kDisconnected = 1,
    kError = 2,
};

/**
 * @brief Point-in-time health for one sensor id (publishable later).
 */
struct DiagnosticSnapshot {
    SensorId id;
    DeviceStatus status = DeviceStatus::kDisconnected;
    std::string message;
    std::uint64_t sample_count = 0;
};

inline const char* ToString(DeviceStatus status) {
    switch (status) {
        case DeviceStatus::kOk:
            return "ok";
        case DeviceStatus::kDisconnected:
            return "disconnected";
        case DeviceStatus::kError:
            return "error";
    }
    return "unknown";
}

}  // namespace diagnostics
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_STATUS_HPP_
