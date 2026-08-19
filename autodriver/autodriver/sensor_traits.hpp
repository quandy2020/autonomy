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

#ifndef AUTODRIVER_SENSOR_TRAITS_HPP_
#define AUTODRIVER_SENSOR_TRAITS_HPP_

#include <string>
#include <string_view>

#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

template <SensorType kType>
struct SensorTraits;

#define AUTODRIVER_TRAITS(enum_name, sample_t)                   \
    template <>                                                  \
    struct SensorTraits<SensorType::enum_name> {                 \
        using Sample = sample_t;                                 \
        using Message = typename Sample::Message;                \
        static Message& ToMessage(Sample& sample) {              \
            return sample.StampInPlace();                        \
        }                                                        \
    }

AUTODRIVER_TRAITS(kImu, ImuSample);
AUTODRIVER_TRAITS(kGps, GpsSample);
AUTODRIVER_TRAITS(kCamera, CameraFrame);
AUTODRIVER_TRAITS(kLidar2d, LidarScan);
AUTODRIVER_TRAITS(kLidar3d, LidarCloud);
AUTODRIVER_TRAITS(kRangeFinder, RangeSample);
AUTODRIVER_TRAITS(kWheelOdometry, WheelOdometrySample);

#undef AUTODRIVER_TRAITS

inline std::string ChannelSuffix(SensorType type, std::string_view stream = {}) {
    switch (type) {
        case SensorType::kCamera:
            return stream == "depth" ? "/depth/image_raw" : "/image_raw";
        case SensorType::kLidar2d:
            return "/scan";
        case SensorType::kLidar3d:
            return "/points";
        case SensorType::kWheelOdometry:
            return "/odom";
        case SensorType::kImu:
        case SensorType::kGps:
        case SensorType::kRangeFinder:
            return {};
    }
    return {};
}

// Empty `channel` → "/{id}" plus a type-specific suffix so N sensors of
// the same modality never share a topic.
inline std::string ResolveChannel(const std::string& channel, const SensorId& id,
                                  SensorType type,
                                  std::string_view stream = {}) {
    if (!channel.empty()) {
        return channel;
    }
    return "/" + id + ChannelSuffix(type, stream);
}

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_TRAITS_HPP_
