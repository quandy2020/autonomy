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
 * @brief Compile-time mapping from SensorType to sample types and channel helpers.
 */

#ifndef AUTODRIVER_SENSOR_TRAITS_HPP_
#define AUTODRIVER_SENSOR_TRAITS_HPP_

#include <string>
#include <string_view>

#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @struct autodriver::SensorTraits
 * @brief Associates a SensorType with its TypedSample and message type.
 * @tparam kType Sensor modality to specialize.
 */
template <SensorType kType>
struct SensorTraits;

#define AUTODRIVER_TRAITS(enum_name, sample_t)                   \
    template <>                                                  \
    struct SensorTraits<SensorType::enum_name> {                 \
        /**
         * @brief Concrete typed sample for this modality.
         */   \
        using Sample = sample_t;                                 \
        /**
         * @brief Underlying automsgs protobuf message type.
         */ \
        using Message = typename Sample::Message;                \
        /**
         * @brief Stamp the sample header and return the message.
         */ \
        static Message& ToMessage(Sample& sample) {            \
            return sample.StampInPlace();                      \
        }                                                        \
    }

AUTODRIVER_TRAITS(kImu, ImuSample);
AUTODRIVER_TRAITS(kGps, GpsSample);
AUTODRIVER_TRAITS(kCamera, CameraFrame);
AUTODRIVER_TRAITS(kLidar2d, LidarScan);
AUTODRIVER_TRAITS(kLidar3d, LidarCloud);
AUTODRIVER_TRAITS(kRangeFinder, RangeSample);
AUTODRIVER_TRAITS(kWheelOdometry, WheelOdometrySample);
AUTODRIVER_TRAITS(kRadar, RadarSample);
AUTODRIVER_TRAITS(kMicrophone, MicrophoneSample);

#undef AUTODRIVER_TRAITS

/**
 * @brief Default Autolink channel suffix for a sensor type.
 * @param type Sensor modality used to choose the suffix.
 * @param stream Optional stream hint (e.g. "depth" for camera depth topics).
 * @return Topic suffix including a leading slash when applicable.
 */
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
        case SensorType::kRadar:
            return "/radar";
        case SensorType::kMicrophone:
            return "/audio";
        case SensorType::kImu:
        case SensorType::kGps:
        case SensorType::kRangeFinder:
            return {};
    }
    return {};
}

/**
 * @brief Resolve the publish channel for a sensor.
 * When channel is empty, returns "/{id}" plus a type-specific suffix so
 * multiple sensors of the same modality never share a topic.
 * @param channel Explicit channel from configuration; may be empty.
 * @param id Sensor identifier used as the topic prefix.
 * @param type Sensor modality used to choose the default suffix.
 * @param stream Optional stream hint passed to ChannelSuffix().
 * @return Fully qualified channel name.
 */
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
