/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Shared OrbbecSDK pipeline hub for multi-stream devices.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autodriver/driver_params.hpp"

namespace autodriver {
namespace hardware {
namespace orbbec {

enum class StreamKind {
    kColor,
    kDepth,
    kInfrared,
};

StreamKind ParseStreamKind(const std::string& text, StreamKind default_kind);
std::string EncodingForStreamKind(StreamKind kind);
std::string DefaultFrameId(StreamKind kind);

}  // namespace orbbec
}  // namespace hardware

namespace io {

bool OrbbecAvailable();

struct OrbbecVideoFrame {
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::string encoding;
    std::vector<std::uint8_t> data;
    double timestamp_ms{0.0};
    std::string frame_id;
    automsgs::msgs::sensor_msgs::CameraInfo camera_info;
    bool has_camera_info{false};
};

struct OrbbecPointCloudFrame {
    double timestamp_ms{0.0};
    std::string frame_id;
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
};

using OrbbecVideoCallback = std::function<void(OrbbecVideoFrame frame)>;
using OrbbecPointCloudCallback =
    std::function<void(OrbbecPointCloudFrame frame)>;

/**
 * @class autodriver::io::OrbbecDeviceHub
 * @brief One OrbbecSDK pipeline per physical device, shared by drivers.
 */
class OrbbecDeviceHub {
public:
    static std::shared_ptr<OrbbecDeviceHub> Acquire(
        const hardware::DriverParams& params);

    ~OrbbecDeviceHub();

    std::uint64_t SubscribeVideo(hardware::orbbec::StreamKind stream, int width,
                                 int height, int fps,
                                 OrbbecVideoCallback callback);

    std::uint64_t SubscribePointCloud(int width, int height, int fps,
                                      OrbbecPointCloudCallback callback);

    void Unsubscribe(std::uint64_t subscription_id);

    bool Start();
    void Stop();
    bool IsRunning() const;
    const std::string& last_error() const;

private:
    explicit OrbbecDeviceHub(const hardware::DriverParams& params);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace io
}  // namespace autodriver
