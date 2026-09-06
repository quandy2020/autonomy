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
 * @brief Shared librealsense pipeline hub for multi-stream D400 devices.
 */

#pragma once

#include <array>
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
namespace realsense {

/**
 * @brief RealSense video stream kinds supported by autodriver.
 */
enum class StreamKind {
    /**
     * @brief RGB color stream.
     */
    kColor,
    /**
     * @brief Raw depth stream.
     */
    kDepth,
    /**
     * @brief First infrared stream.
     */
    kInfrared1,
    /**
     * @brief Second infrared stream.
     */
    kInfrared2,
    /**
     * @brief Depth aligned to the color optical frame.
     */
    kAlignedDepthToColor,
    /**
     * @brief Colored point cloud derived from depth and color.
     */
    kPointCloud,
};

/**
 * @brief Parses a stream name string into a StreamKind enum value.
 */
StreamKind ParseStreamKind(const std::string& text, StreamKind default_kind);

bool MatchesModelFilter(const std::string& product_name,
                        const std::string& model_filter);

/**
 * @brief Returns the image encoding string for a RealSense stream kind.
 */
std::string EncodingForStreamKind(StreamKind kind);

/**
 * @brief Returns the default TF frame_id for a RealSense stream kind.
 */
std::string DefaultFrameId(StreamKind kind);

}  // namespace realsense
}  // namespace hardware

namespace io {

/**
 * @brief Returns true when librealsense was linked at build time.
 */
bool RealSenseAvailable();

/**
 * @brief Decoded video frame with optional CameraInfo metadata.
 */
struct RealSenseVideoFrame {
    // Image width in pixels.
    std::uint32_t width{0};

    // Image height in pixels.
    std::uint32_t height{0};

    // Pixel encoding (e.g. rgb8, 16UC1).
    std::string encoding;

    // Raw pixel buffer.
    std::vector<std::uint8_t> data;

    // Frame timestamp in milliseconds.
    double timestamp_ms{0.0};

    // TF frame_id for the optical frame.
    std::string frame_id;

    // Intrinsics when available from the device.
    automsgs::msgs::sensor_msgs::CameraInfo camera_info;

    // True when camera_info was populated.
    bool has_camera_info{false};
};

/**
 * @brief Decoded colored point cloud frame.
 */
struct RealSensePointCloudFrame {
    // Frame timestamp in milliseconds.
    double timestamp_ms{0.0};

    // TF frame_id for the cloud.
    std::string frame_id;

    // Serialized PointCloud2 message.
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
};

// Callback invoked for each decoded video frame.
using RealSenseVideoCallback = std::function<void(RealSenseVideoFrame frame)>;

// Callback invoked for each decoded point cloud frame.
using RealSensePointCloudCallback =
    std::function<void(RealSensePointCloudFrame frame)>;

// Callback invoked for fused IMU accel/gyro samples.
using RealSenseImuCallback = std::function<void(
    std::array<double, 3> linear_acceleration,
    std::array<double, 3> angular_velocity,
    double timestamp_ms)>;

/**
 * @class autodriver::io::RealSenseDeviceHub
 * @brief One librealsense pipeline per physical device, shared by all drivers.
 * Drivers subscribe to individual streams (color, depth, IR, IMU, point cloud).
 * Device options such as emitter_enabled are read from DriverParams.
 */
class RealSenseDeviceHub {
 public:
    /**
     * @brief Returns a shared hub for the device key, creating one if needed.
     */
    static std::shared_ptr<RealSenseDeviceHub> Acquire(
        const hardware::DriverParams& params);

    /**
     * @brief Stops the pipeline and worker thread on destruction.
     */
    ~RealSenseDeviceHub();

    /**
     * @brief Registers a video stream callback and restarts the hub if running.
     */
    std::uint64_t SubscribeVideo(hardware::realsense::StreamKind stream, int width,
                                 int height, int fps,
                                 RealSenseVideoCallback callback);

    /**
     * @brief Registers a point-cloud callback and restarts the hub if running.
     */
    std::uint64_t SubscribePointCloud(int width, int height, int fps,
                                      RealSensePointCloudCallback callback);

    /**
     * @brief Registers an IMU callback and restarts the hub if running.
     */
    std::uint64_t SubscribeImu(RealSenseImuCallback callback);

    /**
     * @brief Removes a subscription and restarts the hub when others remain.
     */
    void Unsubscribe(std::uint64_t subscription_id);

    /**
     * @brief Starts the pipeline and frame capture worker thread.
     */
    bool Start();

    /**
     * @brief Stops the pipeline and joins the capture worker thread.
     */
    void Stop();

    /**
     * @brief Returns true while the capture loop is active.
     */
    bool IsRunning() const;

    /**
     * @brief Returns the most recent pipeline or device error message.
     */
    const std::string& last_error() const;

 private:
    /**
     * @brief Constructs a hub bound to driver params and a device pool key.
     */
    explicit RealSenseDeviceHub(const hardware::DriverParams& params);

    struct Impl;

    // Opaque librealsense pipeline and subscription state.
    std::unique_ptr<Impl> impl_;
};

}  // namespace io
}  // namespace autodriver
