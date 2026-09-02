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
 * @brief Parse a stream kind from configuration text.
 * @param text Stream name from YAML (e.g. "color", "depth")
 * @param default_kind Value returned when text is unrecognized
 * @return Parsed stream kind
 */
StreamKind ParseStreamKind(const std::string& text, StreamKind default_kind);

/**
 * @brief Check whether a device product name matches a model filter.
 * @param product_name librealsense product name string
 * @param model_filter Substring or model token from configuration
 * @return True when the device matches the filter
 */
bool MatchesModelFilter(const std::string& product_name,
                        const std::string& model_filter);

/**
 * @brief Image encoding string for a stream kind.
 * @param kind Video stream kind
 * @return ROS-style encoding name (e.g. rgb8, 16UC1)
 */
std::string EncodingForStreamKind(StreamKind kind);

/**
 * @brief Default TF frame_id for a stream kind.
 * @param kind Video stream kind
 * @return Frame identifier string
 */
std::string DefaultFrameId(StreamKind kind);

}  // namespace realsense
}  // namespace hardware

namespace io {

/**
 * @brief Whether librealsense was linked at build time.
 * @return True when RealSense support is available
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
     * @brief Acquire or create a hub for the device identified by params.
     * @param params Must include model and/or serial; options are merged on reuse.
     * @return Shared hub instance for the physical device
     */
    static std::shared_ptr<RealSenseDeviceHub> Acquire(
        const hardware::DriverParams& params);

    /**
     * @brief Destructor for autodriver::io::RealSenseDeviceHub
     */
    ~RealSenseDeviceHub();

    /**
     * @brief Subscribe to a video stream.
     * @param stream Video stream kind
     * @param width Requested frame width
     * @param height Requested frame height
     * @param fps Requested frame rate
     * @param callback Invoked for each decoded frame
     * @return Subscription id for Unsubscribe()
     */
    std::uint64_t SubscribeVideo(hardware::realsense::StreamKind stream, int width,
                                 int height, int fps,
                                 RealSenseVideoCallback callback);

    /**
     * @brief Subscribe to a colored depth point cloud.
     * @param width Requested depth map width
     * @param height Requested depth map height
     * @param fps Requested frame rate
     * @param callback Invoked for each decoded cloud
     * @return Subscription id for Unsubscribe()
     */
    std::uint64_t SubscribePointCloud(int width, int height, int fps,
                                      RealSensePointCloudCallback callback);

    /**
     * @brief Subscribe to fused accel/gyro motion streams.
     * @param callback Invoked for each IMU sample
     * @return Subscription id for Unsubscribe()
     */
    std::uint64_t SubscribeImu(RealSenseImuCallback callback);

    /**
     * @brief Remove a subscription; restarts the pipeline when already running.
     * @param subscription_id Id returned from Subscribe*()
     */
    void Unsubscribe(std::uint64_t subscription_id);

    /**
     * @brief Start the librealsense pipeline when subscriptions exist.
     * @return True on success
     */
    bool Start();

    /**
     * @brief Stop the librealsense pipeline and release streams.
     */
    void Stop();

    /**
     * @brief Whether the pipeline is currently running.
     * @return True while capturing
     */
    bool IsRunning() const;

    /**
     * @brief Last librealsense or configuration error message.
     * @return Human-readable error text
     */
    const std::string& last_error() const;

 private:
    /**
     * @brief Private constructor; use Acquire() to obtain shared instances.
     * @param params Device selection and option parameters
     */
    explicit RealSenseDeviceHub(const hardware::DriverParams& params);

    struct Impl;

    // Opaque librealsense pipeline and subscription state.
    std::unique_ptr<Impl> impl_;
};

}  // namespace io
}  // namespace autodriver
