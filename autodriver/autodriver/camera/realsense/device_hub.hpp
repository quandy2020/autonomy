/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file device_hub.hpp
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
#include "autolink/common/macros.hpp"

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
 * @param[in] text Stream string (e.g. "color", "depth", "ir1").
 * @param[in] default_kind Fallback when @p text is empty or unknown.
 * @return Resolved StreamKind.
 */
StreamKind ParseStreamKind(const std::string& text, StreamKind default_kind);

/**
 * @brief Whether a device product name matches a YAML model filter.
 * @param[in] product_name Device product / name string from librealsense.
 * @param[in] model_filter Substring filter from DriverParams (empty matches all).
 * @return True when @p model_filter is empty or found in @p product_name.
 */
bool MatchesModelFilter(const std::string& product_name,
                        const std::string& model_filter);

/**
 * @brief Returns the image encoding string for a RealSense stream kind.
 * @param[in] kind Stream selection.
 * @return Encoding label suitable for sensor_msgs/Image.
 */
std::string EncodingForStreamKind(StreamKind kind);

/**
 * @brief Returns the default TF frame_id for a RealSense stream kind.
 * @param[in] kind Stream selection.
 * @return Default optical frame_id string.
 */
std::string DefaultFrameId(StreamKind kind);

}  // namespace realsense
}  // namespace hardware

namespace io {

/**
 * @brief Returns true when librealsense was linked at build time.
 * @return True when AUTODRIVER_HAVE_REALSENSE (or equivalent) is enabled.
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
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(RealSenseDeviceHub)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(RealSenseDeviceHub)
    /**
     * @brief Returns a shared hub for the device key, creating one if needed.
     * @param[in] params Device identity and options (serial / index / model).
     * @return Shared hub instance for the resolved device key.
     */
    static std::shared_ptr<RealSenseDeviceHub> Acquire(
        const hardware::DriverParams& params);

    /**
     * @brief Stops the pipeline and worker thread on destruction.
     */
    ~RealSenseDeviceHub();

    /**
     * @brief Registers a video stream callback and restarts the hub if running.
     * @param[in] stream Color / depth / IR / aligned-depth selection.
     * @param[in] width Requested width in pixels.
     * @param[in] height Requested height in pixels.
     * @param[in] fps Requested frames per second.
     * @param[in] callback Invoked with each decoded video frame.
     * @return Non-zero subscription id used with Unsubscribe().
     */
    std::uint64_t SubscribeVideo(hardware::realsense::StreamKind stream, int width,
                                 int height, int fps,
                                 RealSenseVideoCallback callback);

    /**
     * @brief Registers a point-cloud callback and restarts the hub if running.
     * @param[in] width Requested depth width in pixels.
     * @param[in] height Requested depth height in pixels.
     * @param[in] fps Requested frames per second.
     * @param[in] callback Invoked with each decoded point cloud frame.
     * @return Non-zero subscription id used with Unsubscribe().
     */
    std::uint64_t SubscribePointCloud(int width, int height, int fps,
                                      RealSensePointCloudCallback callback);

    /**
     * @brief Registers an IMU callback and restarts the hub if running.
     * @param[in] callback Invoked with fused accel/gyro samples.
     * @return Non-zero subscription id used with Unsubscribe().
     */
    std::uint64_t SubscribeImu(RealSenseImuCallback callback);

    /**
     * @brief Removes a subscription and restarts the hub when others remain.
     * @param[in] subscription_id Token from SubscribeVideo / PointCloud / Imu.
     */
    void Unsubscribe(std::uint64_t subscription_id);

    /**
     * @brief Starts the pipeline and frame capture worker thread.
     * @return True when the pipeline started (or was already running).
     */
    bool Start();

    /**
     * @brief Stops the pipeline and joins the capture worker thread.
     */
    void Stop();

    /**
     * @brief Returns true while the capture loop is active.
     * @return True after a successful Start until Stop completes.
     */
    bool IsRunning() const;

    /**
     * @brief Returns the most recent pipeline or device error message.
     * @return Reference to the hub error string (empty when ok).
     */
    const std::string& last_error() const;

 private:
    /**
     * @brief Constructs a hub bound to driver params and a device pool key.
     * @param[in] params Device identity and stream defaults.
     */
    explicit RealSenseDeviceHub(const hardware::DriverParams& params);

    struct Impl;

    // Opaque librealsense pipeline and subscription state.
    std::unique_ptr<Impl> impl_{nullptr};
};

}  // namespace io
}  // namespace autodriver
