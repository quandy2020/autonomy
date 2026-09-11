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
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {
namespace orbbec {

/**
 * @enum autodriver::hardware::orbbec::StreamKind
 * @brief Orbbec video stream selection for camera drivers.
 */
enum class StreamKind {
    kColor,          ///< Color / RGB stream.
    kDepth,          ///< Depth stream.
    kInfrared,       ///< Single IR stream.
    kInfraredLeft,   ///< Stereo left IR.
    kInfraredRight,  ///< Stereo right IR.
};

/**
 * @brief Parse YAML stream name into StreamKind.
 * @param[in] text Stream string (e.g. "color", "depth", "ir").
 * @param[in] default_kind Fallback when @p text is empty or unknown.
 * @return Resolved StreamKind.
 */
StreamKind ParseStreamKind(const std::string& text, StreamKind default_kind);

/**
 * @brief Default image encoding string for a stream kind.
 * @param[in] kind Stream selection.
 * @return Encoding label suitable for sensor_msgs/Image.
 */
std::string EncodingForStreamKind(StreamKind kind);

/**
 * @brief Default optical frame_id suffix for a stream kind.
 * @param[in] kind Stream selection.
 * @return Frame id fragment (e.g. for composing with sensor id).
 */
std::string DefaultFrameId(StreamKind kind);

}  // namespace orbbec
}  // namespace hardware

namespace io {

/**
 * @brief Whether OrbbecSDK was linked at build time.
 * @return true when AUTODRIVER_HAVE_ORBBEC and runtime SDK are available.
 */
bool OrbbecAvailable();

/**
 * @struct autodriver::io::OrbbecVideoFrame
 * @brief One video frame plus optional CameraInfo from the Orbbec pipeline.
 */
struct OrbbecVideoFrame {
    std::uint32_t width{0};   ///< Image width in pixels.
    std::uint32_t height{0};  ///< Image height in pixels.
    std::string encoding;     ///< sensor_msgs encoding string.
    std::vector<std::uint8_t> data;  ///< Packed image bytes.
    double timestamp_ms{0.0};        ///< Device or host timestamp (ms).
    std::string frame_id;            ///< Optical frame id.
    automsgs::msgs::sensor_msgs::CameraInfo camera_info;  ///< Intrinsics when set.
    bool has_camera_info{false};  ///< Whether @c camera_info is valid.
};

/**
 * @struct autodriver::io::OrbbecPointCloudFrame
 * @brief One PointCloud2 frame from the Orbbec depth pipeline.
 */
struct OrbbecPointCloudFrame {
    double timestamp_ms{0.0};  ///< Device or host timestamp (ms).
    std::string frame_id;      ///< Cloud frame id.
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;  ///< Filled cloud message.
};

/**
 * @brief Callback for subscribed video frames.
 */
using OrbbecVideoCallback = std::function<void(OrbbecVideoFrame frame)>;

/**
 * @brief Callback for subscribed point cloud frames.
 */
using OrbbecPointCloudCallback =
    std::function<void(OrbbecPointCloudFrame frame)>;

/**
 * @class autodriver::io::OrbbecDeviceHub
 * @brief One OrbbecSDK pipeline per physical device, shared by drivers.
 */
class OrbbecDeviceHub {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(OrbbecDeviceHub)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(OrbbecDeviceHub)
    /**
     * @brief Acquire or create a hub for the device described by @p params.
     * @param[in] params Must identify the device (serial / index / model).
     * @return Shared hub, or empty when SDK is unavailable / open fails.
     */
    static std::shared_ptr<OrbbecDeviceHub> Acquire(
        const hardware::DriverParams& params);

    /**
     * @brief Stop the pipeline and release SDK resources.
     */
    ~OrbbecDeviceHub();

    /**
     * @brief Subscribe to a video stream; starts the pipeline if needed.
     * @param[in] stream Color / depth / IR selection.
     * @param[in] width Requested width.
     * @param[in] height Requested height.
     * @param[in] fps Requested frame rate.
     * @param[in] callback Invoked on the hub/SDK thread with each frame.
     * @return Non-zero subscription id, or 0 on failure.
     */
    std::uint64_t SubscribeVideo(hardware::orbbec::StreamKind stream, int width,
                                 int height, int fps,
                                 OrbbecVideoCallback callback);

    /**
     * @brief Subscribe to depth/RGB point clouds; starts the pipeline if needed.
     * @param[in] width Requested width.
     * @param[in] height Requested height.
     * @param[in] fps Requested frame rate.
     * @param[in] callback Invoked on the hub/SDK thread with each cloud.
     * @return Non-zero subscription id, or 0 on failure.
     */
    std::uint64_t SubscribePointCloud(int width, int height, int fps,
                                      OrbbecPointCloudCallback callback);

    /**
     * @brief Remove a subscription; may stop the pipeline when unused.
     * @param[in] subscription_id Token from SubscribeVideo / SubscribePointCloud.
     */
    void Unsubscribe(std::uint64_t subscription_id);

    /**
     * @brief Start the OrbbecSDK pipeline if not already running.
     * @return true on success.
     */
    bool Start();

    /**
     * @brief Stop the pipeline (subscriptions remain until Unsubscribe).
     */
    void Stop();

    /**
     * @brief Whether the SDK pipeline thread is active.
     * @return true while Start succeeded and Stop has not completed.
     */
    bool IsRunning() const;

    /**
     * @brief Last open / start error message.
     * @return Reference to the hub error string (empty when ok).
     */
    const std::string& last_error() const;

private:
    /**
     * @brief Construct a hub for @p params (use Acquire).
     * @param[in] params Device identity and stream defaults.
     */
    explicit OrbbecDeviceHub(const hardware::DriverParams& params);

    struct Impl;
    std::unique_ptr<Impl> impl_{nullptr};
};

}  // namespace io
}  // namespace autodriver
