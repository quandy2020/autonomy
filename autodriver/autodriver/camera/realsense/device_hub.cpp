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

#include "autodriver/camera/realsense/device_hub.hpp"

#ifdef AUTODRIVER_HAVE_REALSENSE
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#endif

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstring>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autodriver/driver_params.hpp"
#include "autodriver/camera/realsense/camera_info.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/base/rw_lock_guard.hpp"

namespace autodriver {
namespace io {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

// Protects the shared RealSense device hub pool.
AtomicRWLock g_pool_mutex;

// Weak handles to hubs keyed by serial or index/model.
std::unordered_map<std::string, std::weak_ptr<RealSenseDeviceHub>> g_pool;

/**
 * @brief Builds a stable lookup key from serial, index, or model params.
 */
std::string DeviceKey(const hardware::DriverParams& params) {
    const std::string serial = hardware::GetString(params, "serial");
    if (!serial.empty()) {
        return serial;
    }
    const int index = hardware::ParseInt(params, "index", 0);
    const std::string model = hardware::GetString(params, "model");
    return "index:" + std::to_string(index) + ":model:" + model;
}

}  // namespace

bool RealSenseAvailable() {
#ifdef AUTODRIVER_HAVE_REALSENSE
    return true;
#else
    return false;
#endif
}

struct RealSenseDeviceHub::Impl {
    // Merged driver parameters from all subscribers on this device.
    hardware::DriverParams params;

    // Pool lookup key derived from serial or index/model.
    std::string device_key;

    // Most recent pipeline or device error message.
    std::string last_error;

    // True while the capture worker thread is active.
    std::atomic<bool> running{false};

    // Monotonic id generator for stream subscriptions.
    std::atomic<std::uint64_t> next_subscription_id{1};

    // Protects subscription vectors during subscribe/unsubscribe.
    autolink::base::AtomicRWLock mutex;

    /**
     * @brief Active video stream subscription state.
     */
    struct VideoSubscription {
        // Unique subscription handle.
        std::uint64_t id{0};
        /**
         * @brief Logical stream kind requested by the driver.
         */
        hardware::realsense::StreamKind stream{
            hardware::realsense::StreamKind::kColor};

        // Requested frame width in pixels.
        int width{640};

        // Requested frame height in pixels.
        int height{480};

        // Requested frames per second.
        int fps{30};

        // Callback invoked for each decoded video frame.
        RealSenseVideoCallback callback;
    };

    /**
     * @brief Active point-cloud subscription state.
     */
    struct PointCloudSubscription {
        // Unique subscription handle.
        std::uint64_t id{0};

        // Depth stream width used for point generation.
        int width{640};

        // Depth stream height used for point generation.
        int height{480};

        // Requested frames per second.
        int fps{30};

        // Callback invoked for each colored point cloud.
        RealSensePointCloudCallback callback;
    };

    /**
     * @brief Active IMU subscription state.
     */
    struct ImuSubscription {
        // Unique subscription handle.
        std::uint64_t id{0};

        // Callback invoked when accel and gyro samples are available.
        RealSenseImuCallback callback;
    };

    // Registered video stream subscriptions.
    std::vector<VideoSubscription> video_subscriptions;

    // Registered point-cloud subscriptions.
    std::vector<PointCloudSubscription> pointcloud_subscriptions;

    // Registered IMU subscriptions.
    std::vector<ImuSubscription> imu_subscriptions;

    // Background thread running CaptureLoop().
    std::thread worker;

#ifndef AUTODRIVER_HAVE_REALSENSE
    /**
     * @brief Reports missing librealsense when RealSense support is disabled.
     */
    bool StartPipeline() {
        last_error =
            "librealsense2 not found at build time; install librealsense2-dev "
            "and rebuild with AUTODRIVER_WITH_REALSENSE=ON";
        return false;
    }

    /**
     * @brief No-op pipeline stop when RealSense support is disabled.
     */
    void StopPipeline() {}

    /**
     * @brief No-op capture loop when RealSense support is disabled.
     */
    void CaptureLoop() {}
#else
    // librealsense runtime context for device enumeration.
    rs2::context context;

    // Active streaming pipeline bound to one physical device.
    rs2::pipeline pipeline;

    // Pipeline stream configuration rebuilt on each restart.
    rs2::config config;

    // Serial number of the device bound to the pipeline.
    std::string bound_serial;

    // Depth scale in meters per depth unit from the depth sensor.
    float depth_scale_{0.001f};

    /**
     * @brief librealsense stream type and index pair.
     */
    struct StreamSpec {
        // librealsense stream type constant.
        rs2_stream type{RS2_STREAM_COLOR};

        // Stream index within the type (e.g. IR1 vs IR2).
        int index{0};
    };

    /**
     * @brief Maps a StreamKind to librealsense stream type and index.
     */
    StreamSpec StreamSpecFor(
        const hardware::realsense::StreamKind kind) const {
        StreamSpec spec;
        switch (kind) {
            case hardware::realsense::StreamKind::kColor:
            case hardware::realsense::StreamKind::kAlignedDepthToColor:
                spec.type = RS2_STREAM_COLOR;
                break;
            case hardware::realsense::StreamKind::kDepth:
            case hardware::realsense::StreamKind::kPointCloud:
                spec.type = RS2_STREAM_DEPTH;
                break;
            case hardware::realsense::StreamKind::kInfrared1:
                spec.type = RS2_STREAM_INFRARED;
                spec.index = 1;
                break;
            case hardware::realsense::StreamKind::kInfrared2:
                spec.type = RS2_STREAM_INFRARED;
                spec.index = 2;
                break;
        }
        return spec;
    }

    /**
     * @brief Finds the first frame matching a stream type and index.
     */
    rs2::frame FindVideoFrame(const rs2::frameset& frames,
                              const StreamSpec& spec) const {
        rs2::frame holder;
        frames.foreach_rs([&](const rs2::frame& frm) {
            if (holder) {
                return;
            }
            const rs2::stream_profile profile = frm.get_profile();
            if (profile.stream_type() == spec.type &&
                profile.stream_index() == spec.index) {
                holder = frm;
            }
        });
        return holder;
    }

    /**
     * @brief Returns the librealsense pixel format for a stream kind.
     */
    rs2_format StreamFormat(const hardware::realsense::StreamKind kind) const {
        switch (kind) {
            case hardware::realsense::StreamKind::kColor:
                return RS2_FORMAT_RGB8;
            case hardware::realsense::StreamKind::kDepth:
            case hardware::realsense::StreamKind::kAlignedDepthToColor:
                return RS2_FORMAT_Z16;
            case hardware::realsense::StreamKind::kInfrared1:
            case hardware::realsense::StreamKind::kInfrared2:
                return RS2_FORMAT_Y8;
            case hardware::realsense::StreamKind::kPointCloud:
                return RS2_FORMAT_Z16;
        }
        return RS2_FORMAT_RGB8;
    }

    /**
     * @brief Returns true when aligned depth or point cloud needs color+depth.
     */
    bool NeedsColorDepthPair() const {
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream ==
                hardware::realsense::StreamKind::kAlignedDepthToColor) {
                return true;
            }
        }
        return !pointcloud_subscriptions.empty();
    }

    /**
     * @brief Returns true when any subscription requires the given stream.
     */
    bool HasStream(const hardware::realsense::StreamKind kind) const {
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == kind) {
                return true;
            }
        }
        if (kind == hardware::realsense::StreamKind::kDepth &&
            NeedsColorDepthPair()) {
            return true;
        }
        if (kind == hardware::realsense::StreamKind::kColor &&
            NeedsColorDepthPair()) {
            return true;
        }
        return false;
    }

    /**
     * @brief Selects a RealSense device by serial, model filter, or index.
     */
    rs2::device ResolveDevice() {
        const std::string serial = hardware::GetString(params, "serial");
        const std::string model = hardware::GetString(params, "model");
        const int index = hardware::ParseInt(params, "index", 0);

        const rs2::device_list devices = context.query_devices();
        if (devices.size() == 0) {
            last_error = "No Intel RealSense device connected";
            return rs2::device();
        }

        if (!serial.empty()) {
            for (rs2::device device : devices) {
                if (device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial) {
                    return device;
                }
            }
            last_error = "RealSense serial not found: " + serial;
            return rs2::device();
        }

        std::vector<rs2::device> candidates;
        for (rs2::device device : devices) {
            const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
            if (hardware::realsense::MatchesModelFilter(name, model)) {
                candidates.push_back(device);
            }
        }

        if (candidates.empty()) {
            last_error = model.empty()
                             ? "No RealSense device matched selection"
                             : "No RealSense device matched model filter: " +
                                   model;
            return rs2::device();
        }

        if (index < 0 ||
            static_cast<std::size_t>(index) >= candidates.size()) {
            last_error =
                "RealSense device index out of range: " + std::to_string(index);
            return rs2::device();
        }

        return candidates[static_cast<std::size_t>(index)];
    }

    /**
     * @brief Enables a stream in the pipeline config when a subscriber needs it.
     */
    void EnableStreamIfNeeded(const hardware::realsense::StreamKind kind,
                              int width, int height, int fps) {
        if (!HasStream(kind)) {
            return;
        }
        const StreamSpec spec = StreamSpecFor(kind);
        config.enable_stream(spec.type, spec.index, width, height,
                             StreamFormat(kind), fps);
    }

    /**
     * @brief Builds CameraInfo from a RealSense video frame intrinsics.
     */
    automsgs::msgs::sensor_msgs::CameraInfo BuildCameraInfo(
        const rs2::video_frame& frame,
        const hardware::realsense::StreamKind kind) const {
        const rs2::video_stream_profile profile =
            frame.get_profile().as<rs2::video_stream_profile>();
        const rs2_intrinsics intr = profile.get_intrinsics();
        const std::string frame_id = hardware::realsense::DefaultFrameId(kind);
        return hardware::realsense::MakeCameraInfo(
            static_cast<std::uint32_t>(frame.get_width()),
            static_cast<std::uint32_t>(frame.get_height()), frame_id, intr.fx,
            intr.fy, intr.ppx, intr.ppy, intr.coeffs, RS2_DISTORTION_COUNT);
    }

    /**
     * @brief Configures streams and IMU based on active subscriptions.
     */
    bool ConfigurePipeline() {
        config = rs2::config();
        const rs2::device device = ResolveDevice();
        if (!device) {
            return false;
        }

        bound_serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        config.enable_device(bound_serial);

        int color_w = 640;
        int color_h = 480;
        int color_fps = 30;
        int depth_w = 640;
        int depth_h = 480;
        int depth_fps = 30;

        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::realsense::StreamKind::kColor ||
                sub.stream ==
                    hardware::realsense::StreamKind::kAlignedDepthToColor) {
                color_w = sub.width;
                color_h = sub.height;
                color_fps = sub.fps;
            }
            if (sub.stream == hardware::realsense::StreamKind::kDepth ||
                sub.stream ==
                    hardware::realsense::StreamKind::kAlignedDepthToColor) {
                depth_w = sub.width;
                depth_h = sub.height;
                depth_fps = sub.fps;
            }
        }
        for (const PointCloudSubscription& sub : pointcloud_subscriptions) {
            color_w = sub.width;
            color_h = sub.height;
            color_fps = sub.fps;
            depth_w = sub.width;
            depth_h = sub.height;
            depth_fps = sub.fps;
        }

        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream ==
                hardware::realsense::StreamKind::kAlignedDepthToColor) {
                continue;
            }
            const StreamSpec spec = StreamSpecFor(sub.stream);
            config.enable_stream(spec.type, spec.index, sub.width, sub.height,
                                 StreamFormat(sub.stream), sub.fps);
        }

        if (NeedsColorDepthPair()) {
            config.enable_stream(RS2_STREAM_COLOR, 0, color_w, color_h,
                                 RS2_FORMAT_RGB8, color_fps);
            config.enable_stream(RS2_STREAM_DEPTH, 0, depth_w, depth_h,
                                 RS2_FORMAT_Z16, depth_fps);
        }

        if (!imu_subscriptions.empty()) {
            config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        }

        return true;
    }

    /**
     * @brief Applies depth emitter and related device options from params.
     */
    void ApplyDeviceOptions(const rs2::device& device) {
        const bool emitter_enabled = hardware::ParseBool(
            params, "emitter_enabled",
            hardware::ParseBool(params, "enable_ir_emitter", false));
        try {
            const rs2::depth_sensor depth = device.first<rs2::depth_sensor>();
            if (depth.supports(RS2_OPTION_EMITTER_ENABLED)) {
                depth.set_option(RS2_OPTION_EMITTER_ENABLED,
                                 emitter_enabled ? 1.f : 0.f);
            }
        } catch (const rs2::error&) {
        }
    }

    /**
     * @brief Starts the librealsense pipeline with the current config.
     */
    bool StartPipeline() {
        StopPipeline();
        if (!ConfigurePipeline()) {
            return false;
        }

        try {
            const rs2::pipeline_profile profile = pipeline.start(config);
            const rs2::device device = profile.get_device();
            const rs2::depth_sensor depth =
                device.first<rs2::depth_sensor>();
            depth_scale_ = depth.get_depth_scale();
            ApplyDeviceOptions(device);
            last_error.clear();
            return true;
        } catch (const rs2::error& ex) {
            last_error =
                std::string("RealSense pipeline start failed: ") + ex.what();
            return false;
        }
    }

    /**
     * @brief Stops the librealsense pipeline, ignoring errors.
     */
    void StopPipeline() {
        try {
            pipeline.stop();
        } catch (...) {
        }
    }

    /**
     * @brief Converts a video frame into a RealSenseVideoFrame callback payload.
     */
    void DispatchVideoFrame(const VideoSubscription& sub,
                            const rs2::video_frame& frame) {
        if (!sub.callback || !frame) {
            return;
        }

        RealSenseVideoFrame payload;
        payload.width = static_cast<std::uint32_t>(frame.get_width());
        payload.height = static_cast<std::uint32_t>(frame.get_height());
        payload.encoding =
            hardware::realsense::EncodingForStreamKind(sub.stream);
        payload.timestamp_ms = frame.get_timestamp();
        payload.frame_id = hardware::realsense::DefaultFrameId(sub.stream);
        const auto* bytes = static_cast<const std::uint8_t*>(frame.get_data());
        const std::size_t size = frame.get_data_size();
        payload.data.assign(bytes, bytes + size);
        payload.camera_info = BuildCameraInfo(frame, sub.stream);
        payload.has_camera_info = true;
        sub.callback(std::move(payload));
    }

    /**
     * @brief Builds a colored PointCloud2 message from depth and texture.
     */
    automsgs::msgs::sensor_msgs::PointCloud2 BuildPointCloud(
        const rs2::points& points, const rs2::video_frame& color,
        const std::string& frame_id, const double timestamp_ms) const {
        automsgs::msgs::sensor_msgs::PointCloud2 cloud;
        cloud.mutable_header()->set_frame_id(frame_id);
        cloud.set_height(1);
        cloud.set_width(static_cast<std::uint32_t>(points.size()));
        cloud.set_is_bigendian(false);
        cloud.set_is_dense(false);
        cloud.set_point_step(16);
        cloud.set_row_step(cloud.point_step() * cloud.width());

        auto* x_field = cloud.add_fields();
        x_field->set_name("x");
        x_field->set_offset(0);
        x_field->set_datatype(
            automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        x_field->set_count(1);
        auto* y_field = cloud.add_fields();
        y_field->set_name("y");
        y_field->set_offset(4);
        y_field->set_datatype(
            automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        y_field->set_count(1);
        auto* z_field = cloud.add_fields();
        z_field->set_name("z");
        z_field->set_offset(8);
        z_field->set_datatype(
            automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        z_field->set_count(1);
        auto* rgb_field = cloud.add_fields();
        rgb_field->set_name("rgb");
        rgb_field->set_offset(12);
        rgb_field->set_datatype(
            automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        rgb_field->set_count(1);

        std::vector<std::uint8_t> buffer(cloud.row_step(), 0);
        const rs2::vertex* vertices = points.get_vertices();
        const rs2::texture_coordinate* tex_coords = points.get_texture_coordinates();
        const auto* color_bytes =
            color ? static_cast<const std::uint8_t*>(color.get_data()) : nullptr;
        const int color_width = color ? static_cast<int>(color.get_width()) : 0;
        const int color_height =
            color ? static_cast<int>(color.get_height()) : 0;

        for (std::uint32_t i = 0; i < cloud.width(); ++i) {
            const std::size_t offset = static_cast<std::size_t>(i) * 16;
            float* xyz = reinterpret_cast<float*>(buffer.data() + offset);
            xyz[0] = vertices[i].x;
            xyz[1] = vertices[i].y;
            xyz[2] = vertices[i].z;

            std::uint8_t r = 255;
            std::uint8_t g = 255;
            std::uint8_t b = 255;
            if (color_bytes != nullptr) {
                const int px = static_cast<int>(tex_coords[i].u * color_width);
                const int py = static_cast<int>(tex_coords[i].v * color_height);
                if (px >= 0 && px < color_width && py >= 0 && py < color_height) {
                    const std::size_t color_offset =
                        static_cast<std::size_t>(py * color_width + px) * 3;
                    r = color_bytes[color_offset];
                    g = color_bytes[color_offset + 1];
                    b = color_bytes[color_offset + 2];
                }
            }
            const std::uint32_t rgb =
                (static_cast<std::uint32_t>(r) << 16) |
                (static_cast<std::uint32_t>(g) << 8) |
                static_cast<std::uint32_t>(b);
            std::memcpy(buffer.data() + offset + 12, &rgb, sizeof(rgb));
        }
        cloud.set_data(reinterpret_cast<const char*>(buffer.data()),
                       buffer.size());
        (void)timestamp_ms;
        return cloud;
    }

    /**
     * @brief Polls frames and dispatches video, point cloud, and IMU callbacks.
     */
    void CaptureLoop() {
        // Latest accelerometer sample pending IMU fusion.
        std::array<double, 3> latest_accel{{0.0, 0.0, 0.0}};

        // Latest gyroscope sample pending IMU fusion.
        std::array<double, 3> latest_gyro{{0.0, 0.0, 0.0}};

        // Timestamp of the most recent partial IMU frame.
        double imu_timestamp_ms = 0.0;

        // True after an accel frame has been received this cycle.
        bool have_accel = false;

        // True after a gyro frame has been received this cycle.
        bool have_gyro = false;
        rs2::align align_to_color(RS2_STREAM_COLOR);
        rs2::pointcloud pc;

        while (running.load()) {
            rs2::frameset frames;
            try {
                if (!pipeline.poll_for_frames(&frames)) {
                    continue;
                }
            } catch (const rs2::error& ex) {
                last_error = std::string("RealSense poll failed: ") + ex.what();
                break;
            }

            std::vector<VideoSubscription> video_subs;
            std::vector<PointCloudSubscription> pointcloud_subs;
            std::vector<ImuSubscription> imu_subs;
            {
                WriteLock lock(mutex);
                video_subs = video_subscriptions;
                pointcloud_subs = pointcloud_subscriptions;
                imu_subs = imu_subscriptions;
            }

            const bool needs_align = std::any_of(
                video_subs.begin(), video_subs.end(),
                [](const VideoSubscription& sub) {
                    return sub.stream ==
                           hardware::realsense::StreamKind::kAlignedDepthToColor;
                });

            rs2::frameset aligned = frames;
            if (needs_align || !pointcloud_subs.empty()) {
                aligned = align_to_color.process(frames);
            }

            for (const VideoSubscription& sub : video_subs) {
                if (sub.stream ==
                    hardware::realsense::StreamKind::kAlignedDepthToColor) {
                    const rs2::depth_frame depth =
                        aligned.get_depth_frame();
                    DispatchVideoFrame(sub, depth);
                    continue;
                }
                const StreamSpec spec = StreamSpecFor(sub.stream);
                const rs2::frame frame = FindVideoFrame(frames, spec);
                if (frame) {
                    DispatchVideoFrame(sub, frame.as<rs2::video_frame>());
                }
            }

            if (!pointcloud_subs.empty()) {
                const rs2::depth_frame depth = aligned.get_depth_frame();
                const rs2::video_frame color = aligned.get_color_frame();
                if (depth && color) {
                    pc.map_to(color);
                    const rs2::points points = pc.calculate(depth);
                    for (const PointCloudSubscription& sub : pointcloud_subs) {
                        if (!sub.callback) {
                            continue;
                        }
                        RealSensePointCloudFrame payload;
                        payload.timestamp_ms = depth.get_timestamp();
                        payload.frame_id = "camera_depth_optical_frame";
                        payload.cloud = BuildPointCloud(
                            points, color, payload.frame_id,
                            payload.timestamp_ms);
                        sub.callback(std::move(payload));
                    }
                }
            }

            if (!imu_subs.empty()) {
                const rs2::motion_frame accel =
                    frames.first_or_default(RS2_STREAM_ACCEL);
                const rs2::motion_frame gyro =
                    frames.first_or_default(RS2_STREAM_GYRO);
                if (accel) {
                    const rs2_vector v = accel.get_motion_data();
                    latest_accel = {v.x, v.y, v.z};
                    imu_timestamp_ms = accel.get_timestamp();
                    have_accel = true;
                }
                if (gyro) {
                    const rs2_vector v = gyro.get_motion_data();
                    latest_gyro = {v.x, v.y, v.z};
                    imu_timestamp_ms = gyro.get_timestamp();
                    have_gyro = true;
                }
                if (have_accel && have_gyro) {
                    for (const ImuSubscription& sub : imu_subs) {
                        if (sub.callback) {
                            sub.callback(latest_accel, latest_gyro,
                                         imu_timestamp_ms);
                        }
                    }
                }
            }
        }
    }
#endif
};

RealSenseDeviceHub::RealSenseDeviceHub(const hardware::DriverParams& params)
    : impl_(std::make_unique<Impl>()) {
    impl_->params = params;
    impl_->device_key = DeviceKey(params);
}

RealSenseDeviceHub::~RealSenseDeviceHub() { Stop(); }

std::shared_ptr<RealSenseDeviceHub> RealSenseDeviceHub::Acquire(
    const hardware::DriverParams& params) {
    const std::string key = DeviceKey(params);
    WriteLock lock(g_pool_mutex);
    if (const auto it = g_pool.find(key); it != g_pool.end()) {
        if (auto existing = it->second.lock()) {
            for (const auto& entry : params) {
                existing->impl_->params.emplace(entry.first, entry.second);
            }
            return existing;
        }
    }

    auto hub = std::shared_ptr<RealSenseDeviceHub>(new RealSenseDeviceHub(params));
    g_pool[key] = hub;
    return hub;
}

std::uint64_t RealSenseDeviceHub::SubscribeVideo(
    const hardware::realsense::StreamKind stream, const int width,
    const int height, const int fps, RealSenseVideoCallback callback) {
    const bool restart = impl_->running.load();
    if (restart) {
        Stop();
    }

    std::uint64_t id = 0;
    {
        WriteLock lock(impl_->mutex);
        id = impl_->next_subscription_id.fetch_add(1);
        impl_->video_subscriptions.push_back(
            Impl::VideoSubscription{id, stream, width, height, fps,
                                    std::move(callback)});
    }

    if (restart) {
        Start();
    }
    return id;
}

std::uint64_t RealSenseDeviceHub::SubscribePointCloud(
    const int width, const int height, const int fps,
    RealSensePointCloudCallback callback) {
    const bool restart = impl_->running.load();
    if (restart) {
        Stop();
    }

    std::uint64_t id = 0;
    {
        WriteLock lock(impl_->mutex);
        id = impl_->next_subscription_id.fetch_add(1);
        impl_->pointcloud_subscriptions.push_back(
            Impl::PointCloudSubscription{id, width, height, fps,
                                         std::move(callback)});
    }

    if (restart) {
        Start();
    }
    return id;
}

std::uint64_t RealSenseDeviceHub::SubscribeImu(RealSenseImuCallback callback) {
    const bool restart = impl_->running.load();
    if (restart) {
        Stop();
    }

    std::uint64_t id = 0;
    {
        WriteLock lock(impl_->mutex);
        id = impl_->next_subscription_id.fetch_add(1);
        impl_->imu_subscriptions.push_back(
            Impl::ImuSubscription{id, std::move(callback)});
    }

    if (restart) {
        Start();
    }
    return id;
}

void RealSenseDeviceHub::Unsubscribe(const std::uint64_t subscription_id) {
    const bool was_running = impl_->running.load();
    if (was_running) {
        Stop();
    }

    {
        WriteLock lock(impl_->mutex);
        auto& video = impl_->video_subscriptions;
        video.erase(
            std::remove_if(
                video.begin(), video.end(),
                [subscription_id](const Impl::VideoSubscription& sub) {
                    return sub.id == subscription_id;
                }),
            video.end());
        auto& pointcloud = impl_->pointcloud_subscriptions;
        pointcloud.erase(
            std::remove_if(
                pointcloud.begin(), pointcloud.end(),
                [subscription_id](const Impl::PointCloudSubscription& sub) {
                    return sub.id == subscription_id;
                }),
            pointcloud.end());
        auto& imu = impl_->imu_subscriptions;
        imu.erase(
            std::remove_if(
                imu.begin(), imu.end(),
                [subscription_id](const Impl::ImuSubscription& sub) {
                    return sub.id == subscription_id;
                }),
            imu.end());
    }

    if (was_running &&
        (!impl_->video_subscriptions.empty() ||
         !impl_->pointcloud_subscriptions.empty() ||
         !impl_->imu_subscriptions.empty())) {
        Start();
    }
}

bool RealSenseDeviceHub::Start() {
    if (impl_->running.exchange(true)) {
        return true;
    }

    {
        WriteLock lock(impl_->mutex);
        if (impl_->video_subscriptions.empty() &&
            impl_->pointcloud_subscriptions.empty() &&
            impl_->imu_subscriptions.empty()) {
            impl_->last_error = "RealSense hub has no active subscriptions";
            impl_->running = false;
            return false;
        }
    }

    if (!impl_->StartPipeline()) {
        impl_->running = false;
        return false;
    }

    impl_->worker = std::thread([this]() { impl_->CaptureLoop(); });
    return true;
}

void RealSenseDeviceHub::Stop() {
    if (!impl_->running.exchange(false)) {
        return;
    }

    impl_->StopPipeline();
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
}

bool RealSenseDeviceHub::IsRunning() const { return impl_->running.load(); }

const std::string& RealSenseDeviceHub::last_error() const {
    return impl_->last_error;
}

}  // namespace io

namespace hardware {
namespace realsense {
namespace {

/**
 * @brief Lowercases a string for case-insensitive RealSense matching.
 */
std::string ToLower(std::string text) {
    std::transform(
        text.begin(), text.end(), text.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

}  // namespace

StreamKind ParseStreamKind(const std::string& text,
                           const StreamKind default_kind) {
    const std::string value = ToLower(text);
    if (value.empty() || value == "color" || value == "rgb") {
        return StreamKind::kColor;
    }
    if (value == "depth" || value == "z16") {
        return StreamKind::kDepth;
    }
    if (value == "infrared" || value == "ir" || value == "ir1" ||
        value == "infrared1" || value == "infra1") {
        return StreamKind::kInfrared1;
    }
    if (value == "ir2" || value == "infrared2" || value == "infra2") {
        return StreamKind::kInfrared2;
    }
    if (value == "aligned_depth_to_color" || value == "aligned_depth") {
        return StreamKind::kAlignedDepthToColor;
    }
    if (value == "pointcloud" || value == "points") {
        return StreamKind::kPointCloud;
    }
    return default_kind;
}

bool MatchesModelFilter(const std::string& product_name,
                        const std::string& model_filter) {
    if (model_filter.empty()) {
        return true;
    }
    const std::string name = ToLower(product_name);
    const std::string model = ToLower(model_filter);
    return name.find(model) != std::string::npos;
}

std::string EncodingForStreamKind(const StreamKind kind) {
    switch (kind) {
        case StreamKind::kColor:
            return "rgb8";
        case StreamKind::kDepth:
        case StreamKind::kAlignedDepthToColor:
            return "16UC1";
        case StreamKind::kInfrared1:
        case StreamKind::kInfrared2:
            return "mono8";
        case StreamKind::kPointCloud:
            return {};
    }
    return "rgb8";
}

std::string DefaultFrameId(const StreamKind kind) {
    switch (kind) {
        case StreamKind::kColor:
            return "camera_color_optical_frame";
        case StreamKind::kDepth:
        case StreamKind::kAlignedDepthToColor:
            return "camera_depth_optical_frame";
        case StreamKind::kInfrared1:
            return "camera_infra1_optical_frame";
        case StreamKind::kInfrared2:
            return "camera_infra2_optical_frame";
        case StreamKind::kPointCloud:
            return "camera_depth_optical_frame";
    }
    return "camera_link";
}

}  // namespace realsense
}  // namespace hardware
}  // namespace autodriver
