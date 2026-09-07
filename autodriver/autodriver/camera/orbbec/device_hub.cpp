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

#include "autodriver/camera/orbbec/device_hub.hpp"

#ifdef AUTODRIVER_HAVE_ORBBEC
#include <libobsensor/ObSensor.hpp>
#endif

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstring>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autodriver/camera/orbbec/camera_info.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/base/rw_lock_guard.hpp"

namespace autodriver {
namespace io {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

AtomicRWLock g_pool_mutex;
std::unordered_map<std::string, std::weak_ptr<OrbbecDeviceHub>> g_pool;

std::string DeviceKey(const hardware::DriverParams& params) {
    const std::string serial = hardware::GetString(params, "serial");
    if (!serial.empty()) {
        return serial;
    }
    const int index = hardware::ParseInt(params, "index", 0);
    const std::string model = hardware::GetString(params, "model");
    return "index:" + std::to_string(index) + ":model:" + model;
}

std::string ToLower(std::string text) {
    std::transform(
        text.begin(), text.end(), text.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

}  // namespace

bool OrbbecAvailable() {
#ifdef AUTODRIVER_HAVE_ORBBEC
    return true;
#else
    return false;
#endif
}

struct OrbbecDeviceHub::Impl {
    hardware::DriverParams params;
    std::string device_key;
    std::string last_error;
    std::atomic<bool> running{false};
    std::atomic<std::uint64_t> next_subscription_id{1};
    autolink::base::AtomicRWLock mutex;

    struct VideoSubscription {
        std::uint64_t id{0};
        hardware::orbbec::StreamKind stream{hardware::orbbec::StreamKind::kColor};
        int width{640};
        int height{480};
        int fps{30};
        OrbbecVideoCallback callback;
    };

    struct PointCloudSubscription {
        std::uint64_t id{0};
        int width{640};
        int height{480};
        int fps{30};
        OrbbecPointCloudCallback callback;
    };

    std::vector<VideoSubscription> video_subscriptions;
    std::vector<PointCloudSubscription> pointcloud_subscriptions;
    std::thread worker;

#ifndef AUTODRIVER_HAVE_ORBBEC
    bool StartPipeline() {
        last_error =
            "OrbbecSDK not found at build time; install OrbbecSDK and rebuild "
            "with AUTODRIVER_WITH_ORBBEC=ON";
        return false;
    }
    void StopPipeline() {}
    void CaptureLoop() {}
#else
    std::shared_ptr<ob::Pipeline> pipeline;
    std::shared_ptr<ob::Config> config;
    std::shared_ptr<ob::PointCloudFilter> point_cloud_filter;
    std::shared_ptr<ob::Align> align_filter;
    std::vector<std::shared_ptr<ob::Filter>> depth_filters;

    bool NeedsDepth() const {
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::orbbec::StreamKind::kDepth) {
                return true;
            }
        }
        return !pointcloud_subscriptions.empty();
    }

    bool NeedsColor() const {
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::orbbec::StreamKind::kColor) {
                return true;
            }
        }
        return !pointcloud_subscriptions.empty();
    }

    bool NeedsInfrared() const {
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::orbbec::StreamKind::kInfrared) {
                return true;
            }
        }
        return false;
    }

    bool NeedsInfraredLeft() const {
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::orbbec::StreamKind::kInfraredLeft) {
                return true;
            }
        }
        return false;
    }

    bool NeedsInfraredRight() const {
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::orbbec::StreamKind::kInfraredRight) {
                return true;
            }
        }
        return false;
    }

    struct IrProfile {
        int width{0};
        int height{0};
        int fps{0};
    };

    IrProfile ProfileFor(hardware::orbbec::StreamKind stream) const {
        IrProfile profile;
        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == stream) {
                profile.width = sub.width;
                profile.height = sub.height;
                profile.fps = sub.fps;
            }
        }
        return profile;
    }

    std::shared_ptr<ob::Device> ResolveDevice() {
        ob::Context context;
        auto list = context.queryDeviceList();
        if (list == nullptr || list->getCount() == 0) {
            last_error = "No Orbbec device connected";
            return nullptr;
        }
        const std::string serial = hardware::GetString(params, "serial");
        const std::string model = hardware::GetString(params, "model");
        const int index = hardware::ParseInt(params, "index", 0);

        if (!serial.empty()) {
            for (uint32_t i = 0; i < list->getCount(); ++i) {
                auto device = list->getDevice(i);
                auto info = device->getDeviceInfo();
                if (info && serial == info->getSerialNumber()) {
                    return device;
                }
            }
            last_error = "Orbbec device serial not found: " + serial;
            return nullptr;
        }

        std::vector<std::shared_ptr<ob::Device>> matched;
        for (uint32_t i = 0; i < list->getCount(); ++i) {
            auto device = list->getDevice(i);
            auto info = device->getDeviceInfo();
            const std::string name = info ? info->getName() : "";
            if (model.empty() ||
                ToLower(name).find(ToLower(model)) != std::string::npos) {
                matched.push_back(device);
            }
        }
        if (matched.empty()) {
            last_error = "No Orbbec device matched model filter";
            return nullptr;
        }
        if (index < 0 || static_cast<std::size_t>(index) >= matched.size()) {
            last_error = "Orbbec device index out of range";
            return nullptr;
        }
        return matched[static_cast<std::size_t>(index)];
    }

    /** @brief 0 / negative → OB_*_ANY (OrbbecSDK_ROS2 width/height/fps:=0). */
    static std::uint32_t StreamDim(int value) {
        return value > 0 ? static_cast<std::uint32_t>(value) : OB_WIDTH_ANY;
    }

    bool ConfigurePipeline(const std::shared_ptr<ob::Device>& device) {
        config = std::make_shared<ob::Config>();
        int color_w = 0;
        int color_h = 0;
        int color_fps = 0;
        int depth_w = 0;
        int depth_h = 0;
        int depth_fps = 0;

        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::orbbec::StreamKind::kColor) {
                color_w = sub.width;
                color_h = sub.height;
                color_fps = sub.fps;
            } else if (sub.stream == hardware::orbbec::StreamKind::kDepth) {
                depth_w = sub.width;
                depth_h = sub.height;
                depth_fps = sub.fps;
            }
        }
        for (const PointCloudSubscription& sub : pointcloud_subscriptions) {
            if (sub.width > 0) {
                color_w = sub.width;
                depth_w = sub.width;
            }
            if (sub.height > 0) {
                color_h = sub.height;
                depth_h = sub.height;
            }
            if (sub.fps > 0) {
                color_fps = sub.fps;
                depth_fps = sub.fps;
            }
        }

        const IrProfile ir = ProfileFor(hardware::orbbec::StreamKind::kInfrared);
        const IrProfile left_ir =
            ProfileFor(hardware::orbbec::StreamKind::kInfraredLeft);
        const IrProfile right_ir =
            ProfileFor(hardware::orbbec::StreamKind::kInfraredRight);

        try {
            if (NeedsColor()) {
                // Prefer RGB for Image encoding rgb8; ROS2 default is ANY.
                config->enableVideoStream(OB_STREAM_COLOR, StreamDim(color_w),
                                          StreamDim(color_h),
                                          StreamDim(color_fps), OB_FORMAT_RGB);
            }
            if (NeedsDepth()) {
                // Official depth_format:=ANY — let SDK pick with HW D2D.
                config->enableVideoStream(OB_STREAM_DEPTH, StreamDim(depth_w),
                                          StreamDim(depth_h),
                                          StreamDim(depth_fps), OB_FORMAT_ANY);
            }
            // Gemini 330 series exposes stereo IR as LEFT/RIGHT, not OB_STREAM_IR.
            if (NeedsInfraredLeft()) {
                config->enableVideoStream(
                    OB_STREAM_IR_LEFT, StreamDim(left_ir.width),
                    StreamDim(left_ir.height), StreamDim(left_ir.fps),
                    OB_FORMAT_Y8);
            }
            if (NeedsInfraredRight()) {
                config->enableVideoStream(
                    OB_STREAM_IR_RIGHT, StreamDim(right_ir.width),
                    StreamDim(right_ir.height), StreamDim(right_ir.fps),
                    OB_FORMAT_Y8);
            }
            if (NeedsInfrared()) {
                config->enableVideoStream(OB_STREAM_IR, StreamDim(ir.width),
                                          StreamDim(ir.height),
                                          StreamDim(ir.fps), OB_FORMAT_Y8);
            }
            // frame_aggregate_mode:=ANY (OrbbecSDK_ROS2 default).
            config->setFrameAggregateOutputMode(
                OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
            if (hardware::ParseBool(params, "enable_depth_scale", true)) {
                config->setDepthScaleRequire(true);
            }
        } catch (const ob::Error& ex) {
            last_error =
                std::string("Orbbec config failed: ") + ex.what();
            return false;
        }
        (void)device;
        return true;
    }

    bool StartPipeline() {
        StopPipeline();
        auto device = ResolveDevice();
        if (!device) {
            return false;
        }
        try {
            ApplyDeviceOptions(device);
            SetupDepthFilters(device);
            pipeline = std::make_shared<ob::Pipeline>(device);
            if (!ConfigurePipeline(device)) {
                return false;
            }
            pipeline->enableFrameSync();
            pipeline->start(config);
            point_cloud_filter = std::make_shared<ob::PointCloudFilter>();
            align_filter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
            last_error.clear();
            return true;
        } catch (const ob::Error& ex) {
            last_error =
                std::string("Orbbec pipeline start failed: ") + ex.what();
            pipeline.reset();
            return false;
        }
    }

    static bool PropertyWritable(const std::shared_ptr<ob::Device>& device,
                                 OBPropertyID id) {
        return device->isPropertySupported(id, OB_PERMISSION_WRITE) ||
               device->isPropertySupported(id, OB_PERMISSION_READ_WRITE);
    }

    /**
     * @brief Device-level options from OrbbecSDK_ROS2 gemini_330_series.launch.py.
     */
    void ApplyDeviceOptions(const std::shared_ptr<ob::Device>& device) {
        if (!device) {
            return;
        }

        // Official default: device_preset:=Default (not High Accuracy).
        const std::string preset =
            hardware::GetString(params, "device_preset", "Default");
        if (!preset.empty() && preset != "Custom") {
            try {
                device->loadPreset(preset.c_str());
            } catch (const ob::Error&) {
            }
        }

        // disparity_to_depth_mode:=HW (OrbbecSDK_ROS2 default).
        std::string d2d_mode =
            hardware::GetString(params, "disparity_to_depth_mode", "HW");
        for (char& c : d2d_mode) {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        try {
            if (PropertyWritable(device, OB_PROP_DISPARITY_TO_DEPTH_BOOL) &&
                PropertyWritable(device, OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL)) {
                if (d2d_mode == "HW") {
                    device->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL,
                                            true);
                    device->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL,
                                            false);
                } else if (d2d_mode == "SW") {
                    device->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL,
                                            false);
                    device->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL,
                                            true);
                } else if (d2d_mode == "DISABLE") {
                    device->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL,
                                            false);
                    device->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL,
                                            false);
                }
            }
        } catch (const ob::Error&) {
        }

        const bool laser_enabled = hardware::ParseBool(
            params, "enable_laser",
            hardware::ParseBool(
                params, "emitter_enabled",
                hardware::ParseBool(params, "enable_ir_emitter", true)));
        try {
            if (PropertyWritable(device, OB_PROP_LASER_CONTROL_INT)) {
                device->setIntProperty(OB_PROP_LASER_CONTROL_INT,
                                       laser_enabled ? 1 : 0);
            } else if (PropertyWritable(device, OB_PROP_LASER_BOOL)) {
                device->setBoolProperty(OB_PROP_LASER_BOOL, laser_enabled);
            }
        } catch (const ob::Error&) {
        }

        const int laser_energy =
            hardware::ParseInt(params, "laser_energy_level", -1);
        if (laser_energy >= 0) {
            try {
                if (PropertyWritable(device, OB_PROP_LASER_ENERGY_LEVEL_INT)) {
                    device->setIntProperty(OB_PROP_LASER_ENERGY_LEVEL_INT,
                                           laser_energy);
                }
            } catch (const ob::Error&) {
            }
        }

        // enable_hardware_noise_removal_filter:=false
        const bool hw_noise = hardware::ParseBool(
            params, "enable_hardware_noise_removal_filter", false);
        try {
            if (PropertyWritable(device,
                                 OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL)) {
                device->setBoolProperty(
                    OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL, hw_noise);
            }
        } catch (const ob::Error&) {
        }

        // enable_noise_removal_filter:=true (soft speckle / cluster filter)
        const bool soft_noise = hardware::ParseBool(
            params, "enable_noise_removal_filter",
            hardware::ParseBool(params, "enable_soft_filter", true));
        try {
            if (PropertyWritable(device, OB_PROP_DEPTH_SOFT_FILTER_BOOL)) {
                device->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL,
                                        soft_noise);
            }
        } catch (const ob::Error&) {
        }

        const int min_diff = hardware::ParseInt(
            params, "noise_removal_filter_min_diff",
            hardware::ParseInt(params, "soft_filter_max_diff", 256));
        if (min_diff >= 0) {
            try {
                if (PropertyWritable(device, OB_PROP_DEPTH_MAX_DIFF_INT)) {
                    device->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, min_diff);
                }
            } catch (const ob::Error&) {
            }
        }

        const int max_size = hardware::ParseInt(
            params, "noise_removal_filter_max_size",
            hardware::ParseInt(params, "soft_filter_speckle_size", 80));
        if (max_size >= 0) {
            try {
                if (PropertyWritable(device, OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT)) {
                    device->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT,
                                           max_size);
                }
            } catch (const ob::Error&) {
            }
        }
    }

    /**
     * @brief Depth post-process filters — setupDepthPostProcessFilter
     * (OrbbecSDK_ROS2) + Gemini330PostFilterStrategy (OrbbecSDK_v2).
     */
    void SetupDepthFilters(const std::shared_ptr<ob::Device>& device) {
        depth_filters.clear();
        if (!device) {
            return;
        }
        try {
            auto depth_sensor = device->getSensor(OB_SENSOR_DEPTH);
            if (!depth_sensor) {
                return;
            }
            depth_filters = depth_sensor->createRecommendedFilters();
        } catch (const ob::Error&) {
            depth_filters.clear();
            return;
        }

        // gemini_330_series.launch.py defaults.
        const bool enable_spatial =
            hardware::ParseBool(params, "enable_spatial_filter", false);
        const bool enable_temporal =
            hardware::ParseBool(params, "enable_temporal_filter", false);
        const bool enable_hole =
            hardware::ParseBool(params, "enable_hole_filling_filter", false);
        const bool enable_threshold =
            hardware::ParseBool(params, "enable_threshold_filter", false);
        const bool enable_decimation =
            hardware::ParseBool(params, "enable_decimation_filter", false);
        const bool enable_hdr =
            hardware::ParseBool(params, "enable_hdr_merge", false);
        const bool enable_sequence =
            hardware::ParseBool(params, "enable_sequence_id_filter", false);
        const bool enable_disparity =
            hardware::ParseBool(params, "enable_disparity_to_depth", true);
        const bool enable_edge =
            hardware::ParseBool(params, "enable_edge_noise_removal_filter",
                                false);
        const bool enable_spatial_fast =
            hardware::ParseBool(params, "enable_spatial_fast_filter", false);
        const bool enable_spatial_mod =
            hardware::ParseBool(params, "enable_spatial_moderate_filter",
                                false);
        const bool enable_fp =
            hardware::ParseBool(params, "enable_false_positive_filter", false);

        const std::unordered_map<std::string, bool> toggles = {
            {"DecimationFilter", enable_decimation},
            {"HDRMerge", enable_hdr},
            {"SequenceIdFilter", enable_sequence},
            {"SpatialAdvancedFilter", enable_spatial},
            {"SpatialFastFilter", enable_spatial_fast},
            {"SpatialModerateFilter", enable_spatial_mod},
            {"TemporalFilter", enable_temporal},
            {"HoleFillingFilter", enable_hole},
            {"ThresholdFilter", enable_threshold},
            {"DisparityTransform", enable_disparity},
            {"EdgeNoiseRemovalFilter", enable_edge},
            {"FalsePositiveFilter", enable_fp},
        };

        for (const auto& filter : depth_filters) {
            if (!filter) {
                continue;
            }
            const std::string name = filter->type();
            const auto it = toggles.find(name);
            if (it == toggles.end()) {
                // Keep SDK recommended default for unknown filters.
                continue;
            }
            try {
                filter->enable(it->second);
            } catch (const ob::Error&) {
            }

            if (name == "SpatialAdvancedFilter" && it->second) {
                const double alpha =
                    hardware::ParseDouble(params, "spatial_filter_alpha", -1.0);
                const int magnitude = hardware::ParseInt(
                    params, "spatial_filter_magnitude", -1);
                const int radius =
                    hardware::ParseInt(params, "spatial_filter_radius", -1);
                const int diff = hardware::ParseInt(
                    params, "spatial_filter_diff_threshold", -1);
                if (alpha >= 0.0 && magnitude >= 0 && radius >= 0 &&
                    diff >= 0) {
                    try {
                        auto spatial = filter->as<ob::SpatialAdvancedFilter>();
                        if (spatial) {
                            OBSpatialAdvancedFilterParams p{};
                            p.alpha = static_cast<float>(alpha);
                            p.magnitude = static_cast<std::uint8_t>(magnitude);
                            p.radius = static_cast<std::uint16_t>(radius);
                            p.disp_diff = static_cast<std::uint16_t>(diff);
                            spatial->setFilterParams(p);
                        }
                    } catch (const ob::Error&) {
                    }
                }
            }
        }
    }

    std::shared_ptr<ob::Frame> ProcessDepthFilters(
        std::shared_ptr<ob::Frame> frame) {
        // Match OBCameraNode::processDepthFrameFilter.
        if (!frame || frame->getType() != OB_FRAME_DEPTH) {
            return frame;
        }
        for (const auto& filter : depth_filters) {
            if (!filter || !filter->isEnabled()) {
                continue;
            }
            try {
                auto out = filter->process(frame);
                if (!out) {
                    break;
                }
                frame = out;
            } catch (const ob::Error&) {
                break;
            }
        }
        return frame;
    }

    void StopPipeline() {
        if (pipeline) {
            try {
                pipeline->stop();
            } catch (...) {
            }
        }
        pipeline.reset();
        config.reset();
        point_cloud_filter.reset();
        align_filter.reset();
        depth_filters.clear();
    }

    void DispatchVideo(const VideoSubscription& sub,
                       const std::shared_ptr<ob::Frame>& frame) {
        if (!sub.callback || !frame) {
            return;
        }
        auto video = frame->as<ob::VideoFrame>();
        if (!video) {
            return;
        }
        OrbbecVideoFrame payload;
        payload.width = video->getWidth();
        payload.height = video->getHeight();
        payload.encoding =
            hardware::orbbec::EncodingForStreamKind(sub.stream);
        payload.timestamp_ms =
            static_cast<double>(video->getTimeStampUs()) / 1000.0;
        payload.frame_id = hardware::orbbec::DefaultFrameId(sub.stream);
        const auto* bytes = video->getData();
        payload.data.assign(bytes, bytes + video->getDataSize());

        try {
            auto profile = video->getStreamProfile();
            if (profile) {
                auto vsp = profile->as<ob::VideoStreamProfile>();
                if (vsp) {
                    const OBCameraIntrinsic intr = vsp->getIntrinsic();
                    const OBCameraDistortion dist = vsp->getDistortion();
                    float coeffs[5] = {dist.k1, dist.k2, dist.p1, dist.p2,
                                       dist.k3};
                    payload.camera_info = hardware::orbbec::MakeCameraInfo(
                        payload.width, payload.height, payload.frame_id,
                        intr.fx, intr.fy, intr.cx, intr.cy, coeffs, 5);
                    payload.has_camera_info = true;
                }
            }
        } catch (...) {
        }
        sub.callback(std::move(payload));
    }

    automsgs::msgs::sensor_msgs::PointCloud2 BuildPointCloud(
        const std::shared_ptr<ob::Frame>& points_frame,
        const std::string& frame_id, bool colored) const {
        automsgs::msgs::sensor_msgs::PointCloud2 cloud;
        cloud.mutable_header()->set_frame_id(frame_id);
        cloud.set_height(1);
        cloud.set_is_bigendian(false);
        cloud.set_is_dense(false);
        cloud.set_point_step(colored ? 16u : 12u);

        auto add_f32 = [&](const char* name, std::uint32_t offset) {
            auto* f = cloud.add_fields();
            f->set_name(name);
            f->set_offset(offset);
            f->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
            f->set_count(1);
        };
        add_f32("x", 0);
        add_f32("y", 4);
        add_f32("z", 8);
        if (colored) {
            add_f32("rgb", 12);
        }

        if (!points_frame) {
            cloud.set_width(0);
            cloud.set_row_step(0);
            return cloud;
        }

        const std::size_t stride =
            colored ? sizeof(OBColorPoint) : sizeof(OBPoint);
        const std::size_t count =
            points_frame->getDataSize() / stride;
        cloud.set_width(static_cast<std::uint32_t>(count));
        cloud.set_row_step(cloud.point_step() * cloud.width());

        std::vector<std::uint8_t> buffer(cloud.row_step(), 0);
        if (colored) {
            const auto* pts =
                reinterpret_cast<const OBColorPoint*>(points_frame->getData());
            for (std::size_t i = 0; i < count; ++i) {
                const std::size_t offset = i * 16;
                float* xyz = reinterpret_cast<float*>(buffer.data() + offset);
                xyz[0] = pts[i].x;
                xyz[1] = pts[i].y;
                xyz[2] = pts[i].z;
                const std::uint32_t rgb =
                    (static_cast<std::uint32_t>(
                         std::min(255.f, std::max(0.f, pts[i].r)))
                     << 16) |
                    (static_cast<std::uint32_t>(
                         std::min(255.f, std::max(0.f, pts[i].g)))
                     << 8) |
                    static_cast<std::uint32_t>(
                        std::min(255.f, std::max(0.f, pts[i].b)));
                std::memcpy(buffer.data() + offset + 12, &rgb, sizeof(rgb));
            }
        } else {
            const auto* pts =
                reinterpret_cast<const OBPoint*>(points_frame->getData());
            for (std::size_t i = 0; i < count; ++i) {
                const std::size_t offset = i * 12;
                float* xyz = reinterpret_cast<float*>(buffer.data() + offset);
                xyz[0] = pts[i].x;
                xyz[1] = pts[i].y;
                xyz[2] = pts[i].z;
            }
        }
        cloud.set_data(reinterpret_cast<const char*>(buffer.data()),
                       buffer.size());
        return cloud;
    }

    void CaptureLoop() {
        while (running.load()) {
            if (!pipeline) {
                break;
            }
            std::shared_ptr<ob::FrameSet> frameset;
            try {
                frameset = pipeline->waitForFrameset(100);
            } catch (const ob::Error& ex) {
                last_error =
                    std::string("Orbbec waitForFrameset failed: ") + ex.what();
                break;
            }
            if (!frameset) {
                continue;
            }

            std::vector<VideoSubscription> video_subs;
            std::vector<PointCloudSubscription> pointcloud_subs;
            {
                WriteLock lock(mutex);
                video_subs = video_subscriptions;
                pointcloud_subs = pointcloud_subscriptions;
            }

            for (const VideoSubscription& sub : video_subs) {
                std::shared_ptr<ob::Frame> frame;
                switch (sub.stream) {
                    case hardware::orbbec::StreamKind::kColor:
                        frame = frameset->getFrame(OB_FRAME_COLOR);
                        break;
                    case hardware::orbbec::StreamKind::kDepth:
                        frame = frameset->getFrame(OB_FRAME_DEPTH);
                        frame = ProcessDepthFilters(frame);
                        break;
                    case hardware::orbbec::StreamKind::kInfrared:
                        frame = frameset->getFrame(OB_FRAME_IR);
                        break;
                    case hardware::orbbec::StreamKind::kInfraredLeft:
                        frame = frameset->getFrame(OB_FRAME_IR_LEFT);
                        break;
                    case hardware::orbbec::StreamKind::kInfraredRight:
                        frame = frameset->getFrame(OB_FRAME_IR_RIGHT);
                        break;
                }
                DispatchVideo(sub, frame);
            }

            if (!pointcloud_subs.empty() && point_cloud_filter && align_filter) {
                try {
                    std::shared_ptr<ob::Frame> input = frameset;
                    try {
                        auto aligned = align_filter->process(frameset);
                        if (aligned) {
                            input = aligned;
                        }
                    } catch (...) {
                    }
                    const bool have_color =
                        frameset->getFrame(OB_FRAME_COLOR) != nullptr;
                    point_cloud_filter->setCreatePointFormat(
                        have_color ? OB_FORMAT_RGB_POINT : OB_FORMAT_POINT);
                    auto points = point_cloud_filter->process(input);
                    for (const PointCloudSubscription& sub : pointcloud_subs) {
                        if (!sub.callback) {
                            continue;
                        }
                        OrbbecPointCloudFrame payload;
                        auto depth = frameset->getFrame(OB_FRAME_DEPTH);
                        payload.timestamp_ms =
                            depth ? static_cast<double>(depth->getTimeStampUs()) /
                                        1000.0
                                  : 0.0;
                        payload.frame_id = "camera_depth_optical_frame";
                        payload.cloud = BuildPointCloud(
                            points, payload.frame_id, have_color);
                        sub.callback(std::move(payload));
                    }
                } catch (const ob::Error& ex) {
                    last_error =
                        std::string("Orbbec point cloud failed: ") + ex.what();
                }
            }
        }
    }
#endif
};

OrbbecDeviceHub::OrbbecDeviceHub(const hardware::DriverParams& params)
    : impl_(std::make_unique<Impl>()) {
    impl_->params = params;
    impl_->device_key = DeviceKey(params);
}

OrbbecDeviceHub::~OrbbecDeviceHub() { Stop(); }

std::shared_ptr<OrbbecDeviceHub> OrbbecDeviceHub::Acquire(
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
    auto hub = std::shared_ptr<OrbbecDeviceHub>(new OrbbecDeviceHub(params));
    g_pool[key] = hub;
    return hub;
}

std::uint64_t OrbbecDeviceHub::SubscribeVideo(
    const hardware::orbbec::StreamKind stream, const int width, const int height,
    const int fps, OrbbecVideoCallback callback) {
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

std::uint64_t OrbbecDeviceHub::SubscribePointCloud(
    const int width, const int height, const int fps,
    OrbbecPointCloudCallback callback) {
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

void OrbbecDeviceHub::Unsubscribe(const std::uint64_t subscription_id) {
    const bool restart = impl_->running.load();
    if (restart) {
        Stop();
    }
    {
        WriteLock lock(impl_->mutex);
        auto& videos = impl_->video_subscriptions;
        videos.erase(std::remove_if(videos.begin(), videos.end(),
                                    [subscription_id](const auto& s) {
                                        return s.id == subscription_id;
                                    }),
                     videos.end());
        auto& clouds = impl_->pointcloud_subscriptions;
        clouds.erase(std::remove_if(clouds.begin(), clouds.end(),
                                    [subscription_id](const auto& s) {
                                        return s.id == subscription_id;
                                    }),
                     clouds.end());
    }
    if (restart && (!impl_->video_subscriptions.empty() ||
                    !impl_->pointcloud_subscriptions.empty())) {
        Start();
    }
}

bool OrbbecDeviceHub::Start() {
    if (impl_->running.exchange(true)) {
        return true;
    }
    if (!impl_->StartPipeline()) {
        impl_->running = false;
        return false;
    }
    impl_->worker = std::thread([this]() { impl_->CaptureLoop(); });
    return true;
}

void OrbbecDeviceHub::Stop() {
    if (!impl_->running.exchange(false)) {
        return;
    }
    impl_->StopPipeline();
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
}

bool OrbbecDeviceHub::IsRunning() const { return impl_->running.load(); }

const std::string& OrbbecDeviceHub::last_error() const {
    return impl_->last_error;
}

}  // namespace io

namespace hardware {
namespace orbbec {
namespace {

std::string ToLowerCopy(std::string text) {
    std::transform(
        text.begin(), text.end(), text.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

}  // namespace

StreamKind ParseStreamKind(const std::string& text,
                           const StreamKind default_kind) {
    const std::string value = ToLowerCopy(text);
    if (value.empty() || value == "color" || value == "rgb") {
        return StreamKind::kColor;
    }
    if (value == "depth" || value == "z16") {
        return StreamKind::kDepth;
    }
    // Gemini 330 series: stereo IR topics are left_ir / right_ir.
    if (value == "left_ir" || value == "ir1" || value == "infrared1" ||
        value == "ir" || value == "infrared") {
        return StreamKind::kInfraredLeft;
    }
    if (value == "right_ir" || value == "ir2" || value == "infrared2") {
        return StreamKind::kInfraredRight;
    }
    if (value == "ir0" || value == "mono_ir") {
        return StreamKind::kInfrared;
    }
    return default_kind;
}

std::string EncodingForStreamKind(const StreamKind kind) {
    switch (kind) {
        case StreamKind::kColor:
            return "rgb8";
        case StreamKind::kDepth:
            return "16UC1";
        case StreamKind::kInfrared:
        case StreamKind::kInfraredLeft:
        case StreamKind::kInfraredRight:
            return "mono8";
    }
    return "rgb8";
}

std::string DefaultFrameId(const StreamKind kind) {
    switch (kind) {
        case StreamKind::kColor:
            return "camera_color_optical_frame";
        case StreamKind::kDepth:
            return "camera_depth_optical_frame";
        case StreamKind::kInfrared:
            return "camera_ir_optical_frame";
        case StreamKind::kInfraredLeft:
            return "camera_left_ir_optical_frame";
        case StreamKind::kInfraredRight:
            return "camera_right_ir_optical_frame";
    }
    return "camera_link";
}

}  // namespace orbbec
}  // namespace hardware
}  // namespace autodriver
