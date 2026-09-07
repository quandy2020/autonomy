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

    bool ConfigurePipeline(const std::shared_ptr<ob::Device>& device) {
        config = std::make_shared<ob::Config>();
        int color_w = 640;
        int color_h = 480;
        int color_fps = 30;
        int depth_w = 640;
        int depth_h = 480;
        int depth_fps = 30;
        int ir_w = 640;
        int ir_h = 480;
        int ir_fps = 30;

        for (const VideoSubscription& sub : video_subscriptions) {
            if (sub.stream == hardware::orbbec::StreamKind::kColor) {
                color_w = sub.width;
                color_h = sub.height;
                color_fps = sub.fps;
            } else if (sub.stream == hardware::orbbec::StreamKind::kDepth) {
                depth_w = sub.width;
                depth_h = sub.height;
                depth_fps = sub.fps;
            } else if (sub.stream == hardware::orbbec::StreamKind::kInfrared) {
                ir_w = sub.width;
                ir_h = sub.height;
                ir_fps = sub.fps;
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

        try {
            if (NeedsColor()) {
                config->enableVideoStream(OB_STREAM_COLOR, color_w, color_h,
                                          color_fps, OB_FORMAT_RGB);
            }
            if (NeedsDepth()) {
                config->enableVideoStream(OB_STREAM_DEPTH, depth_w, depth_h,
                                          depth_fps, OB_FORMAT_Y16);
            }
            if (NeedsInfrared()) {
                config->enableVideoStream(OB_STREAM_IR, ir_w, ir_h, ir_fps,
                                          OB_FORMAT_Y8);
            }
            if (NeedsColor() && NeedsDepth()) {
                config->setFrameAggregateOutputMode(
                    OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
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
                        break;
                    case hardware::orbbec::StreamKind::kInfrared:
                        frame = frameset->getFrame(OB_FRAME_IR);
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
    if (value == "infrared" || value == "ir" || value == "ir1") {
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
            return "camera_infra_optical_frame";
    }
    return "camera_link";
}

}  // namespace orbbec
}  // namespace hardware
}  // namespace autodriver
