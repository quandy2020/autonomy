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
 * @file publisher.cpp
 * @brief Autolink bridge that publishes sensor samples to configured channels
 *        (implementation).
 */

#include "autodriver/bridge/publisher.hpp"

#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "autodriver/bridge/channels.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_traits.hpp"
#include "autolink/autolink.hpp"
#include "autolink/base/rw_lock_guard.hpp"
#include "autolink/common/log.hpp"
#include "autolink/node/writer.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

namespace autodriver {
namespace bridge {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using ReadLock = autolink::base::ReadLockGuard<AtomicRWLock>;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

// Default Autolink writer queue depth for sensor publishers.
constexpr int kWriterDepth = 10;

/**
 * @brief Builds RoleAttributes with keep-last QoS for a channel name.
 * @param[in] channel Autolink channel name for the writer.
 * @return RoleAttributes with HISTORY_KEEP_LAST QoS.
 */
autolink::proto::RoleAttributes WriterAttr(std::string_view channel) {
    autolink::proto::RoleAttributes attr;
    attr.set_channel_name(std::string(channel));
    auto* qos = attr.mutable_qos_profile();
    qos->set_history(autolink::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    qos->set_depth(kWriterDepth);
    return attr;
}

/**
 * @brief Returns configured channels or a default derived from sensor metadata.
 * @param[in] sensor Sensor config (may list channels).
 * @param[in] type Sensor type used for the default channel.
 * @return Configured channels, or a single default channel name.
 */
std::vector<std::string> ResolvePublishChannels(
    const Config::Sensor& sensor, SensorType type) {
    if (!sensor.channels.empty()) {
        return sensor.channels;
    }
    const std::string stream = hardware::GetString(sensor.params, "stream");
    return {ResolveChannel("", sensor.id, type, stream)};
}

}  // namespace

Publisher::Publisher(std::string node_name, std::size_t async_queue_capacity)
    : node_name_(std::move(node_name)),
      async_capacity_(async_queue_capacity == 0 ? 1 : async_queue_capacity) {}

Publisher::~Publisher() {
    StopPublishThread();
    WriteLock lock(lock_);
    writers_.clear();
    diagnostics_writer_.reset();
    node_.reset();
}

bool Publisher::Initialize() {
    if (node_) {
        return true;
    }
    node_ = autolink::CreateNode(node_name_);
    if (!node_) {
        AERROR << "CreateNode failed: " << node_name_;
        return false;
    }
    EnsurePublishThread();
    return true;
}

void Publisher::EnsurePublishThread() {
    if (publish_running_.exchange(true)) {
        return;
    }
    publish_thread_ = std::thread([this]() { PublishLoop(); });
}

void Publisher::StopPublishThread() {
    if (!publish_running_.exchange(false)) {
        return;
    }
    queue_cv_.notify_all();
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.clear();
}

void Publisher::PublishLoop() {
    while (publish_running_.load()) {
        PendingSample pending;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() {
                return !queue_.empty() || !publish_running_.load();
            });
            if (queue_.empty()) {
                continue;
            }
            pending = std::move(queue_.front());
            queue_.pop_front();
        }
        if (pending.write && pending.sample) {
            pending.write(pending.sample);
        }
    }
}

bool Publisher::HandleSensorAttach(const Config::Sensor& sensor, SensorType type) {
    if (!Initialize()) {
        return false;
    }
    switch (type) {
        case SensorType::kImu:
            return OpenTypedWriter<SensorType::kImu>(sensor);
        case SensorType::kGps:
            return OpenTypedWriter<SensorType::kGps>(sensor);
        case SensorType::kCamera:
            return OpenCameraWriters(sensor);
        case SensorType::kLidar2d:
            return OpenTypedWriter<SensorType::kLidar2d>(sensor);
        case SensorType::kLidar3d:
            return OpenTypedWriter<SensorType::kLidar3d>(sensor);
        case SensorType::kRangeFinder:
            return OpenTypedWriter<SensorType::kRangeFinder>(sensor);
        case SensorType::kWheelOdometry:
            return OpenTypedWriter<SensorType::kWheelOdometry>(sensor);
        case SensorType::kRadar:
            return OpenTypedWriter<SensorType::kRadar>(sensor);
        case SensorType::kMicrophone:
            return OpenTypedWriter<SensorType::kMicrophone>(sensor);
    }
    AERROR << "unknown sensor type for " << sensor.id;
    return false;
}

void Publisher::HandleSensorDetach(const SensorId& id) {
    WriteLock lock(lock_);
    writers_.erase(id);
}

void Publisher::HandleSensorSample(std::shared_ptr<SensorSample> sample) {
    if (!sample) {
        return;
    }
    // Resolve WriteFn under a short read lock; enqueue for the publish thread.
    WriteFn write;
    {
        ReadLock lock(lock_);
        const auto it = writers_.find(sample->id());
        if (it == writers_.end()) {
            return;
        }
        write = it->second;
    }
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (queue_.size() >= async_capacity_) {
            queue_.pop_front();
        }
        queue_.push_back(PendingSample{std::move(write), std::move(sample)});
    }
    queue_cv_.notify_one();
}

void Publisher::SetDiagnosticsChannel(std::string channel) {
    if (!channel.empty()) {
        diagnostics_channel_ = std::move(channel);
    }
}

void Publisher::HandleDiagnostic(
    const diagnostics::DiagnosticSnapshot& snapshot) {
    if (!Initialize()) {
        return;
    }
    {
        WriteLock lock(lock_);
        if (!diagnostics_writer_) {
            diagnostics_writer_ = node_->CreateWriter<
                automsgs::msgs::diagnostic_msgs::DiagnosticArray>(
                WriterAttr(diagnostics_channel_));
            if (!diagnostics_writer_) {
                AERROR << "CreateWriter failed on " << diagnostics_channel_;
                return;
            }
        }
    }
    auto array =
        std::make_shared<automsgs::msgs::diagnostic_msgs::DiagnosticArray>();
    auto* status = array->add_status();
    switch (snapshot.status) {
        case diagnostics::DeviceStatus::kOk:
            status->set_level(
                automsgs::msgs::diagnostic_msgs::DiagnosticStatus::OK);
            break;
        case diagnostics::DeviceStatus::kDisconnected:
            status->set_level(
                automsgs::msgs::diagnostic_msgs::DiagnosticStatus::STALE);
            break;
        case diagnostics::DeviceStatus::kError:
            status->set_level(
                automsgs::msgs::diagnostic_msgs::DiagnosticStatus::ERROR);
            break;
    }
    status->set_name(snapshot.id);
    status->set_hardware_id(snapshot.id);
    status->set_message(snapshot.message);
    if (snapshot.sample_count > 0) {
        auto* kv = status->add_values();
        kv->set_key("sample_count");
        kv->set_value(std::to_string(snapshot.sample_count));
    }
    diagnostics_writer_->Write(array);
}

bool Publisher::OpenCameraWriters(const Config::Sensor& sensor) {
    using Traits = SensorTraits<SensorType::kCamera>;
    using Sample = CameraFrame;
    using Message = typename Traits::Message;
    const std::vector<std::string> channels =
        ResolvePublishChannels(sensor, SensorType::kCamera);
    if (channels.empty()) {
        AERROR << "no publish channel for " << sensor.id;
        return false;
    }

    using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
    using CameraInfoWriter = std::shared_ptr<autolink::Writer<CameraInfo>>;

    struct ChannelWriters {
        std::shared_ptr<autolink::Writer<Message>> image;
        CameraInfoWriter camera_info;
        std::string camera_info_channel;
    };

    std::vector<ChannelWriters> channel_writers;
    channel_writers.reserve(channels.size());
    std::string joined;
    for (const std::string& channel : channels) {
        auto image_writer = node_->CreateWriter<Message>(WriterAttr(channel));
        if (!image_writer) {
            AERROR << "CreateWriter failed on " << channel;
            return false;
        }
        ChannelWriters writers;
        writers.image = std::move(image_writer);
        writers.camera_info_channel = CameraInfoChannelForImage(channel);
        writers.camera_info =
            node_->CreateWriter<CameraInfo>(
                WriterAttr(writers.camera_info_channel));
        if (!writers.camera_info) {
            AERROR << "CreateWriter failed on "
                   << writers.camera_info_channel;
            return false;
        }
        channel_writers.push_back(std::move(writers));
        if (!joined.empty()) {
            joined += ", ";
        }
        joined += channel;
        if (!channel_writers.back().camera_info_channel.empty()) {
            joined += " + " + channel_writers.back().camera_info_channel;
        }
    }

    WriteFn fanout =
        [channel_writers = std::move(channel_writers)](
            const std::shared_ptr<SensorSample>& sample) {
            if (!sample) {
                return;
            }
            auto& data = static_cast<Sample&>(*sample);
            const auto image =
                std::shared_ptr<Message>(sample, &data.msg);
            for (const ChannelWriters& writers : channel_writers) {
                writers.image->Write(image);
                if (writers.camera_info && data.has_camera_info) {
                    auto info = std::make_shared<CameraInfo>(data.camera_info);
                    info->mutable_header()->mutable_stamp()->CopyFrom(
                        data.msg.header().stamp());
                    if (!data.frame_id.empty()) {
                        info->mutable_header()->set_frame_id(data.frame_id);
                    }
                    writers.camera_info->Write(info);
                }
            }
        };

    WriteLock lock(lock_);
    writers_[sensor.id] = std::move(fanout);
    AINFO << "writer " << sensor.id << " -> " << joined;
    return true;
}

template <SensorType kType>
bool Publisher::OpenTypedWriter(const Config::Sensor& sensor) {
    using Traits = SensorTraits<kType>;
    using Sample = typename Traits::Sample;
    using Message = typename Traits::Message;
    const std::vector<std::string> channels =
        ResolvePublishChannels(sensor, kType);
    if (channels.empty()) {
        AERROR << "no publish channel for " << sensor.id;
        return false;
    }

    std::vector<WriteFn> writers;
    writers.reserve(channels.size());
    std::string joined;
    for (const std::string& channel : channels) {
        auto writer = node_->CreateWriter<Message>(WriterAttr(channel));
        if (!writer) {
            AERROR << "CreateWriter failed on " << channel;
            return false;
        }
        writers.push_back(
            [writer](const std::shared_ptr<SensorSample>& sample) {
                if (!sample) {
                    return;
                }
                // Skip non-matching polymorphic samples (e.g. LidarPacketScan
                // on a PointCloud2 writer).
                auto* data = dynamic_cast<Sample*>(sample.get());
                if (data == nullptr) {
                    return;
                }
                writer->Write(std::shared_ptr<Message>(sample, &data->msg));
            });
        if (!joined.empty()) {
            joined += ", ";
        }
        joined += channel;
    }

    WriteFn fanout = [writers = std::move(writers)](
                         const std::shared_ptr<SensorSample>& sample) {
        for (const WriteFn& write : writers) {
            write(sample);
        }
    };
    WriteLock lock(lock_);
    writers_[sensor.id] = std::move(fanout);
    AINFO << "writer " << sensor.id << " -> " << joined;
    return true;
}

}  // namespace bridge
}  // namespace autodriver
