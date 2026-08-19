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

#include "autodriver/bridge/publisher.hpp"

#include <string_view>
#include <utility>
#include <vector>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_traits.hpp"
#include "autolink/autolink.hpp"
#include "autolink/base/rw_lock_guard.hpp"
#include "autolink/common/log.hpp"
#include "autolink/node/writer.hpp"
#include "autolink/proto/role_attributes.pb.h"

namespace autodriver {
namespace bridge {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using ReadLock = autolink::base::ReadLockGuard<AtomicRWLock>;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

constexpr int kWriterDepth = 10;

autolink::proto::RoleAttributes WriterAttr(std::string_view channel) {
    autolink::proto::RoleAttributes attr;
    attr.set_channel_name(std::string(channel));
    auto* qos = attr.mutable_qos_profile();
    qos->set_history(autolink::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    qos->set_depth(kWriterDepth);
    return attr;
}

std::vector<std::string> ResolvePublishChannels(
    const Config::Sensor& sensor, SensorType type) {
    if (!sensor.channels.empty()) {
        return sensor.channels;
    }
    const std::string stream = hardware::GetString(sensor.params, "stream");
    return {ResolveChannel("", sensor.id, type, stream)};
}

}  // namespace

Publisher::Publisher(std::string node_name) : node_name_(std::move(node_name)) {}

Publisher::~Publisher() {
    WriteLock lock(lock_);
    writers_.clear();
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
    return true;
}

bool Publisher::OnAttach(const Config::Sensor& sensor, SensorType type) {
    if (!Initialize()) {
        return false;
    }
    switch (type) {
        case SensorType::kImu:
            return OpenWriter<SensorType::kImu>(sensor);
        case SensorType::kGps:
            return OpenWriter<SensorType::kGps>(sensor);
        case SensorType::kCamera:
            return OpenWriter<SensorType::kCamera>(sensor);
        case SensorType::kLidar2d:
            return OpenWriter<SensorType::kLidar2d>(sensor);
        case SensorType::kLidar3d:
            return OpenWriter<SensorType::kLidar3d>(sensor);
        case SensorType::kRangeFinder:
            return OpenWriter<SensorType::kRangeFinder>(sensor);
        case SensorType::kWheelOdometry:
            return OpenWriter<SensorType::kWheelOdometry>(sensor);
    }
    AERROR << "unknown sensor type for " << sensor.id;
    return false;
}

void Publisher::OnDetach(const SensorId& id) {
    WriteLock lock(lock_);
    writers_.erase(id);
}

void Publisher::OnSample(std::shared_ptr<SensorSample> sample) {
    if (!sample) {
        return;
    }
    WriteFn write;
    {
        ReadLock lock(lock_);
        const auto it = writers_.find(sample->id());
        if (it == writers_.end()) {
            return;
        }
        write = it->second;
    }
    write(sample);
}

template <SensorType kType>
bool Publisher::OpenWriter(const Config::Sensor& sensor) {
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
                auto& data = static_cast<Sample&>(*sample);
                writer->Write(std::shared_ptr<Message>(sample, &data.msg));
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
