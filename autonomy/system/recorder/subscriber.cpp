/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file subscriber.cpp
 * @brief Bypass RawMessage readers: BEST_EFFORT, schema-once, payload copy.
 */

#include "autonomy/system/recorder/subscriber.hpp"

#include <algorithm>
#include <memory>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/node_channel_impl.hpp"
#include "autolink/proto/qos_profile.pb.h"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autonomy/system/recorder/clock.hpp"
#include "autonomy/system/recorder/signals.hpp"

namespace autonomy {
namespace system {
namespace recorder {
namespace {

bool IsBulky(const std::string& name) {
    return name.find("image") != std::string::npos ||
           name.find("camera") != std::string::npos ||
           name.find("points") != std::string::npos ||
           name.find("lidar_bev") != std::string::npos ||
           name.find("policy_raw") != std::string::npos ||
           name.find("/grid") != std::string::npos ||
           name.find("costmap") != std::string::npos ||
           name.find("occupancy") != std::string::npos ||
           name == "/map";
}

int KeepLastDepth(const proto::ChannelSpec& spec, bool full_dump) {
    if (spec.depth() > 0) {
        return spec.depth();
    }
    if (spec.name().find("tf_static") != std::string::npos) {
        return 1;
    }
    if (IsBulky(spec.name())) {
        return full_dump ? 16 : 2;
    }
    return full_dump ? 64 : 32;
}

int PendingQueue(const proto::ChannelSpec& spec, bool full_dump) {
    if (spec.queue() > 0) {
        return spec.queue();
    }
    return std::max(8, KeepLastDepth(spec, full_dump) * 2);
}

}  // namespace

bool BypassSubscriber::Attach(const std::shared_ptr<autolink::Node>& node,
                              const proto::RecorderOptions& options,
                              Callback callback) {
    if (!node || !callback) {
        return false;
    }
    Reset();
    auto* channels =
        autolink::service_discovery::TopologyManager::Instance()
            ->channel_manager()
            .get();
    for (const auto& spec : options.channels()) {
        if (spec.name().empty()) {
            continue;
        }
        autolink::ReaderConfig cfg;
        cfg.channel_name = spec.name();
        cfg.pending_queue_size = PendingQueue(spec, options.full_dump());
        cfg.qos_profile.set_history(
            autolink::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
        cfg.qos_profile.set_depth(KeepLastDepth(spec, options.full_dump()));
        cfg.qos_profile.set_reliability(
            autolink::proto::QosReliabilityPolicy::RELIABILITY_BEST_EFFORT);
        cfg.qos_profile.set_durability(
            autolink::proto::QosDurabilityPolicy::DURABILITY_VOLATILE);

        const std::string name = spec.name();
        std::string msg_type = spec.message_type();
        std::string proto_desc;
        if (channels != nullptr) {
            if (msg_type.empty()) {
                channels->GetMsgType(name, &msg_type);
            }
            channels->GetProtoDesc(name, &proto_desc);
        }
        const uint16_t flags = SampleFlagsFor(name, options);
        const auto source = options.time_source() == proto::TIME_SOURCE_UNSPECIFIED
                                ? proto::TIME_SOURCE_SOFTWARE
                                : options.time_source();
        auto schema_once = std::make_shared<std::string>(std::move(proto_desc));
        auto reader = node->CreateReader<autolink::message::RawMessage>(
            cfg, [callback, name, msg_type, flags, source, schema_once](
                     const std::shared_ptr<autolink::message::RawMessage>&
                         msg) mutable {
                if (!msg) {
                    return;
                }
                Sample sample;
                sample.channel = name;
                sample.message_type = msg_type;
                sample.flags = flags;
                sample.payload = msg->message;
                if (schema_once && !schema_once->empty()) {
                    sample.proto_desc = std::move(*schema_once);
                    schema_once->clear();
                    schema_once.reset();
                }
                if (msg->timestamp != 0) {
                    sample.clock = DualClockFromMono(
                        static_cast<int64_t>(msg->timestamp), source);
                } else {
                    sample.clock = NowDualClock(source);
                }
                callback(std::move(sample));
            });
        if (reader) {
            readers_.push_back(std::move(reader));
        }
    }
    return !readers_.empty();
}

void BypassSubscriber::Reset() { readers_.clear(); }

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
