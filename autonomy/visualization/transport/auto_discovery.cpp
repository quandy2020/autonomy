/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/visualization/transport/auto_discovery.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/map_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/proto/sensor_msgs.pb.h"
#include "autonomy/commsgs/proto/visualization_msgs.pb.h"
#include "autonomy/visualization/core/channel.hpp"

using namespace autonomy::visualization;

namespace autonomy {
namespace visualization {
namespace transport {

AutoDiscovery::AutoDiscovery(std::shared_ptr<FoxgloveBridge> bridge)
    : bridge_(bridge) {
    node_ = autolink::CreateNode("visualization_auto_discovery");
}

AutoDiscovery::~AutoDiscovery() {
    Stop();
}

bool AutoDiscovery::Start() {
    // 注册 topology 变化监听器
    auto topology = autolink::service_discovery::TopologyManager::Instance();
    if (topology) {
        topology_connection_ = topology->AddChangeListener(
            [this](const autolink::proto::ChangeMsg& change_msg) {
                this->OnTopologyChange(change_msg);
            });
    }
    // 启动发现线程
    discovery_thread_ = std::thread([this] { this->DiscoveryLoop(); });
    return true;
}

void AutoDiscovery::Stop() {
    if (!running_.exchange(false)) {
        return;
    }

    // 断开 topology 监听器
    if (topology_connection_.has_value()) {
        auto topology =
            autolink::service_discovery::TopologyManager::Instance();
        if (topology) {
            topology->RemoveChangeListener(topology_connection_.value());
        }
        topology_connection_.reset();
    }

    // 等待发现线程结束
    if (discovery_thread_.joinable()) {
        discovery_thread_.join();
    }

    // 清理所有订阅
    {
        std::lock_guard<std::mutex> lock(subscribed_mutex_);
        autolink_readers_.clear();
        autolink_channels_.clear();
        subscribed_topics_.clear();
    }

    AINFO << "AutoDiscovery stopped";
}

void AutoDiscovery::DiscoveryLoop() {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // 立即执行一次发现
    AINFO << "Starting first discovery...";
    DiscoverAndSubscribe();

    while (running_.load()) {
        // 每 5 秒扫描一次（处理可能遗漏的 channels，但主要依赖
        // OnTopologyChange）
        std::this_thread::sleep_for(std::chrono::seconds(5));
        DiscoverAndSubscribe();
    }
}

void AutoDiscovery::DiscoverAndSubscribe() {
    auto topology = autolink::service_discovery::TopologyManager::Instance();
    if (!topology) {
        AERROR << "TopologyManager instance is null";
        return;
    }
    AINFO << "TopologyManager instance obtained";

    auto channel_manager = topology->channel_manager();
    if (!channel_manager) {
        AERROR << "ChannelManager is null";
        return;
    }
    AINFO << "ChannelManager obtained";

    // 直接获取所有 writers（更可靠的方式，因为消息类型在 writer 的
    // RoleAttributes 中） 注意：如果 writer 的 message_type
    // 为空（可能在注册时还未设置），则跳过 这些 writer 应该会在
    // OnTopologyChange 回调中被处理
    autolink::service_discovery::ChannelManager::RoleAttrVec writers;
    channel_manager->GetWriters(&writers);

    AINFO << "Discovered " << writers.size() << " writers";

    int skipped_empty_type = 0;
    for (const auto& writer_attr : writers) {
        const std::string& channel_name = writer_attr.channel_name();
        const std::string& message_type = writer_attr.message_type();

        // 检查是否已经订阅
        {
            std::lock_guard<std::mutex> lock(subscribed_mutex_);
            if (subscribed_topics_.find(channel_name) !=
                subscribed_topics_.end()) {
                continue;
            }
        }

        // 如果消息类型为空，跳过（这些 writer 可能还在初始化，会在
        // OnTopologyChange 中处理）
        if (message_type.empty()) {
            skipped_empty_type++;
            continue;
        }

        AINFO << "Trying to subscribe channel: " << channel_name
              << " (type: " << message_type << ")";

        // 尝试订阅（SubscribeTopicTyped 内部已经会添加到 subscribed_topics_）
        if (SubscribeTopicInternal(channel_name, message_type)) {
            AINFO << "Auto-subscribed topic: " << channel_name
                  << " (type: " << message_type << ")";
        } else {
            AWARN << "Failed to subscribe topic: " << channel_name
                  << " (type: " << message_type << ")";
        }
    }

    if (skipped_empty_type > 0) {
        AINFO << "Skipped " << skipped_empty_type
              << " writers with empty message type (will be handled by "
                 "OnTopologyChange)";
    }
}

bool AutoDiscovery::SubscribeTopic(const std::string& topic_name,
                                   const std::string& message_type) {
    return SubscribeTopicInternal(topic_name, message_type);
}

bool AutoDiscovery::SubscribeTopicInternal(const std::string& topic_name,
                                           const std::string& message_type) {
    if (!bridge_) {
        AERROR << "Bridge is null";
        return false;
    }

    // 如果提供了消息类型，直接使用
    if (!message_type.empty()) {
        return SubscribeByMessageType(topic_name, message_type);
    }

    // 否则从 topology 获取
    auto topology = autolink::service_discovery::TopologyManager::Instance();
    if (!topology) {
        return false;
    }

    auto channel_manager = topology->channel_manager();
    if (!channel_manager) {
        return false;
    }

    std::string msg_type;
    channel_manager->GetMsgType(topic_name, &msg_type);
    if (msg_type.empty()) {
        AWARN << "Cannot get message type for topic: " << topic_name;
        return false;
    }

    return SubscribeByMessageType(topic_name, msg_type);
}

bool AutoDiscovery::SubscribeByMessageType(const std::string& topic_name,
                                           const std::string& message_type) {
    if (!node_) {
        AERROR << "Autolink node is not initialized";
        return false;
    }

    // 检查是否已经订阅
    {
        std::lock_guard<std::mutex> lock(subscribed_mutex_);
        if (autolink_readers_.find(topic_name) != autolink_readers_.end()) {
            AINFO << "Topic already subscribed: " << topic_name;
            return true;
        }
    }

    // 将消息类型字符串转换为小写，支持多种格式
    std::string type_lower = message_type;
    std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // 提取短名称（支持 "autonomy.commsgs.proto.sensor_msgs.LaserScan" 或
    // "LaserScan" 格式）
    std::string short_type = type_lower;
    size_t last_dot = type_lower.find_last_of('.');
    if (last_dot != std::string::npos && last_dot < type_lower.length() - 1) {
        short_type = type_lower.substr(last_dot + 1);
    }

    AINFO << "Message type processing - original: " << message_type
          << ", lower: " << type_lower << ", short: " << short_type;

    // 使用类型萃取来订阅消息
    if (SubscribeByTypeTraits(topic_name, type_lower, short_type)) {
        return true;
    }

    AWARN << "No matching message type handler for: " << message_type
          << " (short: " << short_type << ")";
    return false;
}

bool AutoDiscovery::SubscribeByTypeTraits(const std::string& topic_name,
                                          const std::string& type_lower,
                                          const std::string& short_type) {
    // 类型萃取：根据命名空间和短名称匹配消息类型
    // sensor_msgs
    if (type_lower.find("sensor_msgs") != std::string::npos) {
        if (short_type == "laserscan") {
            return SubscribeTopicTyped<commsgs::proto::sensor_msgs::LaserScan>(
                topic_name);
        }
        if (short_type == "pointcloud2") {
            return SubscribeTopicTyped<
                commsgs::proto::sensor_msgs::PointCloud2>(topic_name);
        }
        if (short_type == "pointcloud") {
            return SubscribeTopicTyped<commsgs::proto::sensor_msgs::PointCloud>(
                topic_name);
        }
        if (short_type == "imu") {
            return SubscribeTopicTyped<commsgs::proto::sensor_msgs::Imu>(
                topic_name);
        }
        if (short_type == "range") {
            return SubscribeTopicTyped<commsgs::proto::sensor_msgs::Range>(
                topic_name);
        }
        if (short_type == "image") {
            return SubscribeTopicTyped<commsgs::proto::sensor_msgs::Image>(
                topic_name);
        }
        if (short_type == "compressedimage") {
            return SubscribeTopicTyped<
                commsgs::proto::sensor_msgs::CompressedImage>(topic_name);
        }
        // 如果匹配了 sensor_msgs 但没有匹配到具体类型，返回 false
        return false;
    }

    // planning_msgs
    if (type_lower.find("planning_msgs") != std::string::npos) {
        if (short_type == "path") {
            return SubscribeTopicTyped<commsgs::proto::planning_msgs::Path>(
                topic_name);
        }
        if (short_type == "odometry") {
            return SubscribeTopicTyped<commsgs::proto::planning_msgs::Odometry>(
                topic_name);
        }
        return false;
    }

    // map_msgs
    if (type_lower.find("map_msgs") != std::string::npos) {
        if (short_type == "occupancygrid") {
            return SubscribeTopicTyped<commsgs::proto::map_msgs::OccupancyGrid>(
                topic_name);
        }
        return false;
    }

    // geometry_msgs
    if (type_lower.find("geometry_msgs") != std::string::npos) {
        if (short_type == "posestamped") {
            return SubscribeTopicTyped<
                commsgs::proto::geometry_msgs::PoseStamped>(topic_name);
        }
        if (short_type == "posearray") {
            return SubscribeTopicTyped<
                commsgs::proto::geometry_msgs::PoseArray>(topic_name);
        }
        if (short_type == "transformstamped") {
            return SubscribeTopicTyped<
                commsgs::proto::geometry_msgs::TransformStamped>(topic_name);
        }
        return false;
    }

    // visualization_msgs
    if (type_lower.find("visualization_msgs") != std::string::npos) {
        if (short_type == "marker") {
            return SubscribeTopicTyped<
                commsgs::proto::visualization_msgs::Marker>(topic_name);
        }
        if (short_type == "markerarray") {
            return SubscribeTopicTyped<
                commsgs::proto::visualization_msgs::MarkerArray>(topic_name);
        }
        return false;
    }

    // 也支持直接通过 short_type 匹配（向后兼容）
    if (short_type == "laserscan") {
        return SubscribeTopicTyped<commsgs::proto::sensor_msgs::LaserScan>(
            topic_name);
    }
    if (short_type == "occupancygrid") {
        return SubscribeTopicTyped<commsgs::proto::map_msgs::OccupancyGrid>(
            topic_name);
    }

    return false;
}

void AutoDiscovery::OnTopologyChange(
    const autolink::proto::ChangeMsg& change_msg) {
    // 当 topology 发生变化时，立即触发发现（只处理新增的 writer）
    if (change_msg.role_type() == autolink::proto::RoleType::ROLE_WRITER &&
        change_msg.operate_type() == autolink::proto::OperateType::OPT_JOIN) {
        const std::string& channel_name = change_msg.role_attr().channel_name();
        const std::string& message_type = change_msg.role_attr().message_type();

        AINFO << "Topology change detected: new writer for channel="
              << channel_name << ", message_type=" << message_type;

        {
            std::lock_guard<std::mutex> lock(subscribed_mutex_);
            if (subscribed_topics_.find(channel_name) !=
                subscribed_topics_.end()) {
                AINFO << "Topic " << channel_name
                      << " already subscribed via topology change, skipping.";
                return;  // 已经订阅，跳过
            }
        }

        // 如果消息类型为空，跳过（等待后续查询）
        if (message_type.empty()) {
            AWARN << "Message type is empty in topology change for channel: "
                  << channel_name;
            return;
        }

        // 异步处理，避免在回调中阻塞（直接使用回调中的 message_type）
        autolink::Async([this, channel_name, message_type]() {
            if (SubscribeTopicInternal(channel_name, message_type)) {
                AINFO << "Auto-subscribed topic via topology change: "
                      << channel_name << " (message type: " << message_type
                      << ")";
            } else {
                AWARN << "Failed to auto-subscribe topic via topology change: "
                      << channel_name << " (message type: " << message_type
                      << ")";
            }
        });
    }
}

template <typename MsgType>
bool AutoDiscovery::SubscribeTopicTyped(const std::string& topic_name) {
    if (!node_) {
        AERROR << "Autolink node is not initialized";
        return false;
    }

    // 检查是否已经订阅
    {
        std::lock_guard<std::mutex> lock(subscribed_mutex_);
        if (autolink_readers_.find(topic_name) != autolink_readers_.end()) {
            return true;
        }
    }

    // 创建 Foxglove Channel（Channel 在 autonomy::visualization 命名空间中）
    auto channel = std::make_shared<Channel<MsgType>>(topic_name, true);
    if (!channel) {
        AERROR << "Failed to create channel for topic: " << topic_name;
        return false;
    }

    // 创建 autolink Reader
    autolink::ReaderConfig reader_cfg;
    reader_cfg.channel_name = topic_name;
    reader_cfg.pending_queue_size = 10;

    auto reader = node_->CreateReader<MsgType>(
        reader_cfg, [this, topic_name](const std::shared_ptr<MsgType>& msg) {
            AINFO << "Reader callback triggered for topic: " << topic_name;
            this->OnProtoMessage<MsgType>(topic_name, msg);
        });

    if (!reader) {
        AERROR << "Failed to create reader for topic: " << topic_name;
        return false;
    }

    // CreateReader 已经自动调用了 Init()，不需要再次调用
    AINFO << "Reader created and initialized for topic: " << topic_name;

    // 保存引用
    {
        std::lock_guard<std::mutex> lock(subscribed_mutex_);
        autolink_channels_[topic_name] = channel;
        autolink_readers_[topic_name] = reader;
        subscribed_topics_.insert(topic_name);
    }

    // 等待 reader 连接建立（给 autolink 时间建立连接）
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return true;
}

template <typename MsgType>
void AutoDiscovery::OnProtoMessage(const std::string& topic_name,
                                   const std::shared_ptr<MsgType>& msg) {
    if (!msg) {
        AWARN << "Received null message for topic: " << topic_name;
        return;
    }

    AINFO << "Received message on topic: " << topic_name;

    std::lock_guard<std::mutex> lock(subscribed_mutex_);
    auto it = autolink_channels_.find(topic_name);
    if (it == autolink_channels_.end()) {
        AWARN << "Channel not found for topic: " << topic_name;
        return;
    }

    auto channel = std::static_pointer_cast<Channel<MsgType>>(it->second);
    if (!channel) {
        AWARN << "Failed to cast channel for topic: " << topic_name;
        return;
    }

    AINFO << "Publishing message to Foxglove for topic: " << topic_name;
    if (!channel->Publish(*msg)) {
        AWARN << "Failed to publish message to Foxglove for topic: "
              << topic_name;
    } else {
        AINFO << "Successfully published message to Foxglove for topic: "
              << topic_name;
    }
}

// 显式实例化常用的消息类型
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::sensor_msgs::LaserScan>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::sensor_msgs::PointCloud2>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::sensor_msgs::PointCloud>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::sensor_msgs::Imu>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::sensor_msgs::Range>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::sensor_msgs::Image>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::sensor_msgs::CompressedImage>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::planning_msgs::Path>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::planning_msgs::Odometry>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::map_msgs::OccupancyGrid>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::geometry_msgs::PoseStamped>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::geometry_msgs::PoseArray>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::geometry_msgs::TransformStamped>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::visualization_msgs::Marker>(const std::string&);
template bool AutoDiscovery::SubscribeTopicTyped<
    commsgs::proto::visualization_msgs::MarkerArray>(const std::string&);

}  // namespace transport
}  // namespace visualization
}  // namespace autonomy
