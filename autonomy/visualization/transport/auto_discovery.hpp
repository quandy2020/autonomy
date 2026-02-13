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

#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/visualization/bridge/foxglove_bridge.hpp"
#include "autonomy/visualization/core/channel.hpp"

namespace autonomy {
namespace visualization {
namespace transport {

/**
 * @brief AutoDiscovery：自动发现 autolink topics 并转发到 Foxglove
 *
 * 功能：
 * - 自动发现 autolink 中所有活跃的 topics
 * - 根据消息类型自动订阅并转发到 Foxglove
 * - 监听 topology 变化，自动添加/移除 topics
 */
class AutoDiscovery
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(AutoDiscovery)

    explicit AutoDiscovery(std::shared_ptr<FoxgloveBridge> bridge);
    ~AutoDiscovery();

    /// 启动自动发现服务
    bool Start();
    /// 停止自动发现服务
    void Stop();
    /// 检查是否运行中
    bool IsRunning() const {
        return running_.load();
    }

    /// 手动订阅指定 topic（支持消息类型自动识别）
    bool SubscribeTopic(const std::string& topic_name, const std::string& message_type = "");

private:
    /// 发现并订阅所有活跃的 topics
    void DiscoverAndSubscribe();

    /// 订阅单个 topic
    bool SubscribeTopicInternal(const std::string& topic_name, const std::string& message_type);

    /// 根据消息类型订阅 topic
    bool SubscribeByMessageType(const std::string& topic_name, const std::string& message_type);

    /// 类型萃取：根据消息类型字符串订阅
    bool SubscribeByTypeTraits(const std::string& topic_name, const std::string& type_lower,
                               const std::string& short_type);

    /// 模板方法：订阅指定类型的 topic
    template <typename MsgType>
    bool SubscribeTopicTyped(const std::string& topic_name);

    /// Topology 变化回调
    void OnTopologyChange(const autolink::proto::ChangeMsg& change_msg);

    /// 发现循环线程
    void DiscoveryLoop();

    /// 处理来自 autolink 的消息并转发到 Foxglove
    template <typename MsgType>
    void OnProtoMessage(const std::string& topic_name, const std::shared_ptr<MsgType>& msg);

    // FoxgloveBridge 实例
    std::shared_ptr<FoxgloveBridge> bridge_{nullptr};

    // Autolink node
    std::shared_ptr<autolink::Node> node_{nullptr};

    // 运行状态
    std::atomic<bool> running_{false};

    // 已订阅的 topics
    std::set<std::string> subscribed_topics_;
    std::mutex subscribed_mutex_;

    // 发现线程
    std::thread discovery_thread_;

    // Topology 监听器连接（使用可选类型来跟踪是否已连接）
    std::optional<autolink::service_discovery::TopologyManager::ChangeConnection> topology_connection_;

    // 存储每个 autolink topic 对应的 Channel 和 Reader（用于转发到 Foxglove）
    std::map<std::string, std::shared_ptr<void>> autolink_channels_;
    std::map<std::string, std::shared_ptr<void>> autolink_readers_;

    /// 根据频率计算是否应该发布消息
    bool ShouldPublishByFrequency(const std::string& topic_name);

    /// 更新topic的频率统计信息
    void UpdateTopicFrequency(const std::string& topic_name);
};

}  // namespace transport
}  // namespace visualization
}  // namespace autonomy
