/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/service_discovery/topology_backend.hpp"
#include "autolink/transport/rtps/participant.hpp"
#include "fastdds/dds/publisher/DataWriter.hpp"
#include "fastdds/dds/publisher/Publisher.hpp"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "fastdds/dds/topic/Topic.hpp"

namespace autolink {
namespace service_discovery {

/**
 * Cross-host topology discovery over RTPS (ChangeMsg on three broadcast topics).
 * Uses RtpsParticipantHub::TopologyParticipant(); QoS RELIABLE + TRANSIENT_LOCAL.
 */
class RtpsTopologyBackend final : public ITopologyBackend
{
public:
    RtpsTopologyBackend();
    ~RtpsTopologyBackend() override;

    bool Start() override;
    int64_t Subscribe(proto::ChangeType type,
                      const ChangeCallback& callback) override;
    void Unsubscribe(int64_t subscription_id) override;
    bool Publish(const proto::ChangeMsg& msg) override;
    void Shutdown() override;

private:
    class TopologyReaderListener;

    struct TopicEndpoint {
        std::string topic_name;
        proto::ChangeType change_type = proto::ChangeType::CHANGE_NODE;
        eprosima::fastdds::dds::Topic* topic = nullptr;
        eprosima::fastdds::dds::DataWriter* writer = nullptr;
        eprosima::fastdds::dds::DataReader* reader = nullptr;
        std::shared_ptr<TopologyReaderListener> listener;
    };

    struct Subscriber {
        proto::ChangeType type;
        ChangeCallback callback;
    };

    bool CreateEndpoints();
    void DestroyEndpoints();
    TopicEndpoint* EndpointFor(proto::ChangeType type);
    void OnUnderlaySample(const std::string& datatype,
                          const std::string& payload);
    void DispatchMessage(const proto::ChangeMsg& msg);
    bool IsFromSelf(const proto::ChangeMsg& msg) const;

    std::atomic<bool> running_{false};
    transport::ParticipantPtr participant_;
    eprosima::fastdds::dds::Publisher* publisher_ = nullptr;
    eprosima::fastdds::dds::Subscriber* subscriber_ = nullptr;
    std::vector<TopicEndpoint> endpoints_;

    std::string self_host_name_;
    int self_process_id_ = 0;

    int64_t next_subscription_id_ = 1;
    std::mutex subscribers_mutex_;
    std::unordered_map<int64_t, Subscriber> subscribers_;
    std::mutex publish_mutex_;
};

}  // namespace service_discovery
}  // namespace autolink
