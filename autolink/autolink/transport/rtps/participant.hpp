/**
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

// Fix FastCDR TEMPLATE_SPEC issue - must be included before any FastCDR headers
#include <fastcdr/config.h>

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <fastdds/rtps/transport/TCPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/utils/IPLocator.hpp>
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/dds/domain/DomainParticipantListener.hpp"

#include "autolink/base/macros.hpp"

#include "autolink/proto/qos_profile.pb.h"
#include "autolink/proto/transport_conf.pb.h"

#include "autolink/common/global_data.hpp"
#include "autolink/common/log.hpp"
#include "autolink/transport/common/common_type.hpp"
#include "autolink/transport/rtps/underlay_message_type.hpp"

namespace autolink {
namespace transport {

class Publisher;
class Subscriber;

class Participant
{
public:
    Participant(
        const std::string& name, int send_port,
        eprosima::fastdds::dds::DomainParticipantListener* listener = nullptr);
    virtual ~Participant();

    void Shutdown();
    bool Init();
    auto CreatePublisher(const std::string& channel_name,
                         const proto::QosProfile& qos)
        -> std::shared_ptr<transport::Publisher>;

    auto CreateSubscriber(const std::string& channel_name,
                          const proto::QosProfile& qos,
                          const rtps::subsciber_callback& callback = nullptr)
        -> std::shared_ptr<Subscriber>;

    bool is_shutdown() const {
        return shutdown_.load();
    }

private:
    Participant(const Participant&) = delete;
    Participant& operator=(const Participant&) = delete;
    bool CreateParticipant(
        const std::string& name, int send_port,
        eprosima::fastdds::dds::DomainParticipantListener* listener);

    bool CheckIPVaild(std::string ip_env);
    std::atomic<bool> shutdown_;
    std::string name_;
    int send_port_;

    using PublisherPtrMap =
        std::unordered_multimap<std::string, std::shared_ptr<Publisher>>;
    using SubscriberPtrMap =
        std::unordered_multimap<std::string, std::shared_ptr<Subscriber>>;

    std::mutex publisher_mutex_;
    PublisherPtrMap publisher_map_;
    std::mutex subscriber_mutex_;
    SubscriberPtrMap subscriber_map_;

    eprosima::fastdds::dds::DomainParticipantListener* listener_;
    eprosima::fastdds::dds::TypeSupport type_support_;
    eprosima::fastdds::dds::DomainParticipant* participant_;
    std::mutex mutex_;
};

}  // namespace transport
}  // namespace autolink