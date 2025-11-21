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

#include <functional>
#include <memory>
#include <string>

#include "autolink/base/macros.hpp"

#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "fastdds/dds/topic/Topic.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"

#include "autolink/proto/qos_profile.pb.h"

#include "autolink/common/log.hpp"
#include "autolink/service_discovery/communication/subscriber_listener.hpp"
#include "autolink/transport/common/common_type.hpp"
#include "autolink/transport/dispatcher/subscriber_listener.hpp"
#include "autolink/transport/message/message_info.hpp"
#include "autolink/transport/qos/qos_filler.hpp"
#include "autolink/transport/rtps/underlay_message.hpp"

namespace autolink {
namespace transport {

class Subscriber
{
public:
    Subscriber(const std::string& name, const proto::QosProfile& qos,
               eprosima::fastdds::dds::DomainParticipant* participant,
               const rtps::subsciber_callback& callback);
    virtual ~Subscriber();

    bool Init();
    void Shutdown();

private:
    Subscriber(const Subscriber&) = delete;
    Subscriber& operator=(const Subscriber&) = delete;
    bool EnsureCreateTopic(const std::string& channel_name);

    std::string channel_name_;
    proto::QosProfile qos_;
    std::atomic<bool> shutdown_;
    rtps::subsciber_callback callback_;

    eprosima::fastdds::dds::DataReaderListener* subscriber_listener_;
    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataReader* reader_;
};
}  // namespace transport
}  // namespace autolink