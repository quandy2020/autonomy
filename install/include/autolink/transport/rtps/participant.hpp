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
#include <vector>

#include "autolink/proto/transport_conf.pb.h"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"

namespace autolink {
namespace transport {

class Participant;
using ParticipantPtr = std::shared_ptr<Participant>;

enum class ParticipantRole {
    kTopology,
    kTransport,
};

/**
 * Owns a Fast DDS 3.x DomainParticipant for RTPS transport or topology.
 * Domain id from GlobalData::DomainId(); unicast from HostIp().
 * Empty discovery_servers keeps SIMPLE discovery; non-empty uses CLIENT.
 */
class Participant
{
public:
    Participant(const proto::RtpsParticipantAttr& attr, ParticipantRole role,
                const std::vector<std::string>& discovery_servers = {});
    ~Participant();

    Participant(const Participant&) = delete;
    Participant& operator=(const Participant&) = delete;

    bool Init();
    void Shutdown();

    eprosima::fastdds::dds::DomainParticipant* get();
    bool is_shutdown() const {
        return shutdown_.load();
    }

    ParticipantRole role() const {
        return role_;
    }

private:
    std::atomic<bool> shutdown_{false};
    proto::RtpsParticipantAttr attr_;
    ParticipantRole role_ = ParticipantRole::kTransport;
    std::vector<std::string> discovery_servers_;
    uint32_t domain_id_ = 80;
    std::string host_ip_;
    std::string name_;
    eprosima::fastdds::dds::TypeSupport type_;
    eprosima::fastdds::dds::DomainParticipant* participant_ = nullptr;
    std::mutex mutex_;
};

}  // namespace transport
}  // namespace autolink
