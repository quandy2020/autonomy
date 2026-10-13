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

#include <mutex>

#include "autolink/proto/transport_conf.pb.h"
#include "autolink/transport/rtps/participant.hpp"

namespace autolink {
namespace transport {

/**
 * Process-wide owner of the topology and transport DomainParticipants.
 * Transport and RtpsTopologyBackend both obtain participants from here.
 */
class RtpsParticipantHub
{
public:
    static RtpsParticipantHub& Instance();

    /** Creates both participants. Idempotent: returns true if already inited. */
    bool Init(const proto::RtpsParticipantAttr& attr);
    ParticipantPtr TopologyParticipant() const;
    ParticipantPtr TransportParticipant() const;
    void Shutdown();

private:
    RtpsParticipantHub() = default;
    ~RtpsParticipantHub() = default;
    RtpsParticipantHub(const RtpsParticipantHub&) = delete;
    RtpsParticipantHub& operator=(const RtpsParticipantHub&) = delete;

    mutable std::mutex mutex_;
    bool inited_ = false;
    ParticipantPtr topology_;
    ParticipantPtr transport_;
};

}  // namespace transport
}  // namespace autolink
