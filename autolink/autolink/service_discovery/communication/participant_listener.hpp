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

#include <functional>
#include <mutex>

#include "autolink/base/macros.hpp"

#include "fastdds/dds/builtin/topic/ParticipantBuiltinTopicData.hpp"
#include "fastdds/dds/domain/DomainParticipantListener.hpp"
#include "fastdds/rtps/participant/ParticipantDiscoveryInfo.hpp"

namespace autolink {
namespace service_discovery {

class ParticipantListener
    : public eprosima::fastdds::dds::DomainParticipantListener
{
public:
    using ChangeFunc = std::function<void()>;

    explicit ParticipantListener(const ChangeFunc& callback);
    virtual ~ParticipantListener();

    void on_participant_discovery(
        eprosima::fastdds::dds::DomainParticipant* participant,
        eprosima::fastdds::rtps::ParticipantDiscoveryStatus reason,
        const eprosima::fastdds::dds::ParticipantBuiltinTopicData& info,
        bool& should_be_ignored) override;

private:
    ChangeFunc callback_;
    std::mutex mutex_;
};

}  // namespace service_discovery
}  // namespace autolink