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

#include "autolink/service_discovery/communication/participant_listener.hpp"

#include "autolink/common/log.hpp"

namespace autolink {
namespace service_discovery {

ParticipantListener::ParticipantListener(const ChangeFunc& callback)
    : callback_(callback) {}

ParticipantListener::~ParticipantListener() {
    std::lock_guard<std::mutex> lck(mutex_);
    callback_ = nullptr;
}

void ParticipantListener::on_participant_discovery(
    eprosima::fastdds::dds::DomainParticipant* participant,
    eprosima::fastdds::rtps::ParticipantDiscoveryStatus reason,
    const eprosima::fastdds::dds::ParticipantBuiltinTopicData& info,
    bool& should_be_ignored) {
    (void)participant;
    (void)reason;
    (void)info;
    should_be_ignored = false;

    std::lock_guard<std::mutex> lock(mutex_);
    RETURN_IF_NULL(callback_);
    callback_();
}

}  // namespace service_discovery
}  // namespace autolink