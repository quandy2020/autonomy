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

#include "autolink/transport/transport.hpp"

#include "autolink/common/global_data.hpp"
#include "autolink/transport/dispatcher/rtps_dispatcher.hpp"

namespace autolink {
namespace transport {

Transport::Transport() {
    CreateParticipant();
    notifier_ = NotifierFactory::CreateNotifier();
    intra_dispatcher_ = IntraDispatcher::Instance();
    shm_dispatcher_ = ShmDispatcher::Instance();
    rtps_dispatcher_ = RtpsDispatcher::Instance();
    rtps_dispatcher_->SetParticipant(participant_);
}

Transport::~Transport() {
    Shutdown();
}

void Transport::Shutdown() {
    if (is_shutdown_.exchange(true)) {
        return;
    }

    intra_dispatcher_->Shutdown();
    shm_dispatcher_->Shutdown();
    rtps_dispatcher_->Shutdown();
    notifier_->Shutdown();

    if (participant_ != nullptr) {
        participant_->Shutdown();
        participant_ = nullptr;
    }
}

void Transport::CreateParticipant() {
    std::string participant_name =
        common::GlobalData::Instance()->HostName() + "+" +
        std::to_string(common::GlobalData::Instance()->ProcessId());
    participant_ = std::make_shared<Participant>(participant_name, 11512);
    if (!participant_->Init()) {
        AERROR << "Transport inner participant init failed!";
    }
}

}  // namespace transport
}  // namespace autolink