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

#include <atomic>
#include <memory>
#include <string>

#include <atomic>
#include <memory>
#include <string>

#include "autolink/proto/transport_conf.pb.h"

#include "autolink/common/macros.hpp"
#include "autolink/transport/dispatcher/intra_dispatcher.hpp"
#include "autolink/transport/dispatcher/rtps_dispatcher.hpp"
#include "autolink/transport/dispatcher/shm_dispatcher.hpp"
#include "autolink/transport/qos/qos_profile_conf.hpp"
#include "autolink/transport/receiver/hybrid_receiver.hpp"
#include "autolink/transport/receiver/intra_receiver.hpp"
#include "autolink/transport/receiver/receiver.hpp"
#include "autolink/transport/receiver/rtps_receiver.hpp"
#include "autolink/transport/receiver/shm_receiver.hpp"
#include "autolink/transport/rtps/participant.hpp"
#include "autolink/transport/shm/notifier_factory.hpp"
#include "autolink/transport/transmitter/hybrid_transmitter.hpp"
#include "autolink/transport/transmitter/intra_transmitter.hpp"
#include "autolink/transport/transmitter/rtps_transmitter.hpp"
#include "autolink/transport/transmitter/shm_transmitter.hpp"
#include "autolink/transport/transmitter/transmitter.hpp"

namespace autolink {
namespace transport {

using autolink::proto::OptionalMode;

class Transport
{
public:
    virtual ~Transport();

    void Shutdown();

    template <typename M>
    auto CreateTransmitter(const RoleAttributes& attr,
                           const OptionalMode& mode = OptionalMode::HYBRID) ->
        typename std::shared_ptr<Transmitter<M>>;

    template <typename M>
    auto CreateReceiver(
        const RoleAttributes& attr,
        const typename Receiver<M>::MessageListener& msg_listener,
        const OptionalMode& mode = OptionalMode::HYBRID) ->
        typename std::shared_ptr<Receiver<M>>;

    ParticipantPtr participant() const {
        return participant_;
    }

private:
    void CreateParticipant();

    std::atomic<bool> is_shutdown_ = {false};
    ParticipantPtr participant_ = nullptr;
    NotifierPtr notifier_ = nullptr;
    IntraDispatcherPtr intra_dispatcher_ = nullptr;
    ShmDispatcherPtr shm_dispatcher_ = nullptr;
    RtpsDispatcherPtr rtps_dispatcher_ = nullptr;

    DECLARE_SINGLETON(Transport)
};

template <typename M>
auto Transport::CreateTransmitter(const RoleAttributes& attr,
                                  const OptionalMode& mode) ->
    typename std::shared_ptr<Transmitter<M>> {
    if (is_shutdown_.load()) {
        AINFO << "transport has been shut down.";
        return nullptr;
    }

    std::shared_ptr<Transmitter<M>> transmitter = nullptr;
    RoleAttributes modified_attr = attr;
    if (!modified_attr.has_qos_profile()) {
        modified_attr.mutable_qos_profile()->CopyFrom(
            QosProfileConf::QOS_PROFILE_DEFAULT);
    }

    switch (mode) {
        case OptionalMode::INTRA:
            transmitter = std::make_shared<IntraTransmitter<M>>(modified_attr);
            break;

        case OptionalMode::SHM:
            transmitter = std::make_shared<ShmTransmitter<M>>(modified_attr);
            break;

        case OptionalMode::RTPS:
            transmitter = std::make_shared<RtpsTransmitter<M>>(modified_attr,
                                                               participant());
            break;

        default:
            transmitter = std::make_shared<HybridTransmitter<M>>(modified_attr,
                                                                 participant());
            break;
    }

    RETURN_VAL_IF_NULL(transmitter, nullptr);
    // For HYBRID mode, Enable is called after topology discovery
    // For specific modes, enable immediately
    if (mode == OptionalMode::HYBRID) {
        // HYBRID mode will be enabled via OnChannelChange callback
    } else {
        transmitter->Enable(modified_attr);
    }
    return transmitter;
}

template <typename M>
auto Transport::CreateReceiver(
    const RoleAttributes& attr,
    const typename Receiver<M>::MessageListener& msg_listener,
    const OptionalMode& mode) -> typename std::shared_ptr<Receiver<M>> {
    if (is_shutdown_.load()) {
        AINFO << "transport has been shut down.";
        return nullptr;
    }

    std::shared_ptr<Receiver<M>> receiver = nullptr;
    RoleAttributes modified_attr = attr;
    if (!modified_attr.has_qos_profile()) {
        modified_attr.mutable_qos_profile()->CopyFrom(
            QosProfileConf::QOS_PROFILE_DEFAULT);
    }

    switch (mode) {
        case OptionalMode::INTRA:
            receiver =
                std::make_shared<IntraReceiver<M>>(modified_attr, msg_listener);
            break;

        case OptionalMode::SHM:
            receiver =
                std::make_shared<ShmReceiver<M>>(modified_attr, msg_listener);
            break;

        case OptionalMode::RTPS:
            receiver =
                std::make_shared<RtpsReceiver<M>>(modified_attr, msg_listener);
            break;

        default:
            receiver = std::make_shared<HybridReceiver<M>>(
                modified_attr, msg_listener, participant());
            break;
    }

    RETURN_VAL_IF_NULL(receiver, nullptr);
    // For HYBRID mode, Enable is called after topology discovery
    // For specific modes, enable immediately
    if (mode == OptionalMode::HYBRID) {
        // HYBRID mode will be enabled via OnChannelChange callback
    } else {
        receiver->Enable();
    }
    return receiver;
}

}  // namespace transport
}  // namespace autolink
