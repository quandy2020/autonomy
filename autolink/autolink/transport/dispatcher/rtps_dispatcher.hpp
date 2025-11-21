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

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/common/log.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/time/time.hpp"
#include "autolink/transport/dispatcher/dispatcher.hpp"
#include "autolink/transport/dispatcher/subscriber_listener.hpp"
#include "autolink/transport/qos/qos_filler.hpp"
#include "autolink/transport/rtps/attributes_filler.hpp"
#include "autolink/transport/rtps/participant.hpp"
#include "autolink/transport/rtps/subscriber.hpp"

namespace autolink {
namespace transport {

class RtpsDispatcher;
using RtpsDispatcherPtr = RtpsDispatcher*;

using ParticipantPtr = std::shared_ptr<Participant>;

class RtpsDispatcher : public Dispatcher
{
public:
    virtual ~RtpsDispatcher();

    void Shutdown() override;

    template <typename MessageT>
    void AddListener(const RoleAttributes& self_attr,
                     const MessageListener<MessageT>& listener);

    template <typename MessageT>
    void AddListener(const RoleAttributes& self_attr,
                     const RoleAttributes& opposite_attr,
                     const MessageListener<MessageT>& listener);

    void SetParticipant(const ParticipantPtr& participant) {
        participant_ = participant;
    }

private:
    void OnMessage(uint64_t channel_id,
                   const std::shared_ptr<std::string>& msg_str,
                   const MessageInfo& msg_info);
    void AddSubscriber(const RoleAttributes& self_attr);
    // key: channel_id
    std::unordered_map<uint64_t, std::shared_ptr<transport::Subscriber>> subs_;
    std::mutex subs_mutex_;

    ParticipantPtr participant_;

    DECLARE_SINGLETON(RtpsDispatcher)
};

template <typename MessageT>
void RtpsDispatcher::AddListener(const RoleAttributes& self_attr,
                                 const MessageListener<MessageT>& listener) {
    auto listener_adapter = [listener, self_attr](
                                const std::shared_ptr<std::string>& msg_str,
                                const MessageInfo& msg_info) {
        auto msg = std::make_shared<MessageT>();
        RETURN_IF(!message::ParseFromString(*msg_str, msg.get()));
        uint64_t recv_time = Time::Now().ToNanosecond();
        uint64_t send_time = msg_info.send_time();
        if (send_time > recv_time) {
            AWARN << "The message is received (" << recv_time
                  << ") earlier than the message is sent (" << send_time << ")";
        } else {
            auto diff = (recv_time - send_time);
            // TODO: SamplingTranLatency method not implemented yet
            // statistics::Statistics::Instance(true)->SamplingTranLatency<uint64_t>(
            //     self_attr.channel_name(), diff);
        }
        // Statistics removed
        (void)recv_time;
        (void)self_attr;
        listener(msg, msg_info);
    };

    Dispatcher::AddListener<std::string>(self_attr, listener_adapter);
    AddSubscriber(self_attr);
}

template <typename MessageT>
void RtpsDispatcher::AddListener(const RoleAttributes& self_attr,
                                 const RoleAttributes& opposite_attr,
                                 const MessageListener<MessageT>& listener) {
    auto listener_adapter = [listener, self_attr](
                                const std::shared_ptr<std::string>& msg_str,
                                const MessageInfo& msg_info) {
        auto msg = std::make_shared<MessageT>();
        RETURN_IF(!message::ParseFromString(*msg_str, msg.get()));
        uint64_t recv_time = Time::Now().ToNanosecond();
        uint64_t send_time = msg_info.send_time();
        if (send_time > recv_time) {
            AWARN << "The message is received (" << recv_time
                  << ") earlier than the message is sent (" << send_time << ")";
        } else {
            auto diff = (recv_time - send_time);
            // TODO: SamplingTranLatency method not implemented yet
            // statistics::Statistics::Instance(true)->SamplingTranLatency<uint64_t>(
            //     self_attr.channel_name(), diff);
        }
        // Statistics removed
        (void)recv_time;
        (void)self_attr;
        listener(msg, msg_info);
    };

    Dispatcher::AddListener<std::string>(self_attr, opposite_attr,
                                         listener_adapter);
    AddSubscriber(self_attr);
}

}  // namespace transport
}  // namespace autolink