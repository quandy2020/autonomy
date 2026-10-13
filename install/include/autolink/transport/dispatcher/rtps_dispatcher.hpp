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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/common/log.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/transport/dispatcher/dispatcher.hpp"
#include "autolink/transport/rtps/attributes_filler.hpp"
#include "autolink/transport/rtps/message_info_prefix.hpp"
#include "autolink/transport/rtps/participant.hpp"
#include "autolink/transport/rtps/rtps_stats.hpp"
#include "autolink/transport/rtps/underlay_message.hpp"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/subscriber/DataReader.hpp"
#include "fastdds/dds/subscriber/DataReaderListener.hpp"
#include "fastdds/dds/subscriber/SampleInfo.hpp"
#include "fastdds/dds/subscriber/Subscriber.hpp"
#include "fastdds/dds/subscriber/qos/DataReaderQos.hpp"
#include "fastdds/dds/subscriber/qos/SubscriberQos.hpp"
#include "fastdds/dds/topic/Topic.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"
#include "fastdds/dds/core/ReturnCode.hpp"
#include "fastdds/dds/topic/qos/TopicQos.hpp"

namespace autolink {
namespace transport {

class RtpsReaderListener;

struct RtpsSubscriber
{
    RtpsSubscriber()
        : topic(nullptr),
          subscriber(nullptr),
          reader(nullptr),
          listener(nullptr) {}

    eprosima::fastdds::dds::Topic* topic;
    eprosima::fastdds::dds::Subscriber* subscriber;
    eprosima::fastdds::dds::DataReader* reader;
    std::shared_ptr<RtpsReaderListener> listener;
};

class RtpsDispatcher;
using RtpsDispatcherPtr = RtpsDispatcher*;

/**
 * Singleton RTPS receive path: one DataReader per channel_id.
 * Listener unpacks 24B MessageInfo prefix then dispatches payload string.
 */
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

    void set_participant(const ParticipantPtr& participant) {
        participant_ = participant;
    }

private:
    void OnMessage(uint64_t channel_id,
                   const std::shared_ptr<std::string>& msg_str,
                   const MessageInfo& msg_info);
    void AddSubscriber(const RoleAttributes& self_attr);

    eprosima::fastdds::dds::Topic* EnsureTopic(
            eprosima::fastdds::dds::DomainParticipant* dp,
            const std::string& channel_name);

    // key: channel_id
    std::unordered_map<uint64_t, RtpsSubscriber> subs_;
    std::mutex subs_mutex_;

    ParticipantPtr participant_;

    DECLARE_SINGLETON(RtpsDispatcher)
};

class RtpsReaderListener : public eprosima::fastdds::dds::DataReaderListener
{
public:
    using NewMsgCallback = std::function<void(
            uint64_t channel_id, const std::shared_ptr<std::string>& msg_str,
            const MessageInfo& msg_info)>;

    RtpsReaderListener(uint64_t channel_id, const NewMsgCallback& callback)
        : channel_id_(channel_id), callback_(callback) {}

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override {
        RETURN_IF_NULL(reader);
        RETURN_IF_NULL(callback_);

        std::lock_guard<std::mutex> lock(mutex_);
        UnderlayMessage sample;
        eprosima::fastdds::dds::SampleInfo info;
        while (reader->take_next_sample(&sample, &info) ==
               eprosima::fastdds::dds::RETCODE_OK) {
            if (!info.valid_data) {
                continue;
            }

            MessageInfo msg_info;
            auto payload = std::make_shared<std::string>();
            if (!UnpackMessageInfoPrefix(sample.data(), &msg_info,
                                         payload.get())) {
                AERROR << "UnpackMessageInfoPrefix failed, channel_id="
                       << channel_id_;
                continue;
            }
            msg_info.set_msg_seq_num(sample.seq());
            msg_info.set_send_time(static_cast<uint64_t>(sample.timestamp()));
            RtpsStats::Instance().AddRecv();
            callback_(channel_id_, payload, msg_info);
        }
    }

    void on_subscription_matched(
            eprosima::fastdds::dds::DataReader* /*reader*/,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
            override {
        RtpsStats::Instance().SetMatchedWriters(info.current_count);
    }

private:
    uint64_t channel_id_;
    NewMsgCallback callback_;
    std::mutex mutex_;
};

template <typename MessageT>
void RtpsDispatcher::AddListener(const RoleAttributes& self_attr,
                                 const MessageListener<MessageT>& listener) {
    auto listener_adapter = [listener](
                                    const std::shared_ptr<std::string>& msg_str,
                                    const MessageInfo& msg_info) {
        auto msg = std::make_shared<MessageT>();
        RETURN_IF(!message::ParseFromString(*msg_str, msg.get()));
        listener(msg, msg_info);
    };

    Dispatcher::AddListener<std::string>(self_attr, listener_adapter);
    AddSubscriber(self_attr);
}

template <typename MessageT>
void RtpsDispatcher::AddListener(const RoleAttributes& self_attr,
                                 const RoleAttributes& opposite_attr,
                                 const MessageListener<MessageT>& listener) {
    auto listener_adapter = [listener](
                                    const std::shared_ptr<std::string>& msg_str,
                                    const MessageInfo& msg_info) {
        auto msg = std::make_shared<MessageT>();
        RETURN_IF(!message::ParseFromString(*msg_str, msg.get()));
        listener(msg, msg_info);
    };

    Dispatcher::AddListener<std::string>(self_attr, opposite_attr,
                                         listener_adapter);
    AddSubscriber(self_attr);
}

}  // namespace transport
}  // namespace autolink
