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

#include <memory>
#include <string>

#include "autolink/common/log.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/transport/rtps/attributes_filler.hpp"
#include "autolink/transport/rtps/message_info_prefix.hpp"
#include "autolink/transport/rtps/participant.hpp"
#include "autolink/transport/rtps/payload_limit.hpp"
#include "autolink/transport/rtps/rtps_stats.hpp"
#include "autolink/transport/rtps/underlay_message.hpp"
#include "autolink/transport/transmitter/transmitter.hpp"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/publisher/DataWriter.hpp"
#include "fastdds/dds/publisher/DataWriterListener.hpp"
#include "fastdds/dds/publisher/Publisher.hpp"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastdds/dds/publisher/qos/PublisherQos.hpp"
#include "fastdds/dds/topic/Topic.hpp"
#include "fastdds/dds/core/ReturnCode.hpp"
#include "fastdds/dds/topic/qos/TopicQos.hpp"

namespace autolink {
namespace transport {

class RtpsWriterListener : public eprosima::fastdds::dds::DataWriterListener {
public:
    void on_publication_matched(
            eprosima::fastdds::dds::DataWriter* /*writer*/,
            const eprosima::fastdds::dds::PublicationMatchedStatus& info)
            override {
        RtpsStats::Instance().SetMatchedReaders(info.current_count);
    }
};

template <typename M>
class RtpsTransmitter : public Transmitter<M>
{
public:
    using MessagePtr = std::shared_ptr<M>;

    RtpsTransmitter(const RoleAttributes& attr,
                    const ParticipantPtr& participant);
    virtual ~RtpsTransmitter();

    void Enable() override;
    void Disable() override;

    void Enable(const RoleAttributes& opposite_attr) override;
    void Disable(const RoleAttributes& opposite_attr) override;

    bool Transmit(const MessagePtr& msg, const MessageInfo& msg_info) override;

    bool AcquireMessage(std::shared_ptr<M>& msg);

private:
    bool Transmit(const M& msg, const MessageInfo& msg_info);

    eprosima::fastdds::dds::Topic* EnsureTopic(
            eprosima::fastdds::dds::DomainParticipant* dp,
            const std::string& channel_name);

    ParticipantPtr participant_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataWriter* writer_;
    std::shared_ptr<RtpsWriterListener> writer_listener_;
};

template <typename M>
bool RtpsTransmitter<M>::AcquireMessage(std::shared_ptr<M>& msg) {
    (void)msg;
    return false;
}

template <typename M>
RtpsTransmitter<M>::RtpsTransmitter(const RoleAttributes& attr,
                                    const ParticipantPtr& participant)
    : Transmitter<M>(attr),
      participant_(participant),
      publisher_(nullptr),
      topic_(nullptr),
      writer_(nullptr),
      writer_listener_(std::make_shared<RtpsWriterListener>()) {}

template <typename M>
RtpsTransmitter<M>::~RtpsTransmitter() {
    Disable();
}

template <typename M>
void RtpsTransmitter<M>::Enable(const RoleAttributes& opposite_attr) {
    (void)opposite_attr;
    this->Enable();
}

template <typename M>
void RtpsTransmitter<M>::Disable(const RoleAttributes& opposite_attr) {
    (void)opposite_attr;
    this->Disable();
}

template <typename M>
eprosima::fastdds::dds::Topic* RtpsTransmitter<M>::EnsureTopic(
        eprosima::fastdds::dds::DomainParticipant* dp,
        const std::string& channel_name) {
    RETURN_VAL_IF_NULL(dp, nullptr);
    auto* desc = dp->lookup_topicdescription(channel_name);
    if (desc != nullptr) {
        return dynamic_cast<eprosima::fastdds::dds::Topic*>(desc);
    }
    return dp->create_topic(channel_name, "UnderlayMessage",
                            eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
}

template <typename M>
void RtpsTransmitter<M>::Enable() {
    if (this->enabled_) {
        return;
    }

    RETURN_IF_NULL(participant_);
    auto* dp = participant_->get();
    RETURN_IF_NULL(dp);

    topic_ = EnsureTopic(dp, this->attr_.channel_name());
    RETURN_IF_NULL(topic_);

    publisher_ = dp->create_publisher(
            eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT);
    RETURN_IF_NULL(publisher_);

    eprosima::fastdds::dds::DataWriterQos wqos =
            eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
    RETURN_IF(!AttributesFiller::FillInPubQos(this->attr_.qos_profile(), &wqos));

    writer_ = publisher_->create_datawriter(
            topic_, wqos, writer_listener_.get(),
            eprosima::fastdds::dds::StatusMask::all());
    RETURN_IF_NULL(writer_);

    this->enabled_ = true;
}

template <typename M>
void RtpsTransmitter<M>::Disable() {
    if (!this->enabled_) {
        return;
    }

    if (participant_ != nullptr) {
        auto* dp = participant_->get();
        if (publisher_ != nullptr && writer_ != nullptr) {
            publisher_->delete_datawriter(writer_);
        }
        if (dp != nullptr && publisher_ != nullptr) {
            dp->delete_publisher(publisher_);
        }
        // Topic may be shared with dispatcher readers; leave for participant
        // teardown.
    }

    writer_ = nullptr;
    publisher_ = nullptr;
    topic_ = nullptr;
    // writer_listener_ kept for reuse on next Enable().
    this->enabled_ = false;
}

template <typename M>
bool RtpsTransmitter<M>::Transmit(const MessagePtr& msg,
                                  const MessageInfo& msg_info) {
    return Transmit(*msg, msg_info);
}

template <typename M>
bool RtpsTransmitter<M>::Transmit(const M& msg, const MessageInfo& msg_info) {
    if (!this->enabled_) {
        ADEBUG << "not enable.";
        return false;
    }
    RETURN_VAL_IF_NULL(writer_, false);
    if (participant_ != nullptr && participant_->is_shutdown()) {
        return false;
    }

    UnderlayMessage underlay;
    RETURN_VAL_IF(!message::SerializeToString(msg, &underlay.data()), false);
    RETURN_VAL_IF(!PackMessageInfoPrefix(msg_info, &underlay.data()), false);

    const auto payload_limit = PayloadLimit::FromEnv();
    const auto check =
            CheckPayloadSize(underlay.data().size(), payload_limit);
    if (check == PayloadCheck::kWarn) {
        AWARN << "RTPS payload exceeds soft limit: size="
              << underlay.data().size()
              << " max=" << payload_limit.max_bytes;
    } else if (check == PayloadCheck::kReject) {
        AERROR << "RTPS payload rejected (oversize): size="
               << underlay.data().size()
               << " max=" << payload_limit.max_bytes;
        RtpsStats::Instance().AddOversize();
        return false;
    }

    underlay.timestamp(
            static_cast<int32_t>(0x0fffffff & msg_info.send_time()));
    underlay.seq(msg_info.msg_seq_num());

    // Fast DDS 3.x: write returns ReturnCode_t (RETCODE_OK == 0).
    if (writer_->write(&underlay) == eprosima::fastdds::dds::RETCODE_OK) {
        RtpsStats::Instance().AddSent();
        return true;
    }
    RtpsStats::Instance().AddWriteFail();
    return false;
}

}  // namespace transport
}  // namespace autolink
