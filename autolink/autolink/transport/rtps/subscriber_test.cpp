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

#include <atomic>
#include <string>

#include "gtest/gtest.h"

#include "autolink/proto/qos_profile.pb.h"
#include "autolink/transport/message/message_info.hpp"
#include "autolink/transport/qos/qos_profile_conf.hpp"
#include "autolink/transport/rtps/subscriber.hpp"
#include "autolink/transport/rtps/underlay_message.hpp"
#include "autolink/transport/rtps/underlay_message_type.hpp"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"

namespace autolink {
namespace transport {
namespace {

class SubscriberTest : public ::testing::Test
{
protected:
    void SetUp() override {
        participant_ =
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
                ->create_participant(
                    0, eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT);
        ASSERT_NE(participant_, nullptr);

        type_support_ =
            eprosima::fastdds::dds::TypeSupport(new UnderlayMessageType());
        ASSERT_EQ(type_support_.register_type(participant_),
                  eprosima::fastdds::dds::RETCODE_OK);
    }

    void TearDown() override {
        if (participant_ != nullptr) {
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()
                ->delete_participant(participant_);
            participant_ = nullptr;
        }
    }

    eprosima::fastdds::dds::DomainParticipant* participant_{nullptr};
    eprosima::fastdds::dds::TypeSupport type_support_;
};

TEST_F(SubscriberTest, InitAndShutdownSuccess) {
    proto::QosProfile qos = QosProfileConf::QOS_PROFILE_DEFAULT;
    std::atomic<int> receive_count{0};
    rtps::subsciber_callback callback =
        [&receive_count](const std::shared_ptr<std::string>& message,
                         uint64_t channel_id, const MessageInfo& info) {
            (void)message;
            (void)channel_id;
            (void)info;
            ++receive_count;
        };

    Subscriber subscriber("rtps_sub_test_channel", qos, participant_, callback);
    EXPECT_TRUE(subscriber.Init());
    subscriber.Shutdown();
    subscriber.Shutdown();  // idempotent
    EXPECT_EQ(receive_count.load(), 0);
}

TEST_F(SubscriberTest, InitFailsWhenQosInvalid) {
    proto::QosProfile qos = QosProfileConf::QOS_PROFILE_DEFAULT;
    qos.set_depth(-1);
    rtps::subsciber_callback callback = [](const std::shared_ptr<std::string>&,
                                           uint64_t, const MessageInfo&) {};

    Subscriber subscriber("invalid_depth_channel", qos, participant_, callback);
    EXPECT_FALSE(subscriber.Init());
}

}  // namespace
}  // namespace transport
}  // namespace autolink
