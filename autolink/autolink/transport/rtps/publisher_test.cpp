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

#include <array>
#include <string>

#include "gtest/gtest.h"

#include "autolink/proto/qos_profile.pb.h"
#include "autolink/transport/message/message_info.hpp"
#include "autolink/transport/qos/qos_profile_conf.hpp"
#include "autolink/transport/rtps/publisher.hpp"
#include "autolink/transport/rtps/underlay_message.hpp"
#include "autolink/transport/rtps/underlay_message_type.hpp"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/domain/DomainParticipantFactory.hpp"
#include "fastdds/dds/domain/qos/DomainParticipantFactoryQos.hpp"
#include "fastdds/dds/domain/qos/DomainParticipantQos.hpp"

namespace autolink {
namespace transport {
namespace {

class PublisherTest : public ::testing::Test
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

TEST_F(PublisherTest, InitAndWriteSuccess) {
    proto::QosProfile qos = QosProfileConf::QOS_PROFILE_DEFAULT;
    Publisher publisher("rtps_test_channel", qos, participant_);
    ASSERT_TRUE(publisher.Init());

    UnderlayMessage message;
    message.data("payload");
    message.seq(1);
    message.timestamp(123456789u);
    EXPECT_TRUE(publisher.Write(message));

    Identity sender(false);
    std::array<char, ID_SIZE> sender_raw{
        {'s', 'e', 'n', 'd', '0', '0', '0', '1'}};
    sender.set_data(sender_raw.data());
    Identity spare(false);
    std::array<char, ID_SIZE> spare_raw{
        {'s', 'p', 'a', 'r', 'e', '0', '0', '1'}};
    spare.set_data(spare_raw.data());

    MessageInfo info;
    info.set_sender_id(sender);
    info.set_spare_id(spare);
    info.set_seq_num(42);
    EXPECT_TRUE(publisher.Write(message, info));

    publisher.Shutdown();
    EXPECT_FALSE(publisher.Write(message));
}

TEST_F(PublisherTest, InitFailsWithInvalidHistoryDepth) {
    proto::QosProfile qos = QosProfileConf::QOS_PROFILE_DEFAULT;
    qos.set_depth(-1);

    Publisher publisher("invalid_qos_channel", qos, participant_);
    EXPECT_FALSE(publisher.Init());
}

}  // namespace
}  // namespace transport
}  // namespace autolink
