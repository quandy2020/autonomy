/*
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

#include "autonomy/visualization/core/channel.hpp"

#include <string>

#include "autolink/proto/unit_test.pb.h"
#include "gtest/gtest.h"

namespace autonomy {
namespace visualization {

class ChannelTest : public ::testing::Test {
 protected:
  void SetUp() override { topic_name_ = "/test/channel"; }

  void TearDown() override {}

  std::string topic_name_;
};

// Test Channel construction
TEST_F(ChannelTest, Constructor) {
  using TestChannel = Channel<autolink::proto::UnitTest>;

  // Create a channel
  TestChannel channel(topic_name_);

  // Verify topic name is set correctly
  EXPECT_EQ(channel.topic_name(), topic_name_);
}

// Test Channel with different message types
TEST_F(ChannelTest, DifferentMessageTypes) {
  // Test with UnitTest message
  {
    Channel<autolink::proto::UnitTest> channel1("/test/unit_test");
    EXPECT_EQ(channel1.topic_name(), "/test/unit_test");
  }

  // Test with Chatter message (if available)
  // Channel<autolink::proto::Chatter> channel2("/test/chatter");
  // EXPECT_EQ(channel2.topic_name(), "/test/chatter");
}

// Test Publish method
TEST_F(ChannelTest, Publish) {
  using TestChannel = Channel<autolink::proto::UnitTest>;

  TestChannel channel(topic_name_);

  // Create a test message
  autolink::proto::UnitTest message;
  message.set_class_name("TestClass");
  message.set_case_name("TestCase");

  // Publish the message
  bool result = channel.Publish(message);

  // Verify publish succeeded
  EXPECT_TRUE(result);
}

// Test Publish with empty message
TEST_F(ChannelTest, PublishEmptyMessage) {
  using TestChannel = Channel<autolink::proto::UnitTest>;

  TestChannel channel(topic_name_);

  // Create an empty message
  autolink::proto::UnitTest message;

  // Publish the empty message
  bool result = channel.Publish(message);

  // Should still succeed (empty messages are valid)
  EXPECT_TRUE(result);
}

// Test multiple publishes
TEST_F(ChannelTest, MultiplePublishes) {
  using TestChannel = Channel<autolink::proto::UnitTest>;

  TestChannel channel(topic_name_);

  // Publish multiple messages
  for (int i = 0; i < 5; ++i) {
    autolink::proto::UnitTest message;
    message.set_class_name("TestClass" + std::to_string(i));
    message.set_case_name("TestCase" + std::to_string(i));

    EXPECT_TRUE(channel.Publish(message));
  }
}

// Test topic name getter
TEST_F(ChannelTest, TopicNameGetter) {
  using TestChannel = Channel<autolink::proto::UnitTest>;

  const std::string test_topic = "/custom/topic/name";
  TestChannel channel(test_topic);

  EXPECT_EQ(channel.topic_name(), test_topic);
  EXPECT_NE(channel.topic_name(), topic_name_);
}

}  // namespace visualization
}  // namespace autonomy
