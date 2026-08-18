/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "autolink/message/message_header.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/transform/listener.hpp"

namespace {

using autoviz::transform::Buffer;
using autoviz::transform::Listener;
using autoviz::transform::TfFrameStats;

std::string SerializeTfMessage(const std::string& parent,
                               const std::string& child) {
  automsgs::msgs::tf2_msgs::TFMessage message;
  auto* transform = message.add_transforms();
  transform->mutable_header()->set_frame_id(parent);
  transform->mutable_header()->mutable_stamp()->set_sec(12);
  transform->set_child_frame_id(child);
  transform->mutable_transform()->mutable_rotation()->set_w(1.0);
  return message.SerializeAsString();
}

/** Wrap a payload the way autolink frames messages on the wire. */
std::string WithMessageHeader(const std::string& body) {
  autolink::message::MessageHeader header;
  header.set_content_size(static_cast<uint32_t>(body.size()));
  std::string payload(sizeof(header), '\0');
  std::memcpy(payload.data(), &header, sizeof(header));
  payload.append(body);
  return payload;
}

const TfFrameStats* FindFrame(const std::vector<TfFrameStats>& frames,
                              const std::string& frame_id) {
  const auto it = std::find_if(
      frames.begin(), frames.end(), [&frame_id](const TfFrameStats& stats) {
        return stats.frame_id == frame_id;
      });
  return it == frames.end() ? nullptr : &(*it);
}

class TfListenerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Buffer::Instance()->clear();
    // No autolink node in tests, so no subscription is created; start() still
    // binds the buffer that applyPayload writes into.
    listener_.start(Buffer::Instance());
  }
  void TearDown() override {
    listener_.stop();
    Buffer::Instance()->clear();
  }

  Listener listener_;
};

TEST_F(TfListenerTest, AppliesStaticTransformsNoDisplayReads) {
  ASSERT_TRUE(
      listener_.applyPayload(SerializeTfMessage("base_link", "imu_link"), true));

  const TfFrameStats* imu =
      FindFrame(Buffer::Instance()->frameStats(), "imu_link");
  ASSERT_NE(imu, nullptr);
  EXPECT_EQ(imu->parent_id, "base_link");
  EXPECT_TRUE(imu->is_static);
}

TEST_F(TfListenerTest, AppliesDynamicTransforms) {
  ASSERT_TRUE(
      listener_.applyPayload(SerializeTfMessage("odom", "base_link"), false));

  const TfFrameStats* base_link =
      FindFrame(Buffer::Instance()->frameStats(), "base_link");
  ASSERT_NE(base_link, nullptr);
  EXPECT_EQ(base_link->parent_id, "odom");
  EXPECT_FALSE(base_link->is_static);
}

TEST_F(TfListenerTest, DecodesHeaderFramedPayload) {
  ASSERT_TRUE(listener_.applyPayload(
      WithMessageHeader(SerializeTfMessage("odom", "base_link")), false));

  EXPECT_NE(FindFrame(Buffer::Instance()->frameStats(), "base_link"), nullptr);
}

TEST_F(TfListenerTest, RejectsUnparseablePayload) {
  // clear() keeps known frame ids, so compare against the pre-state rather than
  // expecting an empty buffer.
  const std::size_t before = Buffer::Instance()->frameStats().size();

  EXPECT_FALSE(listener_.applyPayload("not a protobuf payload", false));
  EXPECT_FALSE(listener_.applyPayload(std::string(), false));

  EXPECT_EQ(Buffer::Instance()->frameStats().size(), before);
}

TEST_F(TfListenerTest, CoversNothingWithoutSubscriptions) {
  // covers() gates display-side re-application; without a live subscription the
  // display must keep feeding the buffer itself.
  EXPECT_FALSE(listener_.covers("/tf"));
  EXPECT_FALSE(listener_.covers(""));
}

}  // namespace
