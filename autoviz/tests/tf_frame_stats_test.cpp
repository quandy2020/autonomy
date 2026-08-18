/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "autoviz/transform/buffer.hpp"

namespace {

using autoviz::transform::Buffer;
using autoviz::transform::TfFrameStats;

automsgs::msgs::geometry_msgs::TransformStamped MakeTransform(
    const std::string& parent, const std::string& child, double stamp_seconds) {
  automsgs::msgs::geometry_msgs::TransformStamped message;
  message.mutable_header()->set_frame_id(parent);
  message.set_child_frame_id(child);
  message.mutable_header()->mutable_stamp()->set_sec(
      static_cast<int32_t>(stamp_seconds));
  message.mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(
      (stamp_seconds - static_cast<int32_t>(stamp_seconds)) * 1e9));
  message.mutable_transform()->mutable_rotation()->set_w(1.0);
  return message;
}

const TfFrameStats* FindFrame(const std::vector<TfFrameStats>& frames,
                              const std::string& frame_id) {
  const auto it = std::find_if(
      frames.begin(), frames.end(), [&frame_id](const TfFrameStats& stats) {
        return stats.frame_id == frame_id;
      });
  return it == frames.end() ? nullptr : &(*it);
}

class TfFrameStatsTest : public ::testing::Test {
 protected:
  void SetUp() override { Buffer::Instance()->clear(); }
  void TearDown() override { Buffer::Instance()->clear(); }
};

TEST_F(TfFrameStatsTest, ReportsRateAndBufferSpanLikeViewFrames) {
  Buffer* buffer = Buffer::Instance();
  // 11 samples spanning exactly one second => 11 Hz over a 1.0 sec buffer.
  for (int index = 0; index <= 10; ++index) {
    buffer->setTransform(
        MakeTransform("odom", "base_link", 100.0 + index * 0.1), "publisher");
  }

  const std::vector<TfFrameStats> frames = buffer->frameStats();
  const TfFrameStats* base_link = FindFrame(frames, "base_link");
  ASSERT_NE(base_link, nullptr);
  EXPECT_EQ(base_link->parent_id, "odom");
  EXPECT_EQ(base_link->authority, "publisher");
  EXPECT_NEAR(base_link->buffer_length_seconds, 1.0, 1e-6);
  EXPECT_NEAR(base_link->average_rate_hertz, 11.0, 1e-3);
  EXPECT_NEAR(static_cast<double>(base_link->oldest_stamp_ns) / 1e9, 100.0,
              1e-6);
  EXPECT_NEAR(static_cast<double>(base_link->last_stamp_ns) / 1e9, 101.0, 1e-6);
}

TEST_F(TfFrameStatsTest, RootFrameHasNoParentAndNoCacheWindow) {
  Buffer* buffer = Buffer::Instance();
  buffer->setTransform(MakeTransform("odom", "base_link", 5.0), "publisher");

  const std::vector<TfFrameStats> frames = buffer->frameStats();
  const TfFrameStats* odom = FindFrame(frames, "odom");
  ASSERT_NE(odom, nullptr);
  EXPECT_TRUE(odom->parent_id.empty() || odom->parent_id == "NO_PARENT");
  // No transform is published *to* the root, so it owns no cache window.
  EXPECT_EQ(odom->transforms_received, 0u);
  EXPECT_DOUBLE_EQ(odom->buffer_length_seconds, 0.0);
}

TEST_F(TfFrameStatsTest, SingleSampleDoesNotDivideByZero) {
  Buffer* buffer = Buffer::Instance();
  buffer->setTransform(MakeTransform("base_link", "laser", 42.0), "publisher");

  const std::vector<TfFrameStats> frames = buffer->frameStats();
  const TfFrameStats* laser = FindFrame(frames, "laser");
  ASSERT_NE(laser, nullptr);
  EXPECT_DOUBLE_EQ(laser->buffer_length_seconds, 0.0);
  EXPECT_TRUE(std::isfinite(laser->average_rate_hertz));
  EXPECT_GT(laser->average_rate_hertz, 0.0);
}

TEST_F(TfFrameStatsTest, StaticTransformIsFlagged) {
  Buffer* buffer = Buffer::Instance();
  buffer->setTransform(MakeTransform("base_link", "imu_link", 7.0),
                       "static_publisher", true);

  const std::vector<TfFrameStats> frames = buffer->frameStats();
  const TfFrameStats* imu = FindFrame(frames, "imu_link");
  ASSERT_NE(imu, nullptr);
  EXPECT_TRUE(imu->is_static);
  EXPECT_EQ(imu->parent_id, "base_link");
}

}  // namespace
