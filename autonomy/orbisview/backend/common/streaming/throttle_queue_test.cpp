/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/streaming/throttle_queue.hpp"

#include <gtest/gtest.h>

using autonomy::orbisview::backend::ThrottleQueue;
using autonomy::orbisview::core::StreamEnvelope;

TEST(ThrottleQueueTest, LatestWinsAndCountsDrops) {
  ThrottleQueue q(1);
  StreamEnvelope a;
  a.channel = "/ch";
  a.sequence = 1;
  StreamEnvelope b = a;
  b.sequence = 2;
  q.Push(a);
  q.Push(b);
  EXPECT_EQ(q.DroppedFrames(), 1u);
  auto out = q.Pop();
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->sequence, 2u);
  EXPECT_FALSE(q.Pop().has_value());
}

TEST(ThrottleQueueTest, PopAllDrainsLatestPerChannel) {
  ThrottleQueue q(1);
  StreamEnvelope a;
  a.channel = "/a";
  a.sequence = 1;
  StreamEnvelope b;
  b.channel = "/b";
  b.sequence = 2;
  q.Push(a);
  q.Push(b);
  auto all = q.PopAll();
  EXPECT_EQ(all.size(), 2u);
  EXPECT_FALSE(q.Pop().has_value());
}

TEST(ThrottleQueueTest, WaitPopForReturnsOnPush) {
  ThrottleQueue q(1);
  StreamEnvelope a;
  a.channel = "/ch";
  a.sequence = 7;
  q.Push(a);
  auto out = q.WaitPopFor(std::chrono::milliseconds(100));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->sequence, 7u);
}
