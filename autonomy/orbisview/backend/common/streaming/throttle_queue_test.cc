/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/streaming/throttle_queue.h"

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
