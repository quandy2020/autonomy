/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/rate_limit/token_bucket.hpp"
#include "gtest/gtest.h"

#include <thread>

namespace autonomy {
namespace bridge {
namespace policy {
namespace {

TEST(TokenBucketTest, DisabledWhenQpsNonPositive) {
    TokenBucket bucket(0.0, 0);
    EXPECT_FALSE(bucket.enabled());
    for (int i = 0; i < 100; ++i) {
        EXPECT_TRUE(bucket.TryAcquire());
    }
}

TEST(TokenBucketTest, BurstThenExhaust) {
    TokenBucket bucket(/*qps=*/1.0, /*burst=*/3);
    EXPECT_TRUE(bucket.enabled());
    EXPECT_TRUE(bucket.TryAcquire());
    EXPECT_TRUE(bucket.TryAcquire());
    EXPECT_TRUE(bucket.TryAcquire());
    EXPECT_FALSE(bucket.TryAcquire());
}

TEST(TokenBucketTest, RefillAfterWait) {
    TokenBucket bucket(/*qps=*/100.0, /*burst=*/1);
    ASSERT_TRUE(bucket.TryAcquire());
    EXPECT_FALSE(bucket.TryAcquire());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_TRUE(bucket.TryAcquire());
}

}  // namespace
}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
