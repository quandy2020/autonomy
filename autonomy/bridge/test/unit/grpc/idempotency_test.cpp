/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/idempotency.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

TEST(IdempotencyTest, DuplicateRejected) {
    CommandIdempotencyCache cache(std::chrono::seconds(60));
    EXPECT_TRUE(cache.TryBegin("cmd-1", TASK_TYPE_NAVIGATION));
    EXPECT_FALSE(cache.TryBegin("cmd-1", TASK_TYPE_NAVIGATION));
    cache.Forget("cmd-1");
    EXPECT_TRUE(cache.TryBegin("cmd-1", TASK_TYPE_NAVIGATION));
}

TEST(IdempotencyTest, EmptyCmdIdAlwaysOk) {
    CommandIdempotencyCache cache(std::chrono::seconds(60));
    EXPECT_TRUE(cache.TryBegin("", TASK_TYPE_FOLLOW));
    EXPECT_TRUE(cache.TryBegin("", TASK_TYPE_FOLLOW));
}

}  // namespace
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
