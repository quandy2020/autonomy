/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/idempotency.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

TEST(TaskMuxerTest, EstopBlocksAcquire) {
    TaskMuxer muxer;
    muxer.SetEstop(true);
    EXPECT_FALSE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "c1", "client"));
    EXPECT_TRUE(muxer.IsEstop());
}

TEST(TaskMuxerTest, ExclusiveAcrossTypes) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_FOLLOW, "a", "c"));
    EXPECT_FALSE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "b", "c2"));
}

TEST(TaskMuxerTest, SameTypeReacquireAllowed) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_TELEOP, "a", "c"));
    EXPECT_TRUE(muxer.TryAcquire(TASK_TYPE_TELEOP, "a2", "c"));
}

TEST(TaskMuxerTest, ReleaseClearsSlot) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_DOCK, "d", "c"));
    muxer.Release(TASK_TYPE_DOCK);
    EXPECT_FALSE(muxer.HasActive());
    EXPECT_TRUE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "n", "c"));
}

TEST(TaskMuxerTest, FilteredReleaseLeavesOtherSlot) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "nav", "c"));
    muxer.Release(TASK_TYPE_FOLLOW);
    EXPECT_TRUE(muxer.HasActive());
    muxer.Release(TASK_TYPE_NAVIGATION);
    EXPECT_FALSE(muxer.HasActive());
}

TEST(CommandIdempotencyCacheTest, DuplicateRejected) {
    CommandIdempotencyCache cache(std::chrono::seconds(60));
    EXPECT_TRUE(cache.TryBegin("cmd-1", TASK_TYPE_NAVIGATION));
    EXPECT_FALSE(cache.TryBegin("cmd-1", TASK_TYPE_NAVIGATION));
    cache.Forget("cmd-1");
    EXPECT_TRUE(cache.TryBegin("cmd-1", TASK_TYPE_NAVIGATION));
}

TEST(CommandIdempotencyCacheTest, EmptyCmdIdAlwaysOk) {
    CommandIdempotencyCache cache(std::chrono::seconds(60));
    EXPECT_TRUE(cache.TryBegin("", TASK_TYPE_FOLLOW));
    EXPECT_TRUE(cache.TryBegin("", TASK_TYPE_FOLLOW));
}

}  // namespace
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
