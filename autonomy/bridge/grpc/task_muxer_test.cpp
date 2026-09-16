/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/command_idempotency.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

TEST(TaskMuxerTest, EstopBlocksAcquire) {
    TaskMuxer muxer;
    muxer.SetEstop(true);
    EXPECT_FALSE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "c1", "client"));
    EXPECT_TRUE(muxer.CheckEstopActive());
}

TEST(TaskMuxerTest, ExclusiveSameTypeWhenDisallowed) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_FOLLOW, "a", "c",
                                 /*allow_same_type=*/false));
    const auto second = muxer.TryAcquireDetailed(
        TASK_TYPE_FOLLOW, "b", "c2", /*allow_same_type=*/false);
    EXPECT_FALSE(second.ok);
    EXPECT_EQ(second.holder.cmd_id, "a");
}

TEST(TaskMuxerTest, SameTypeRefreshAllowed) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_TELEOP, "a", "c",
                                 /*allow_same_type=*/true));
    EXPECT_TRUE(muxer.TryAcquire(TASK_TYPE_TELEOP, "a2", "c",
                                 /*allow_same_type=*/true));
}

TEST(TaskMuxerTest, DifferentTypeRejected) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "nav", "c"));
    EXPECT_FALSE(muxer.TryAcquire(TASK_TYPE_FOLLOW, "fol", "c"));
    EXPECT_NE(muxer.DescribeBusy().find("cmd_id=nav"), std::string::npos);
}

TEST(TaskMuxerTest, ReleaseClearsSlot) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_DOCK, "d", "c", false));
    muxer.Release(TASK_TYPE_DOCK);
    EXPECT_FALSE(muxer.CheckHasActive());
    EXPECT_TRUE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "n", "c", false));
}

TEST(TaskMuxerTest, FilteredReleaseLeavesOtherSlot) {
    TaskMuxer muxer;
    ASSERT_TRUE(muxer.TryAcquire(TASK_TYPE_NAVIGATION, "nav", "c"));
    muxer.Release(TASK_TYPE_FOLLOW);
    EXPECT_TRUE(muxer.CheckHasActive());
    muxer.Release(TASK_TYPE_NAVIGATION);
    EXPECT_FALSE(muxer.CheckHasActive());
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
