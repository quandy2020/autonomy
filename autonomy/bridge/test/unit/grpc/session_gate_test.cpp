/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/session.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include "gtest/gtest.h"

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

TEST(SessionGateTest, NullSchedulerRejected) {
    auto muxer = TaskMuxer::make_shared();
    BackgroundCommandSession<std::string> session(
        /*scheduler=*/nullptr, muxer, TASK_TYPE_NAVIGATION);
    std::vector<std::string> frames;
    const bool ok = session.Start(
        [&](const std::string& f) { frames.push_back(f); }, "ack",
        [](const std::string& msg) { return std::string("reject:") + msg; },
        [](const auto&) { return true; }, "cmd-1");
    EXPECT_FALSE(ok);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], "reject:work scheduler unavailable");
}

TEST(SessionGateTest, EstopBlocksStart) {
    auto scheduler = WorkScheduler::make_shared(1);
    auto muxer = TaskMuxer::make_shared();
    muxer->SetEstop(true);
    BackgroundCommandSession<std::string> session(
        scheduler.get(), muxer, TASK_TYPE_NAVIGATION);
    std::vector<std::string> frames;
    const bool ok = session.Start(
        [&](const std::string& f) { frames.push_back(f); }, "ack",
        [](const std::string& msg) { return std::string("reject:") + msg; },
        [](const auto&) { return true; }, "cmd-1");
    EXPECT_FALSE(ok);
    ASSERT_FALSE(frames.empty());
}

TEST(SessionGateTest, AcceptThenExecute) {
    auto scheduler = WorkScheduler::make_shared(1);
    auto muxer = TaskMuxer::make_shared();
    BackgroundCommandSession<std::string> session(
        scheduler.get(), muxer, TASK_TYPE_TELEOP);
    std::vector<std::string> frames;
    std::atomic<bool> executed{false};
    const bool ok = session.Start(
        [&](const std::string& f) { frames.push_back(f); }, "ack",
        [](const std::string& msg) { return std::string("reject:") + msg; },
        [&](const auto&) {
            executed = true;
            return true;
        },
        "cmd-teleop");
    EXPECT_TRUE(ok);
    ASSERT_FALSE(frames.empty());
    EXPECT_EQ(frames[0], "ack");
    for (int i = 0; i < 50 && !executed.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(executed.load());
}

}  // namespace
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
