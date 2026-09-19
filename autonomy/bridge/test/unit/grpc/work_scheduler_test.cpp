/*
 * Copyright 2026 The Openbot Authors
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/bridge/grpc/session.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include <automsgs/rpcs/navigation.pb.h>
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace {

namespace nav_rpc = ::automsgs::rpcs::navigation;

TEST(WorkSchedulerTest, RunsScheduledWork) {
    WorkScheduler scheduler(2);
    std::mutex mutex;
    std::condition_variable cv;
    bool done = false;

    scheduler.Schedule([&] {
        std::lock_guard<std::mutex> lock(mutex);
        done = true;
        cv.notify_one();
    });

    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(cv.wait_for(lock, std::chrono::seconds(2), [&] { return done; }));
}

TEST(BackgroundCommandSessionTest, AcceptThenExecuteAndRelease) {
    WorkScheduler scheduler(2);
    auto muxer = TaskMuxer::make_shared();
    BackgroundCommandSession<nav_rpc::NavigateResponse> session(
        &scheduler, muxer, TASK_TYPE_NAVIGATION);

    std::mutex mutex;
    std::condition_variable cv;
    std::vector<std::string> messages;
    bool finished = false;

    const bool started = session.Start(
        [&](const nav_rpc::NavigateResponse& response) {
            std::lock_guard<std::mutex> lock(mutex);
            messages.push_back(response.status().message());
            if (response.state() == nav_rpc::NAVIGATION_STATE_ARRIVED ||
                response.state() == nav_rpc::NAVIGATION_STATE_FAILED ||
                response.state() == nav_rpc::NAVIGATION_STATE_CANCELLED) {
                finished = true;
                cv.notify_one();
            }
        },
        [&] {
            nav_rpc::NavigateResponse accept;
            accept.mutable_status()->set_message("accepted");
            accept.set_state(nav_rpc::NAVIGATION_STATE_PLANNING);
            return accept;
        }(),
        [](const std::string& message) {
            nav_rpc::NavigateResponse reject;
            reject.mutable_status()->set_message(message);
            reject.set_state(nav_rpc::NAVIGATION_STATE_FAILED);
            return reject;
        },
        [&](const auto& emit) {
            nav_rpc::NavigateResponse terminal;
            terminal.mutable_status()->set_message("done");
            terminal.set_state(nav_rpc::NAVIGATION_STATE_ARRIVED);
            emit(terminal);
            return false;  // release muxer
        },
        "cmd-1", "client-a");

    ASSERT_TRUE(started);
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(
        cv.wait_for(lock, std::chrono::seconds(2), [&] { return finished; }));
    ASSERT_GE(messages.size(), 2u);
    EXPECT_EQ(messages.front(), "accepted");
    EXPECT_EQ(messages.back(), "done");
    EXPECT_FALSE(muxer->HasActive());
}

TEST(BackgroundCommandSessionTest, RejectsWhenEstop) {
    WorkScheduler scheduler(1);
    auto muxer = TaskMuxer::make_shared();
    muxer->SetEstop(true);
    BackgroundCommandSession<nav_rpc::NavigateResponse> session(
        &scheduler, muxer, TASK_TYPE_FOLLOW);

    std::string message;
    const bool started = session.Start(
        [&](const nav_rpc::NavigateResponse& response) {
            message = response.status().message();
        },
        nav_rpc::NavigateResponse{},
        [](const std::string& msg) {
            nav_rpc::NavigateResponse reject;
            reject.mutable_status()->set_message(msg);
            reject.set_state(nav_rpc::NAVIGATION_STATE_FAILED);
            return reject;
        },
        [](const auto&) { return false; }, "c", "u");

    EXPECT_FALSE(started);
    EXPECT_EQ(message, "emergency stop active");
}

}  // namespace
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
