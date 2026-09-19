/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/command_dispatch.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/bridge/grpc/task_types.hpp"
#include "gtest/gtest.h"

#include <memory>
#include <string>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

enum class DemoCmd { kA = 1, kB = 2, kC = 3 };

TEST(DispatchCommandsTest, MatchesSingleRule) {
    int hit = 0;
    EXPECT_TRUE(DispatchCommands(
        DemoCmd::kA, MakeCommandRule(DemoCmd::kA, [&] { ++hit; })));
    EXPECT_EQ(hit, 1);
}

TEST(DispatchCommandsTest, FirstMatchWins) {
    int hit = 0;
    EXPECT_TRUE(DispatchCommands(
        DemoCmd::kB, MakeCommandRule(DemoCmd::kA, [&] { hit = 1; }),
        MakeCommandRule(DemoCmd::kB, [&] { hit = 2; }),
        MakeCommandRule(DemoCmd::kB, [&] { hit = 3; })));
    EXPECT_EQ(hit, 2);
}

TEST(DispatchCommandsTest, BoolHandlerFailure) {
    EXPECT_FALSE(DispatchCommands(
        DemoCmd::kA, MakeCommandRule(DemoCmd::kA, [] { return false; })));
}

TEST(RejectOnEstopTest, RejectsWhenLatched) {
    auto muxer = TaskMuxer::make_shared();
    muxer->SetEstop(true);
    std::string msg;
    EXPECT_TRUE(RejectOnEstop(muxer, [&](const std::string& m) { msg = m; }));
    EXPECT_FALSE(msg.empty());
}

TEST(RejectOnBusyTest, RejectsWhenSlotHeld) {
    auto muxer = TaskMuxer::make_shared();
    ASSERT_TRUE(muxer->TryAcquire(TASK_TYPE_FOLLOW, "g1", ""));
    struct Req {
        std::string goal_id() const { return "g2"; }
    } request;
    std::string msg;
    EXPECT_TRUE(RejectOnBusy(muxer, TASK_TYPE_NAVIGATION, request,
                             [&](const std::string& m) { msg = m; }));
    EXPECT_FALSE(msg.empty());
}

}  // namespace
}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
