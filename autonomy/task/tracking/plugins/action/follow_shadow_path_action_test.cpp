/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/tracking/tracking_client.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>

#include "behaviortree_cpp/basic_types.h"

namespace autonomy::task::tracking::internal {
namespace {

using Path = automsgs::msgs::nav_msgs::Path;

Path NonEmptyPath(double x = 0.0) {
    Path path;
    path.mutable_header()->set_frame_id("map");
    path.add_poses()->mutable_pose()->mutable_position()->set_x(x);
    return path;
}

struct FakeSession {
    bool has_path{true};
    bool ready{true};
    Path path{NonEmptyPath()};
    uint64_t revision{1};
    int begin_count{0};
    int tick_count{0};
    int cancel_count{0};
    BT::NodeStatus tick_status{BT::NodeStatus::RUNNING};
    int result_code{0};
    std::string result_message;
    std::string controller_id;

    ShadowPathExecutor::Operations Operations() {
        return {
            [this](Path* output, uint64_t* output_revision) {
                if (!has_path) {
                    return false;
                }
                *output = path;
                *output_revision = revision;
                return true;
            },
            [this]() { return ready; },
            [this](const Path&, const std::string& controller) {
                ++begin_count;
                controller_id = controller;
            },
            [this](int* error_code, std::string* error_message) {
                ++tick_count;
                *error_code = result_code;
                *error_message = result_message;
                return tick_status;
            },
            [this]() { ++cancel_count; },
        };
    }
};

TEST(FollowShadowPathActionTest, BeginsFirstPathAndPollsUnchangedRevision) {
    FakeSession session;
    ShadowPathExecutor executor(session.Operations());
    int error_code = -1;
    std::string error_message;

    EXPECT_EQ(executor.Tick("shadow_controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);
    EXPECT_EQ(session.begin_count, 1);
    EXPECT_EQ(session.tick_count, 0);
    EXPECT_EQ(session.controller_id, "shadow_controller");

    EXPECT_EQ(executor.Tick("shadow_controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);
    EXPECT_EQ(session.begin_count, 1);
    EXPECT_EQ(session.tick_count, 1);
}

TEST(FollowShadowPathActionTest, PreemptsOnlyWhenPathRevisionChanges) {
    FakeSession session;
    ShadowPathExecutor executor(session.Operations());
    int error_code = 0;
    std::string error_message;

    ASSERT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);
    session.revision = 2;
    session.path = NonEmptyPath(1.0);

    EXPECT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);
    EXPECT_EQ(session.begin_count, 2);
    EXPECT_EQ(session.tick_count, 0);
}

TEST(FollowShadowPathActionTest, CancelsAndFailsForMissingOrEmptyPath) {
    FakeSession session;
    ShadowPathExecutor executor(session.Operations());
    int error_code = 0;
    std::string error_message;
    ASSERT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);

    session.has_path = false;
    EXPECT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::FAILURE);
    EXPECT_EQ(session.cancel_count, 1);
    EXPECT_NE(error_message.find("path"), std::string::npos);

    session.has_path = true;
    session.path.Clear();
    EXPECT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::FAILURE);
    EXPECT_EQ(session.cancel_count, 2);
}

TEST(FollowShadowPathActionTest, RejectsUnavailableFollowPathClient) {
    FakeSession session;
    session.ready = false;
    ShadowPathExecutor executor(session.Operations());
    int error_code = 0;
    std::string error_message;

    EXPECT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::FAILURE);
    EXPECT_EQ(session.begin_count, 0);
    EXPECT_EQ(session.cancel_count, 1);
    EXPECT_NE(error_message.find("ready"), std::string::npos);
}

TEST(FollowShadowPathActionTest, PropagatesFollowPathResult) {
    FakeSession session;
    ShadowPathExecutor executor(session.Operations());
    int error_code = 0;
    std::string error_message;
    ASSERT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);
    session.tick_status = BT::NodeStatus::SUCCESS;
    session.result_code = 17;
    session.result_message = "complete";

    EXPECT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::SUCCESS);
    EXPECT_EQ(error_code, 17);
    EXPECT_EQ(error_message, "complete");
}

TEST(FollowShadowPathActionTest, PropagatesFollowPathFailure) {
    FakeSession session;
    ShadowPathExecutor executor(session.Operations());
    int error_code = 0;
    std::string error_message;
    ASSERT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);
    session.tick_status = BT::NodeStatus::FAILURE;
    session.result_code = 23;
    session.result_message = "controller rejected path";

    EXPECT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::FAILURE);
    EXPECT_EQ(error_code, 23);
    EXPECT_EQ(error_message, "controller rejected path");
    EXPECT_FALSE(executor.active());
}

TEST(FollowShadowPathActionTest, HaltCancelsActiveSession) {
    FakeSession session;
    ShadowPathExecutor executor(session.Operations());
    int error_code = 0;
    std::string error_message;
    ASSERT_EQ(executor.Tick("controller", &error_code, &error_message),
              BT::NodeStatus::RUNNING);

    executor.Halt();

    EXPECT_EQ(session.cancel_count, 1);
    EXPECT_FALSE(executor.active());
}

}  // namespace
}  // namespace autonomy::task::tracking::internal
