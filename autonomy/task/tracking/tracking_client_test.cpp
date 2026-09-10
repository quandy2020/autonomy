/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/tracking/tracking_client.hpp"
#include "autonomy/task/tracking/tracking.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy::task::tracking {

class TrackingClientTestApi
{
public:
    using Clock = TrackingClient::Clock;
    using SelectionPublisher = TrackingClient::SelectionPublisher;

    static TrackingClient::Ptr Create(Clock clock,
                                      SelectionPublisher publisher) {
        auto client = std::make_shared<TrackingClient>(nullptr);
        client->clock_ = std::move(clock);
        client->selection_publisher_ = std::move(publisher);
        client->shadow_transport_enabled_ = true;
        return client;
    }

    static void ReceiveTarget(
        TrackingClient* client,
        const automsgs::msgs::geometry_msgs::PoseStamped& target) {
        client->HandleShadowTarget(
            std::make_shared<automsgs::msgs::geometry_msgs::PoseStamped>(
                target));
    }

    static void ReceivePath(TrackingClient* client,
                            const automsgs::msgs::nav_msgs::Path& path) {
        client->HandleShadowPath(
            std::make_shared<automsgs::msgs::nav_msgs::Path>(path));
    }

    static bool ShadowTransportEnabled(const TrackingClient& client) {
        return client.shadow_transport_enabled_;
    }
};

namespace {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

class TrackerTaskTestHarness : public ::autonomy::task::TrackerTask
{
public:
    void SetRunning() {
        SetLifecycle(TaskLifecycle::kRunning);
    }

    void Fill(::autonomy::task::proto::TrackerFeedback* feedback) const {
        FillFeedback(feedback);
    }

    bool InitializeClient(
        const ::autonomy::task::proto::TaskServerOptions& options) {
        return OnTreeInitialize(options);
    }

    void Populate(const BT::Blackboard::Ptr& blackboard) {
        PopulateBlackboard(blackboard);
    }

    bool Submit(const ::autonomy::task::proto::TrackerGoal& goal) {
        return OnGoal(goal);
    }
};

automsgs::msgs::geometry_msgs::PoseStamped Pose(double x, double y) {
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id("map");
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    return pose;
}

automsgs::msgs::nav_msgs::Path Path(double robot_x = 0.0,
                                    double robot_y = 0.0) {
    automsgs::msgs::nav_msgs::Path path;
    path.mutable_header()->set_frame_id("map");
    *path.add_poses() = Pose(robot_x, robot_y);
    *path.add_poses() = Pose(robot_x + 0.5, robot_y);
    return path;
}

::autonomy::task::proto::TrackerGoal PersonGoal(
    ::autonomy::task::proto::TrackerCommand command,
    const std::string& target_id) {
    ::autonomy::task::proto::TrackerGoal goal;
    goal.set_command(command);
    goal.set_mode(::autonomy::task::proto::TRACKER_MODE_PERSON);
    goal.set_target_id(target_id);
    return goal;
}

TEST(TrackingClientTest, PublishesPersonStartAndUpdateSelections) {
    TimePoint now{};
    std::vector<std::string> selections;
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [&selections](const automsgs::msgs::std_msgs::String& selection) {
            selections.push_back(selection.data());
            return true;
        });

    EXPECT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
    EXPECT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_UPDATE_TARGET, "")));

    ASSERT_EQ(selections.size(), 2U);
    EXPECT_EQ(selections[0], "track-7");
    EXPECT_TRUE(selections[1].empty());
}

TEST(TrackingClientTest, RejectsPersonGoalWhenSelectionCannotBePublished) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return false; });

    EXPECT_FALSE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
    EXPECT_EQ(client->mode(),
              ::autonomy::task::proto::TRACKER_MODE_UNSPECIFIED);
    EXPECT_TRUE(client->target_id().empty());
}

TEST(TrackingClientTest, PreservesSynchronousSelectionResponse) {
    TimePoint now{};
    TrackingClient::Ptr client;
    client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [&client](const automsgs::msgs::std_msgs::String&) {
            TrackingClientTestApi::ReceiveTarget(client.get(), Pose(1.0, 0.0));
            TrackingClientTestApi::ReceivePath(client.get(), Path());
            return true;
        });

    ASSERT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
    EXPECT_TRUE(client->IsTargetLocked());
}

TEST(TrackingClientTest, ReturnsFreshShadowTargetPathAndMonotonicRevision) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return true; });
    ASSERT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));

    TrackingClientTestApi::ReceiveTarget(client.get(), Pose(3.0, 4.0));
    TrackingClientTestApi::ReceivePath(client.get(), Path());

    automsgs::msgs::geometry_msgs::PoseStamped target;
    automsgs::msgs::nav_msgs::Path path;
    uint64_t revision = 0;
    EXPECT_TRUE(client->GetShadowTarget(&target));
    EXPECT_TRUE(client->GetShadowPath(&path, &revision));
    EXPECT_EQ(revision, 1U);
    EXPECT_TRUE(client->IsTargetLocked());

    TrackingClientTestApi::ReceivePath(client.get(), Path(0.25, 0.0));
    EXPECT_TRUE(client->GetShadowPath(&path, &revision));
    EXPECT_EQ(revision, 2U);

    float distance = 0.0F;
    EXPECT_TRUE(client->GetShadowDistanceToTarget(&distance));
    EXPECT_FLOAT_EQ(distance, 4.8541219F);
}

TEST(TrackingClientTest, RejectsStaleShadowData) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return true; });
    ASSERT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
    TrackingClientTestApi::ReceiveTarget(client.get(), Pose(1.0, 0.0));
    TrackingClientTestApi::ReceivePath(client.get(), Path());

    now += TrackingClient::kFollowDataTimeout + std::chrono::milliseconds(1);

    automsgs::msgs::geometry_msgs::PoseStamped target;
    automsgs::msgs::nav_msgs::Path path;
    uint64_t revision = 0;
    EXPECT_FALSE(client->GetShadowTarget(&target));
    EXPECT_FALSE(client->GetShadowPath(&path, &revision));
    EXPECT_FALSE(client->IsTargetLocked());
}

TEST(TrackingClientTest, RejectsFreshPathWhenTargetIsStale) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return true; });
    ASSERT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
    TrackingClientTestApi::ReceiveTarget(client.get(), Pose(1.0, 0.0));
    TrackingClientTestApi::ReceivePath(client.get(), Path());

    now += TrackingClient::kFollowDataTimeout + std::chrono::milliseconds(1);
    TrackingClientTestApi::ReceivePath(client.get(), Path(0.25, 0.0));

    automsgs::msgs::nav_msgs::Path path;
    uint64_t revision = 0;
    EXPECT_FALSE(client->GetShadowPath(&path, &revision));
    EXPECT_EQ(revision, 2U);
    EXPECT_FALSE(client->IsTargetLocked());
}

TEST(TrackingClientTest, EmptyPathClearsPersonLockAndAdvancesRevision) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return true; });
    ASSERT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
    TrackingClientTestApi::ReceiveTarget(client.get(), Pose(1.0, 0.0));
    TrackingClientTestApi::ReceivePath(client.get(), Path());
    TrackingClientTestApi::ReceivePath(client.get(),
                                       automsgs::msgs::nav_msgs::Path{});

    automsgs::msgs::geometry_msgs::PoseStamped target;
    automsgs::msgs::nav_msgs::Path path;
    uint64_t revision = 0;
    EXPECT_FALSE(client->GetShadowTarget(&target));
    EXPECT_FALSE(client->GetShadowPath(&path, &revision));
    EXPECT_EQ(revision, 2U);
    EXPECT_FALSE(client->IsTargetLocked());
}

TEST(TrackingClientTest, RequestedPersonIdAloneDoesNotProveLock) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return true; });

    ASSERT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));

    EXPECT_FALSE(client->IsTargetLocked());
}

TEST(TrackingClientTest, PersonFeedbackUsesShadowAndRemainsReacquirable) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return true; });
    ASSERT_TRUE(client->ApplyGoal(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
    TrackingClientTestApi::ReceiveTarget(client.get(), Pose(3.0, 4.0));
    TrackingClientTestApi::ReceivePath(client.get(), Path());
    TrackerTaskTestHarness task;
    task.SetTrackingClient(client);
    task.SetRunning();

    ::autonomy::task::proto::TrackerFeedback feedback;
    task.Fill(&feedback);
    EXPECT_EQ(feedback.status(),
              ::autonomy::task::proto::TRACKER_STATUS_TRACKING);
    EXPECT_TRUE(feedback.has_target_pose());
    EXPECT_FLOAT_EQ(feedback.distance_to_target(), 5.0F);

    now += TrackingClient::kFollowDataTimeout + std::chrono::milliseconds(1);
    feedback.Clear();
    task.Fill(&feedback);
    EXPECT_EQ(feedback.status(),
              ::autonomy::task::proto::TRACKER_STATUS_TARGET_LOST);
    EXPECT_FALSE(feedback.has_target_pose());

    TrackingClientTestApi::ReceiveTarget(client.get(), Pose(2.0, 0.0));
    TrackingClientTestApi::ReceivePath(client.get(), Path());
    feedback.Clear();
    task.Fill(&feedback);
    EXPECT_EQ(feedback.status(),
              ::autonomy::task::proto::TRACKER_STATUS_TRACKING);
}

TEST(TrackingClientTest, NavigationOnlyConstructionNeedsNoShadowTransport) {
    // The factory only stores this opaque navigation handle in this test; the
    // aliasing pointer is never dereferenced.
    auto owner = std::make_shared<int>(0);
    auto navigation = navigation::NavigationClient::Ptr(
        owner, reinterpret_cast<navigation::NavigationClient*>(owner.get()));

    const auto client = TrackingClient::Create(std::move(navigation));

    ASSERT_NE(client, nullptr);
    EXPECT_FALSE(TrackingClientTestApi::ShadowTransportEnabled(*client));
}

TEST(TrackingClientTest, TrackerTaskInitializationUsesInjectedNavigationOnly) {
    // The factory only stores this opaque navigation handle in this test; the
    // aliasing pointer is never dereferenced.
    auto owner = std::make_shared<int>(0);
    auto navigation = navigation::NavigationClient::Ptr(
        owner, reinterpret_cast<navigation::NavigationClient*>(owner.get()));
    TrackerTaskTestHarness task;
    task.SetNavigationClient(std::move(navigation));

    ::autonomy::task::proto::TaskServerOptions options;
    EXPECT_TRUE(task.InitializeClient(options));
}

TEST(TrackingClientTest, ReacquirePolicyIsExportedToBlackboard) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return true; });
    TrackerTaskTestHarness task;
    task.SetTrackingClient(client);
    auto blackboard = BT::Blackboard::create();
    auto goal =
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7");

    goal.set_reacquire_on_lost(true);
    ASSERT_TRUE(client->ApplyGoal(goal));
    task.Populate(blackboard);
    int attempts = 0;
    ASSERT_TRUE(blackboard->get("reacquire_attempts", attempts));
    EXPECT_EQ(attempts, -1);

    goal.set_reacquire_on_lost(false);
    ASSERT_TRUE(client->ApplyGoal(goal));
    task.Populate(blackboard);
    ASSERT_TRUE(blackboard->get("reacquire_attempts", attempts));
    EXPECT_EQ(attempts, 1);
}

TEST(TrackingClientTest, TrackerTaskRejectsFailedPersonSelection) {
    TimePoint now{};
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [](const automsgs::msgs::std_msgs::String&) { return false; });
    TrackerTaskTestHarness task;
    task.SetTrackingClient(client);

    EXPECT_FALSE(task.Submit(
        PersonGoal(::autonomy::task::proto::TRACKER_CMD_START, "track-7")));
}

TEST(TrackingClientTest, TargetPoseModeRetainsExistingFollowGoalBehavior) {
    TimePoint now{};
    int selection_count = 0;
    auto client = TrackingClientTestApi::Create(
        [&now]() { return now; },
        [&selection_count](const automsgs::msgs::std_msgs::String&) {
            ++selection_count;
            return true;
        });
    ::autonomy::task::proto::TrackerGoal goal;
    goal.set_command(::autonomy::task::proto::TRACKER_CMD_START);
    goal.set_mode(::autonomy::task::proto::TRACKER_MODE_TARGET_POSE);
    goal.set_follow_distance(1.5F);
    *goal.mutable_target_pose() = Pose(4.0, 2.0);

    ASSERT_TRUE(client->ApplyGoal(goal));

    automsgs::msgs::geometry_msgs::PoseStamped follow_goal;
    ASSERT_TRUE(client->ComputeFollowGoal(follow_goal));
    EXPECT_DOUBLE_EQ(follow_goal.pose().position().x(), 2.5);
    EXPECT_DOUBLE_EQ(follow_goal.pose().position().y(), 2.0);
    EXPECT_EQ(selection_count, 0);
}

}  // namespace
}  // namespace autonomy::task::tracking
