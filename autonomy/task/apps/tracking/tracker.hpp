/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include "autonomy/task/apps/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/apps/tracking/tracking_client.hpp"
#include "autonomy/task/proto/tracker.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

class TrackerTask : public BtTaskApp<::autonomy::task::proto::TrackerGoal,
                                     ::autonomy::task::proto::TrackerFeedback,
                                     ::autonomy::task::proto::TrackerResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TrackerTask);

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetTrackingClient(tracking::TrackingClient::Ptr client);

protected:
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    void OnTreeTick() override;

    bool OnGoal(const ::autonomy::task::proto::TrackerGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::TrackerFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::TrackerResult* result) const override;

private:
    ::autonomy::task::proto::TrackerStatus MapStatus() const;
    bool EnsureTrackingClient();
    std::string ResolveTreeForGoal(
        const ::autonomy::task::proto::TrackerGoal& goal) const;

    tracking::TrackingClient::Ptr tracking_client_;
    std::optional<::autonomy::task::proto::TrackerGoal> active_goal_;
};

}  // namespace task
}  // namespace autonomy
