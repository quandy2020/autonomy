/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>
#include <optional>

#include "autonomy/task/apps/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/apps/localization/localization_client.hpp"
#include "autonomy/task/proto/localization.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

class LocalizationTask
    : public BtTaskApp<::autonomy::task::proto::LocalizationGoal,
                         ::autonomy::task::proto::LocalizationFeedback,
                         ::autonomy::task::proto::LocalizationResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(LocalizationTask);

    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

    void SetLocalizationClient(localization::LocalizationClient::Ptr client);

protected:
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    void OnTreeTick() override;

    bool OnGoal(const ::autonomy::task::proto::LocalizationGoal& goal) override;
    void FillFeedback(
        ::autonomy::task::proto::LocalizationFeedback* feedback) const override;
    void FillResult(
        ::autonomy::task::proto::LocalizationResult* result) const override;

private:
    ::autonomy::task::proto::LocalizationStatus MapStatus() const;
    bool EnsureLocalizationClient();

    localization::LocalizationClient::Ptr localization_client_;
    std::optional<::autonomy::task::proto::LocalizationGoal> active_goal_;
};

}  // namespace task
}  // namespace autonomy
