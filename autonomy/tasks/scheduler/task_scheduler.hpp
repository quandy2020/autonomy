/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/tasks/navigator/navigator_factory.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace scheduler {

class TaskScheduler
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskScheduler)

    TaskScheduler() = default;

    void Initialize(const std::string& configuration_directory);

    void Shutdown();

    behavior_tree::BtStatus NavigateToPose(
        std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal>
            goal);

    void RequestCancel();

    std::shared_ptr<common::TaskContext> TaskContext() const {
        return task_context_;
    }

    template <typename NavigatorT>
    std::shared_ptr<NavigatorT> GetNavigator(const std::string& id) const {
        auto it = navigators_.find(id);
        if (it == navigators_.end()) {
            return nullptr;
        }
        return std::dynamic_pointer_cast<NavigatorT>(it->second);
    }

private:
    void SetupNavigators();

    std::string configuration_directory_;
    proto::TaskOptions task_options_;
    std::shared_ptr<common::TaskContext> task_context_;
    std::shared_ptr<planning::PlannerServer> planner_;
    std::shared_ptr<planning::SmootherServer> smoother_;
    std::shared_ptr<control::ControllerServer> controller_;
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother_;
    std::unordered_map<std::string, std::shared_ptr<common::NavigatorBase>>
        navigators_;
    common::NavigatorMuxer muxer_;
    std::atomic<bool> cancel_requested_{false};
    bool initialized_{false};
};

}  // namespace scheduler
}  // namespace tasks
}  // namespace autonomy
