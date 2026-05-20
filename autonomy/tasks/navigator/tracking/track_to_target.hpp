/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace tracking {

class TrackToTargetNavigator
    : public common::BehaviorTreeNavigator<
          behavior_tree::proto::TrackToTargetAction>
{
public:
    using ActionT = behavior_tree::proto::TrackToTargetAction;

    AUTONOMY_SMART_PTR_DEFINITIONS(TrackToTargetNavigator)

    TrackToTargetNavigator(
        const proto::TaskOptions& options,
        const std::shared_ptr<common::TaskContext>& task_context,
        const std::vector<std::string>& plugin_lib_names,
        const common::FeedbackUtils& feedback_utils,
        const std::shared_ptr<common::NavigatorMuxer>& muxer,
        std::shared_ptr<common::OdomSmoother> odom_smoother);

    bool GoalReceived(std::shared_ptr<const typename ActionT::Goal> goal) override;
    void OnLoop() override;
    void OnPreempt(std::shared_ptr<const typename ActionT::Goal> goal) override;
    void GoalCompleted(std::shared_ptr<typename ActionT::Result> result,
                       const common::BtStatus final_bt_status) override;

    std::string target_pose_blackboard_id_{"target_pose"};
    uint32_t target_id_{0};
};

}  // namespace tracking
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
