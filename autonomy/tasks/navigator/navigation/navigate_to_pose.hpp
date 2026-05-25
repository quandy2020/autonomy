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
 *
 * Design aligned with nav2_bt_navigator::NavigateToPoseNavigator.
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/common/bt_navigator.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"
#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace navigation {

using autonomy::tasks::common::BtStatus;
using autonomy::tasks::common::OdomSmoother;

/**
 * @class NavigateToPoseNavigator
 * @brief 导航到指定位姿的 Navigator，使用行为树实现（与 nav2
 * NavigateToPoseNavigator 对齐）。
 */
class NavigateToPoseNavigator
    : public autonomy::tasks::common::BtNavigator<
          autonomy::tasks::proto::NavigateToPoseAction>
{
public:
    /**
     * Define ActionT type
     */
    using ActionT = autonomy::tasks::proto::NavigateToPoseAction;

    /**
     * Define NavigateToPoseNavigator::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigateToPoseNavigator)

    /** Default constructor for plugin loading; use (node, options) ctor for
     * normal construction. */
    NavigateToPoseNavigator() = default;

    /**
     * @brief 使用 TaskOptions 构造
     * @param options 任务选项（navigators、plugin_lib_names、坐标系、odom 等）
     */
    NavigateToPoseNavigator(
        const autonomy::tasks::proto::TaskOptions& options,
        const std::shared_ptr<autonomy::tasks::common::TaskContext>& task_context,
        const std::vector<std::string>& plugin_lib_names,
        const autonomy::tasks::common::FeedbackUtils& feedback_utils,
        const std::shared_ptr<autonomy::tasks::common::NavigatorMuxer>& muxer,
        std::shared_ptr<OdomSmoother> odom_smoother);

    bool GoalReceived(
        std::shared_ptr<const typename ActionT::Goal> goal) override;
    void OnLoop() override;
    void OnPreempt(std::shared_ptr<const typename ActionT::Goal> goal) override;
    void GoalCompleted(std::shared_ptr<typename ActionT::Result> result,
                       const BtStatus final_bt_status) override;

    /**
     * @brief 将 goal 位姿变换到 global_frame 并写入 blackboard，重置
     * number_recoveries
     * @return 成功返回 true
     */
    bool InitializeGoalPose(std::shared_ptr<const typename ActionT::Goal> goal);

    std::shared_ptr<OdomSmoother> ActiveOdomSmoother() const;

    std::chrono::steady_clock::time_point start_time_;
    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;
    std::shared_ptr<OdomSmoother> odom_smoother_;
};

}  // namespace navigation
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
