// /*
//  * Copyright 2026 The Openbot Authors (duyongquan)
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *      http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  *
//  * Design aligned with nav2_bt_navigator::NavigateToPoseNavigator.
//  */

// #pragma once

// #include <memory>
// #include <string>
// #include <vector>

// #include "autolink/autolink.hpp"
// #include "autonomy/common/logging.hpp"
// #include "autonomy/common/macros.hpp"
// #include "autonomy/control/utils/odometry_utils.hpp"
// #include "autonomy/tasks/common/behavior_tree_navigator.hpp"
// #include "autonomy/tasks/navigator/proto/action.pb.h"
// #include "autonomy/tasks/proto/task_options.pb.h"

// namespace autonomy {
// namespace tasks {
// namespace navigator {
// namespace navigation {

// using autonomy::tasks::common::BtStatus;
// using autonomy::tasks::common::OdomSmoother;

// /**
//  * @class NavigateThroughPosesNavigator
//  * @brief A navigator for navigating to a a bunch of intermediary poses
//  */
// class NavigateThroughPosesNavigator
//     : public autonomy::tasks::common::BehaviorTreeNavigator<
//           autonomy::tasks::behavior_tree::proto::NavigateThroughPosesAction>
// {
// public:
//     /**
//      * Define ActionT type
//      */
//     using ActionT =
//         autonomy::tasks::behavior_tree::proto::NavigateThroughPosesAction;

//     /**
//      * Define NavigateToPoseNavigator::SharedPtr type
//      */
//     AUTONOMY_SMART_PTR_DEFINITIONS(NavigateThroughPosesNavigator)

//     /**
//      * @brief A constructor for NavigateThroughPosesNavigator
//      */
//     NavigateThroughPosesNavigator() = default;

//     /**
//      * @brief 使用节点与 TaskOptions 构造
//      * @param node 用于创建 action server、tf、odom 等的节点
//      * @param options 任务选项（navigators、plugin_lib_names、坐标系、odom
//      等）
//      */
//     NavigateThroughPosesNavigator(std::shared_ptr<::autolink::Node> node,
//                                   const
//                                   autonomy::tasks::behavior_tree::proto::
//                                       NavigateThroughPosesOptions& options);

//     /**
//      * @brief Get action name for this navigator
//      * @return string Name of action server
//      */
//     std::string GetName() override;

//     /**
//      * @brief Get default BT file path
//      * @return Default BT file path
//      */
//     std::string GetDefaultBTFilepath() override;

// protected:
//     /**
//      * @brief A callback to be called when a new goal is received by the BT
//      * action server Can be used to check if goal is valid and put values on
//      the
//      * blackboard which depend on the received goal
//      * @param goal Action template's goal message
//      * @return bool if goal was received successfully to be processed
//      */
//     bool GoalReceived(
//         std::shared_ptr<const typename ActionT::Goal> goal) override;

//     /**
//      * @brief A callback that defines execution that happens on one iteration
//      * through the BT Can be used to publish action feedback
//      */
//     void OnLoop() override;

//     /**
//      * @brief A callback that is called when a preempt is requested
//      */
//     void OnPreempt(std::shared_ptr<const typename ActionT::Goal> goal)
//     override;

//     /**
//      * @brief A callback that is called when a the action is completed, can
//      fill
//      * in action result message or indicate that this action is done.
//      * @param result Action template result message to populate
//      * @param final_bt_status Resulting status of the behavior tree execution
//      * that may be referenced while populating the result.
//      */
//     void GoalCompleted(std::shared_ptr<typename ActionT::Result> result,
//                        const BtStatus final_bt_status) override;

//     /**
//      * @brief Goal pose initialization on the blackboard
//      * @return bool if goal was initialized successfully to be processed
//      */
//     bool InitializeGoalPoses(
//         std::shared_ptr<const typename ActionT::Goal> goal);

//     std::chrono::steady_clock::time_point start_time_;
//     std::string goals_blackboard_id_;
//     std::string path_blackboard_id_;
//     std::string waypoint_statuses_blackboard_id_;

//     // Odometry smoother object
//     std::shared_ptr<OdomSmoother> odom_smoother_;
// };

// }  // namespace navigation
// }  // namespace navigator
// }  // namespace tasks
// }  // namespace autonomy
