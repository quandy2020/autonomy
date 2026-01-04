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

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/navigator/navigator.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace navigation {

/**
 * @class NavigateToPoseNavigator
 * @brief A navigator for navigating to a specified pose
 */
class NavigateToPoseNavigator
    : public Navigator<
          autonomy::tasks::behavior_tree::proto::NavigateToPoseAction>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigateToPoseNavigator)
    using ActionT = autonomy::tasks::behavior_tree::proto::NavigateToPoseAction;

    /**
     * @brief A constructor for NavigateToPoseNavigator
     */
    NavigateToPoseNavigator() : Navigator<ActionT>() {}

    /**
     * @brief A configure state transition to configure navigator's state
     * @param node Shared pointer to the autolink node
     */
    bool Configure(const std::shared_ptr<autolink::Node> node);

    /**
     * @brief A cleanup state transition to remove memory allocated
     */
    bool Cleanup() override;

    /**
     * @brief A subscription and callback to handle the topic-based goal
     * published from rviz
     * @param pose Pose received via atopic
     */
    void OnGoalPoseReceived(
        const commsgs::geometry_msgs::PoseStamped::SharedPtr pose);

    /**
     * @brief Get action name for this navigator
     * @return string Name of action server
     */
    std::string GetName() override {
        return std::string("navigate_to_pose");
    }

    /**
     * @brief Get navigator's default BT
     * @param node WeakPtr to the lifecycle node
     * @return string Filepath to default XML
     */
    std::string GetDefaultBTFilepath(
        std::weak_ptr<autolink::Node> node) override;

protected:
    /**
     * @brief A callback to be called when a new goal is received by the BT
     * action server Can be used to check if goal is valid and put values on the
     * blackboard which depend on the received goal
     * @param goal Action template's goal message
     * @return bool if goal was received successfully to be processed
     */
    bool GoalReceived(
        const std::shared_ptr<typename ActionT::Goal> goal) override;

    /**
     * @brief A callback that defines execution that happens on one iteration
     * through the BT Can be used to publish action feedback
     */
    void OnLoop() override;

    /**
     * @brief A callback that is called when a preempt is requested
     */
    void OnPreempt(const typename ActionT::Goal& goal) override;

    /**
     * @brief A callback that is called when a the action is completed, can fill
     * in action result message or indicate that this action is done.
     * @param result Action template result message to populate
     * @param final_bt_status Resulting status of the behavior tree execution
     * that may be referenced while populating the result.
     */
    void GoalCompleted(const std::shared_ptr<typename ActionT::Result> result,
                       const autonomy::tasks::behavior_tree::BtStatus
                           final_bt_status) override;

    /**
     * @brief Goal pose initialization on the blackboard
     * @param goal Action template's goal message to process
     */
    void InitializeGoalPose(const std::shared_ptr<ActionT::Goal> goal);

    commsgs::builtin_interfaces::Time start_time_;

    // rclcpp::Subscription<commsgs::geometry_msgs::PoseStamped>::SharedPtr
    // goal_sub_; rclcpp_action::Client<ActionT>::SharedPtr self_client_;

    std::string goal_blackboard_id_;
    std::string path_blackboard_id_;

    // Odometry smoother object
    // std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
};

}  // namespace navigation
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy