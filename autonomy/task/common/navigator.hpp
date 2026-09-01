/*
 * Copyright 2026 The Openbot Authors
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

#ifndef AUTONOMY_TASK_COMMON_NAVIGATOR_HPP_
#define AUTONOMY_TASK_COMMON_NAVIGATOR_HPP_

#include <chrono>
#include <functional>
#include <memory>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/task/navigation.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace task {
namespace common {

// Top-level navigate_to_pose / navigate_through_poses action servers.
class Navigator
{
public:
    using SubmitGoalFunction = std::function<bool(
        const ::autonomy::task::proto::NavigationGoal&)>;
    using FeedbackFunction =
        std::function<bool(::autonomy::task::proto::NavigationFeedback*)>;
    using ResultFunction =
        std::function<bool(::autonomy::task::proto::NavigationResult*)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(Navigator)

    Navigator() = default;
    ~Navigator();

    Navigator(const Navigator&) = delete;
    Navigator& operator=(const Navigator&) = delete;

    bool Start(const std::shared_ptr<autolink::Node>& node,
               SubmitGoalFunction submit_goal, FeedbackFunction feedback,
               ResultFunction result, std::chrono::milliseconds period);
    void Stop();

private:
    using NavigateToPoseAction = ::automsgs::actions::NavigateToPoseAction;
    using NavigateThroughPosesAction =
        ::automsgs::actions::NavigateThroughPosesAction;
    using NavigateToPoseServer =
        autolink::action::SimpleActionServer<NavigateToPoseAction>;
    using NavigateThroughPosesServer =
        autolink::action::SimpleActionServer<NavigateThroughPosesAction>;

    void RunNavigateToPose();
    void RunNavigateThroughPoses();
    bool AcceptNavigateToPose(const NavigateToPoseAction::Goal& goal);
    bool AcceptNavigateThroughPoses(
        const NavigateThroughPosesAction::Goal& goal);
    bool Cancel();

    static ::automsgs::msgs::status_msgs::StatusCode ToStatusCode(
        ::autonomy::task::proto::NavigationStatus status, bool rejected);

    template <typename ServerType, typename FeedbackType, typename ResultType>
    void Spin(
        const std::shared_ptr<ServerType>& server,
        std::function<bool()> accept_goal,
        std::function<void(const ::autonomy::task::proto::NavigationFeedback&,
                           FeedbackType*)> fill_feedback);

    SubmitGoalFunction submit_goal_;
    FeedbackFunction feedback_;
    ResultFunction result_;
    std::shared_ptr<autolink::Node> node_;
    std::chrono::milliseconds period_{100};
    std::shared_ptr<NavigateToPoseServer> navigate_to_pose_server_;
    std::shared_ptr<NavigateThroughPosesServer>
        navigate_through_poses_server_;
};

}  // namespace common
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_COMMON_NAVIGATOR_HPP_
