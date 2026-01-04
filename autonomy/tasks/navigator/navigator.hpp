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

#include <glog/logging.h>

#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/transform/buffer.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace autonomy {
namespace tasks {

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms and reference
 * frames.
 */
struct FeedbackUtils {
    std::string robot_frame;
    std::string global_frame;
    double transform_tolerance;
    std::shared_ptr<transform::Buffer> tf;
};

/**
 * @class NavigatorMuxer
 * @brief A class to control the state of the BT navigator by allowing only a
 * single plugin to be processed at a time.
 */
class NavigatorMuxer
{
public:
    /**
     * @brief A Navigator Muxer constructor
     */
    NavigatorMuxer() : current_navigator_("") {}

    /**
     * @brief Get the navigator muxer state
     * @return bool If a navigator is in progress
     */
    bool isNavigating() {
        std::scoped_lock l(mutex_);
        return !current_navigator_.empty();
    }

    /**
     * @brief Start navigating with a given navigator
     * @param string Name of the navigator to start
     */
    void startNavigating(const std::string& navigator_name) {
        std::scoped_lock l(mutex_);
        if (!current_navigator_.empty()) {
            LOG(ERROR)
                << "Major error! Navigation requested while another navigation"
                << " task is in progress! This likely occurred from an "
                   "incorrect"
                << "implementation of a navigator plugin.";
        }
        current_navigator_ = navigator_name;
    }

    /**
     * @brief Stop navigating with a given navigator
     * @param string Name of the navigator ending task
     */
    void stopNavigating(const std::string& navigator_name) {
        std::scoped_lock l(mutex_);
        if (current_navigator_ != navigator_name) {
            LOG(ERROR)
                << "Major error! Navigation stopped while another navigation"
                << " task is in progress! This likely occurred from an "
                   "incorrect"
                << "implementation of a navigator plugin.";
        } else {
            current_navigator_ = std::string("");
        }
    }

protected:
    std::string current_navigator_;
    std::mutex mutex_;
};

/**
 * @class Navigator
 * @brief Navigator interface that acts as a base class for all BT-based
 * Navigator action's plugins
 */
template <class ActionT>
class Navigator
{
public:
    using Ptr = std::shared_ptr<Navigator<ActionT>>;

    /**
     * @brief A Navigator constructor
     */
    Navigator() {
        plugin_muxer_ = nullptr;
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~Navigator() = default;

    /**
     * @brief Get the action name of this navigator to expose
     * @return string Name of action to expose
     */
    virtual std::string GetName() = 0;

    virtual std::string GetDefaultBTFilepath(
        std::weak_ptr<autolink::Node> node) = 0;

    /**
     * @brief Get the action server
     * @return Action server pointer
     */
    // TODO: Implement BtActionServer
    // std::unique_ptr<behavior_tree::BtActionServer<ActionT>> &
    // GetActionServer()
    // {
    //     return bt_action_server_;
    // }

protected:
    /**
     * @brief An intermediate goal reception function to mux navigators.
     */
    bool OnGoalReceived(const std::shared_ptr<typename ActionT::Goal> goal) {
        if (plugin_muxer_->isNavigating()) {
            LOG(ERROR) << "Requested navigation from " << GetName()
                       << " while another navigator is processing,"
                       << " rejecting request.";
            return false;
        }

        bool goal_accepted = GoalReceived(goal);

        if (goal_accepted) {
            plugin_muxer_->startNavigating(GetName());
        }

        return goal_accepted;
    }

    /**
     * @brief An intermediate completion function to mux navigators
     */
    void OnCompletion(
        const std::shared_ptr<typename ActionT::Result> result,
        const autonomy::tasks::behavior_tree::BtStatus final_bt_status) {
        plugin_muxer_->stopNavigating(GetName());
        GoalCompleted(result, final_bt_status);
    }

    /**
     * @brief A callback to be called when a new goal is received by the BT
     * action server Can be used to check if goal is valid and put values on the
     * blackboard which depend on the received goal
     */
    virtual bool GoalReceived(
        const std::shared_ptr<typename ActionT::Goal> goal) = 0;

    /**
     * @brief A callback that defines execution that happens on one iteration
     * through the BT Can be used to publish action feedback
     */
    virtual void OnLoop() = 0;

    /**
     * @brief A callback that is called when a preempt is requested
     */
    virtual void OnPreempt(const typename ActionT::Goal& goal) = 0;

    /**
     * @brief A callback that is called when a the action is completed; Can fill
     * in action result message or indicate that this action is done.
     */
    virtual void GoalCompleted(
        std::shared_ptr<typename ActionT::Result> result,
        const autonomy::tasks::behavior_tree::BtStatus final_bt_status) = 0;

    /**
     * @brief A configure state transition to configure navigator's state
     * @param node Shared pointer to the autolink node
     */
    bool Configure(const std::shared_ptr<autolink::Node> node) {
        return true;
    }

    /**
     * @brief A cleanup state transition to remove memory allocated
     */
    virtual bool Cleanup() {
        return true;
    }

    /**
     * @brief Method to activate any threads involved in execution.
     */
    virtual bool Activate() {
        return true;
    }

    /**
     * @brief Method to deactivate and any threads involved in execution.
     */
    virtual bool Deactivate() {
        return true;
    }

    // TODO: Implement BtActionServer
    // std::unique_ptr<behavior_tree::BtActionServer<ActionT>>
    // bt_action_server_;
    FeedbackUtils feedback_utils_;
    NavigatorMuxer* plugin_muxer_;
};

}  // namespace tasks
}  // namespace autonomy