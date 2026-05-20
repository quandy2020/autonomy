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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_action_server.hpp"
#include "autonomy/tasks/common/bt_blackboard_setup.hpp"
#include "autonomy/tasks/common/feedback_utils.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace common {

namespace behavior_tree = autonomy::tasks::behavior_tree;

using BtStatus = behavior_tree::BtStatus;
using OdomSmoother = autonomy::control::utils::OdomSmoother;

class NavigatorBase
{
public:
    virtual ~NavigatorBase() = default;
};

class NavigatorMuxer
{
public:
    NavigatorMuxer() : current_navigator_("") {}

    bool IsNavigating() const {
        std::scoped_lock lock(mutex_);
        return !current_navigator_.empty();
    }

    void StartNavigating(const std::string& navigator_name) {
        std::scoped_lock lock(mutex_);
        if (!current_navigator_.empty()) {
            AERROR << "NavigatorMuxer: navigation requested while another "
                      "navigation task is in progress.";
        }
        current_navigator_ = navigator_name;
    }

    void StopNavigating(const std::string& navigator_name) {
        std::scoped_lock lock(mutex_);
        if (current_navigator_ != navigator_name) {
            AERROR << "NavigatorMuxer: stop from " << navigator_name
                   << " while current is " << current_navigator_;
        } else {
            current_navigator_.clear();
        }
    }

protected:
    std::string current_navigator_;
    mutable std::mutex mutex_;
};

/**
 * @brief BT-based navigator without middleware action server.
 */
template <class ActionT>
class BehaviorTreeNavigator : public NavigatorBase
{
public:
    BehaviorTreeNavigator() : NavigatorBase(), plugin_muxer_(nullptr) {}

    BehaviorTreeNavigator(
        std::string navigator_name, std::string default_bt_filepath,
        const autonomy::tasks::proto::TaskOptions& options,
        const std::shared_ptr<TaskContext>& task_context,
        const std::vector<std::string>& plugin_lib_names,
        const FeedbackUtils& feedback_utils,
        const std::shared_ptr<NavigatorMuxer>& plugin_muxer,
        std::shared_ptr<OdomSmoother> odom_smoother)
        : NavigatorBase(),
          plugin_muxer_(plugin_muxer.get()),
          navigator_name_(std::move(navigator_name)),
          default_bt_filepath_(std::move(default_bt_filepath)) {
        feedback_utils_ = feedback_utils;
        std::string default_bt_xml =
            feedback_utils.default_bt_xml_filename.empty()
                ? default_bt_filepath_
                : feedback_utils.default_bt_xml_filename;
        if (feedback_utils.bt_xml_path_resolver && !default_bt_xml.empty()) {
            default_bt_xml =
                feedback_utils.bt_xml_path_resolver(default_bt_xml);
        }

        const std::string plugin_lib_path = options.plugin_lib_path();
        bt_ = std::make_unique<behavior_tree::BtActionServer<ActionT>>(
            navigator_name_, plugin_lib_names, default_bt_xml, plugin_lib_path,
            std::bind(&BehaviorTreeNavigator::OnGoalReceived, this,
                      std::placeholders::_1),
            std::bind(&BehaviorTreeNavigator::OnLoop, this),
            std::bind(&BehaviorTreeNavigator::OnPreempt, this,
                      std::placeholders::_1),
            std::bind(&BehaviorTreeNavigator::OnCompletion, this,
                      std::placeholders::_1, std::placeholders::_2));

        BT::Blackboard::Ptr blackboard = bt_->GetBlackboard();
        blackboard->set("task_context", task_context);  // NOLINT
        blackboard->set("tf_buffer", feedback_utils.tf);  // NOLINT
        blackboard->set("odom_smoother", odom_smoother);  // NOLINT
        SetupNavigateToPoseBlackboard(blackboard, task_context, options,
                                      feedback_utils);

        const auto bt_loop_ms = options.bt_loop_duration() > 0
                                    ? std::chrono::milliseconds(
                                          options.bt_loop_duration())
                                    : std::chrono::milliseconds(10);
        bt_->SetLoopDuration(bt_loop_ms);

        if (!default_bt_xml.empty() && !bt_->LoadBehaviorTree(default_bt_xml)) {
            AWARN << navigator_name_
                  << ": failed to preload behavior tree: " << default_bt_xml;
        }
    }

    virtual ~BehaviorTreeNavigator() = default;

    virtual std::string GetName() const {
        return navigator_name_;
    }

    behavior_tree::BtActionServer<ActionT>& Bt() {
        return *bt_;
    }

    const behavior_tree::BtActionServer<ActionT>& Bt() const {
        return *bt_;
    }

protected:
    bool OnGoalReceived(std::shared_ptr<const typename ActionT::Goal> goal) {
        if (plugin_muxer_ && plugin_muxer_->IsNavigating()) {
            AERROR << "Requested navigation from " << GetName()
                   << " while another navigator is processing, rejecting.";
            return false;
        }
        bool goal_accepted = GoalReceived(goal);
        if (goal_accepted && plugin_muxer_) {
            plugin_muxer_->StartNavigating(GetName());
        }
        return goal_accepted;
    }

    void OnCompletion(std::shared_ptr<typename ActionT::Result> result,
                      const BtStatus final_bt_status) {
        if (plugin_muxer_) {
            plugin_muxer_->StopNavigating(GetName());
        }
        GoalCompleted(result, final_bt_status);
    }

    virtual bool GoalReceived(
        std::shared_ptr<const typename ActionT::Goal> goal) = 0;
    virtual void OnLoop() = 0;
    virtual void OnPreempt(
        std::shared_ptr<const typename ActionT::Goal> goal) = 0;
    virtual void GoalCompleted(std::shared_ptr<typename ActionT::Result> result,
                               const BtStatus final_bt_status) = 0;

    std::unique_ptr<behavior_tree::BtActionServer<ActionT>> bt_;
    FeedbackUtils feedback_utils_;
    NavigatorMuxer* plugin_muxer_;
    std::string navigator_name_;
    std::string default_bt_filepath_;
};

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
