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
 * Design aligned with nav2_core::behavior_tree_navigator.hpp
 * (FeedbackUtils, NavigatorMuxer, NavigatorBase, BehaviorTreeNavigator).
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_action_server.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace common {

namespace behavior_tree = autonomy::tasks::behavior_tree;

// Re-export for navigator plugins
using BtStatus = behavior_tree::BtStatus;
using OdomSmoother = autonomy::control::utils::OdomSmoother;

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities (transforms, frames).
 *        Aligned with nav2_core::FeedbackUtils.
 */
struct FeedbackUtils {
    std::string robot_frame;
    std::string global_frame;
    double transform_tolerance = 0.1;
    double local_survival_timeout = 120.0;
    std::shared_ptr<autonomy::transform::Buffer> tf;
    /** Resolved default BT XML path for this navigator (optional, set by
     * BtNavigator). */
    std::string default_bt_xml_filename;
    /** Optional resolver to convert relative BT filename to absolute path. */
    std::function<std::string(const std::string&)> bt_xml_path_resolver;
};

/**
 * @class NavigatorBase
 * @brief Abstract base for all navigator plugins (BT-based or other).
 *        Aligned with nav2_core::NavigatorBase.
 */
class NavigatorBase
{
public:
    virtual ~NavigatorBase() = default;
};

/**
 * @class NavigatorMuxer
 * @brief Ensures only one navigator is active at a time.
 *        Aligned with nav2_core::NavigatorMuxer.
 */
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
                      "navigation task is in "
                      "progress.";
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
 * @class BehaviorTreeNavigator
 * @brief Base class for all BT-based Navigator action plugins. Implements
 * NavigatorBase; final methods (Configure/Activate/Deactivate/Cleanup) delegate
 * to BtActionServer and to virtual configure/activate/deactivate/cleanup.
 * Subclasses implement GoalReceived, OnLoop, OnPreempt, GoalCompleted, GetName,
 * GetDefaultBTFilepath, and optionally configure, cleanup. Aligned with
 * nav2_core::BehaviorTreeNavigator.
 */
template <class ActionT>
class BehaviorTreeNavigator : public NavigatorBase
{
public:
    /** Default constructor for plugin loading; object must be initialized
     * before use. */
    BehaviorTreeNavigator() : NavigatorBase(), plugin_muxer_(nullptr) {}

    BehaviorTreeNavigator(std::shared_ptr<autolink::Node> node,
                          const std::vector<std::string>& plugin_lib_names,
                          const FeedbackUtils& feedback_utils,
                          const std::shared_ptr<NavigatorMuxer>& plugin_muxer,
                          std::shared_ptr<OdomSmoother> odom_smoother)
        : NavigatorBase(), plugin_muxer_(plugin_muxer.get()) {
        feedback_utils_ = feedback_utils;
        std::string default_bt_xml =
            feedback_utils.default_bt_xml_filename.empty()
                ? GetDefaultBTFilepath(node)
                : feedback_utils.default_bt_xml_filename;
        if (feedback_utils.bt_xml_path_resolver && !default_bt_xml.empty()) {
            default_bt_xml =
                feedback_utils.bt_xml_path_resolver(default_bt_xml);
        }

        action_server_ =
            std::make_unique<behavior_tree::BtActionServer<ActionT>>(
                node, GetName(), plugin_lib_names, default_bt_xml,
                std::bind(&BehaviorTreeNavigator::OnGoalReceived, this,
                          std::placeholders::_1),
                std::bind(&BehaviorTreeNavigator::OnLoop, this),
                std::bind(&BehaviorTreeNavigator::OnPreempt, this,
                          std::placeholders::_1),
                std::bind(&BehaviorTreeNavigator::OnCompletion, this,
                          std::placeholders::_1, std::placeholders::_2));

        BT::Blackboard::Ptr blackboard = action_server_->GetBlackboard();
        blackboard->set("tf_buffer", feedback_utils.tf);  // NOLINT
        blackboard->set("initial_pose_received", false);  // NOLINT
        blackboard->set("number_recoveries", 0);          // NOLINT
        blackboard->set("odom_smoother", odom_smoother);  // NOLINT
        // Provide frame defaults to BT XML ports, so trees can safely use
        // {global_frame}/{robot_base_frame} placeholders.
        blackboard->set("global_frame", feedback_utils.global_frame);      // NOLINT
        blackboard->set("robot_base_frame", feedback_utils.robot_frame);   // NOLINT
        // Default timeout (seconds) for local-survival branch in degraded
        // localization; tree may override it via blackboard remapping.
        blackboard->set("local_survival_timeout",
                        feedback_utils.local_survival_timeout);             // NOLINT
    }

    /**
     * @brief 析构
     */
    virtual ~BehaviorTreeNavigator() = default;

    /**
     * @brief 获取默认的 BT XML 文件路径
     * @return 默认的 BT XML 文件路径
     */
    virtual std::string GetDefaultBTFilepath() = 0;

    /**
     * @brief 获取 Navigator 名称
     * @return Navigator 名称
     */
    virtual std::string GetName() = 0;

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
        if (plugin_muxer_)
            plugin_muxer_->StopNavigating(GetName());
        GoalCompleted(result, final_bt_status);
    }

    virtual bool GoalReceived(
        std::shared_ptr<const typename ActionT::Goal> goal) = 0;
    virtual void OnLoop() = 0;
    virtual void OnPreempt(
        std::shared_ptr<const typename ActionT::Goal> goal) = 0;
    virtual void GoalCompleted(std::shared_ptr<typename ActionT::Result> result,
                               const BtStatus final_bt_status) = 0;

    std::unique_ptr<behavior_tree::BtActionServer<ActionT>> action_server_;
    FeedbackUtils feedback_utils_;
    NavigatorMuxer* plugin_muxer_;
};

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
