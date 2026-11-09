/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file navigation_component.hpp
 * @brief autolink Component that drives the Navigation Runner.
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/component/component.hpp"
#include "automsgs/actions/nav_actions.pb.h"
#include "automsgs/msgs/geometry_msgs/pose_stamped.pb.h"
#include "autonomy/common/macros.hpp"
#include "navigation/proto/lifecycle.pb.h"
#include "navigation/proto/observation.pb.h"
#include "navigation/runner.hpp"

namespace navigation {

/**
 * @brief Thin autolink Component; delegates all logic to Runner.
 *
 * @par Invariants
 * - Init() returns false when the configuration or Runner::Create fails.
 * - Proc() runs on the autolink timer thread (single thread).
 *
 * @see Runner
 */
class NavigationComponent final
    : public autolink::Component<observation::Observation>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS_NOT_COPYABLE(NavigationComponent)

    /** @brief Construct an uninitialized component. */
    NavigationComponent();

    /** @brief Stop the Runner and release writers. */
    ~NavigationComponent() override;

protected:
    /**
     * @brief Load configuration and construct/start the Runner.
     *
     * @return false when configuration parsing or Runner::Create fails.
     */
    bool Init() override;

    /** @brief Stop the Runner; safe to call more than once. */
    void Clear() override;

private:
    /** @brief Execute one accepted navigate_to_pose goal. */
    void RunNavigateToPose();
    NavigationStatus MakeStatus(std::int64_t steady_ns, bool pose_valid) const;

    /**
     * @brief Forward one Observation to Runner::Tick and publish cmd_vel_raw.
     *
     * @param[in] observation Fused observation from the observation_builder.
     * @return false on null input or writer failure.
     */
    bool Proc(
        const std::shared_ptr<observation::Observation>& observation) override;

    /** @brief Business core; null until Init succeeds. */
    std::unique_ptr<Runner> runner_{nullptr};
    /** @brief Writer for /navigation/cmd_vel_raw. */
    std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::Twist>>
        command_writer_{nullptr};
    /** @brief /navigation/goal reader. */
    std::shared_ptr<autolink::Reader<GoalSpec>> goal_reader_{nullptr};
    /** @brief Autoviz 2D Goal Pose publishes PoseStamped on /goal_pose. */
    std::shared_ptr<
        autolink::Reader<automsgs::msgs::geometry_msgs::PoseStamped>>
        pose_goal_reader_{nullptr};
    std::shared_ptr<autolink::Reader<BehaviorRequest>> behavior_request_reader_;
    std::shared_ptr<autolink::Reader<lifecycle::Heartbeat>>
        behavior_heartbeat_reader_;
    std::atomic<std::int64_t> behavior_received_ns_{0};
    std::string session_id_, failure_reason_;
    /** @brief /navigation/status writer. */
    std::shared_ptr<autolink::Writer<NavigationStatus>> status_writer_{nullptr};
    /** @brief /navigation/heartbeat writer. */
    std::shared_ptr<autolink::Writer<lifecycle::Heartbeat>> heartbeat_writer_{
        nullptr};
    /** @brief Last accepted goal. */
    GoalSpec goal_;
    /** @brief Last accepted goal sequence. */
    std::uint64_t goal_sequence_{0};
    /** @brief True once a goal has been accepted. */
    bool has_goal_{false};
    /** @brief Last topic goal seen; action goals must not replay a cached
     * topic. */
    std::optional<GoalSpec> last_topic_goal_;
    /** @brief Last Autoviz /goal_pose sample applied as TYPE_POSE. */
    std::optional<automsgs::msgs::geometry_msgs::PoseStamped> last_pose_goal_;
    /** @brief Serializes goal ownership across observation and action threads.
     */
    std::mutex goal_state_mutex_;
    /** @brief Monotonic status sequence. */
    std::uint64_t status_sequence_{0};
    /** @brief Monotonic heartbeat sequence. */
    std::uint64_t heartbeat_sequence_{0};
    /** @brief /navigate_to_pose action server. */
    std::shared_ptr<autolink::action::SimpleActionServer<
        automsgs::actions::NavigateToPoseAction>>
        action_server_{nullptr};
    /** @brief True while the action server owns the goal. */
    std::atomic<bool> action_active_{false};
    /** @brief Incremented whenever a newer goal (any source) takes over. */
    std::atomic<std::uint64_t> goal_epoch_{0};
    /** @brief Latest pose for action feedback. */
    mutable std::mutex action_pose_mutex_;
    /** @brief Latest map-frame pose. */
    common::Pose2D action_pose_;
    /** @brief Latest sensor timestamp for feedback pose headers. */
    std::int64_t action_pose_stamp_ns_{0};
};

}  // namespace navigation
