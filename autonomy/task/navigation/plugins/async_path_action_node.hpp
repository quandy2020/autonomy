/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file async_path_action_node.hpp
 * @brief Native nonblocking plan acceptance, correlated results and map validation.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_PLUGINS_ASYNC_PATH_ACTION_NODE_HPP_
#define AUTONOMY_TASK_NAVIGATION_PLUGINS_ASYNC_PATH_ACTION_NODE_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/path_update.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

/** @brief Shared planning/smoothing lifecycle polled by the native BT owner.
 * Each stage retains its own acceptance, result and map-validation evidence.
 */
template <typename ActionT>
/**
 * @class AsyncPathActionNode
 * @brief Base node that drives one plan/act action asynchronously.
 *
 * The node sends the goal on the first tick, polls the action result on every
 * following tick and cancels the request when the tree is halted, so a preempt
 * or a shutdown never leaves a native goal running in the background.
 * Subclasses only choose the client, build the goal and publish the produced
 * path.
 */
class AsyncPathActionNode : public BtStatefulAction {
public:
    /**
     * @brief Constructs the node and binds it to the tree node config.
     * @param[in] name Node name from the tree XML.
     * @param[in] config Ports and blackboard supplied by the tree.
     */
    AsyncPathActionNode(const std::string& name, const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}
    ~AsyncPathActionNode() override { CancelRequest(); }

protected:
    using Goal = typename ActionT::Goal;
    using Client = common::TaskActionClient<ActionT>;
    using GoalHandle = typename Client::GoalHandle;
    using WrappedResult = typename Client::WrappedResult;
    using Result = typename ActionT::Result;
    using Path = automsgs::msgs::nav_msgs::Path;
    using Pose = automsgs::msgs::geometry_msgs::PoseStamped;
    using NavClient = ::autonomy::task::navigation::NavigationClient;

    /**
     * @brief Selects the action client this node drives.
     * @param[in] client Navigation client owning every action client.
     * @return Reference to the action client used by this node.
     */
    virtual Client& GetClient(NavClient& client) = 0;
    /**
     * @brief Fills the action goal from the node ports.
     * @param[out] goal Goal sent to the action server.
     * @param[out] endpoint Final pose, used to decide whether the goal is empty.
     * @return false when the ports do not describe a usable goal.
     */
    virtual bool BuildGoal(Goal& goal, Pose& endpoint) = 0;
    /**
     * @brief Publishes the path produced or accepted by this node.
     * @param[in] path Path to write to the output port.
     * @param[in] computed true when the path came from the server, false when it
     *            is the node's initial fallback.
     */
    virtual void OutputPath(const Path& path, bool computed) = 0;
    /**
     * @brief Reports whether an empty goal should succeed immediately.
     * @return true when the node treats "nothing to do" as success.
     */
    virtual bool EmptyGoalIsComplete() const { return false; }
    /**
     * @brief Path published while the server is still computing.
     * @param[in] goal Goal being executed.
     * @return Fallback path, usually empty.
     */
    virtual Path InitialFallback(const Goal& goal) const { return {}; }
    /**
     * @brief Selects whether the node consumes the published /plan message.
     * @return true when the node waits for the published plan as well.
     */
    virtual bool UsePublishedPlan() const { return true; }
    /**
     * @brief Stores the terminal action result in the node state.
     * @param[in] result Result returned by the action server.
     */
    virtual void RecordResult(const Result& result) {}
    /**
     * @brief Clears per-goal result metadata before a new goal is sent.
     */
    virtual void ResetResultMetadata() {}

    int GoalEpoch() const { return Epoch(); }

    BT::NodeStatus OnFirstTick() override { return Step(); }
    BT::NodeStatus OnExecute() override { return Step(); }
    void OnHalted() override { CancelRequest(); }

private:
    int Epoch() const {
        int epoch = 0;
        if (config().blackboard) config().blackboard->get("goal_epoch", epoch);
        return epoch;
    }
    void CancelRequest() {
        if (cancellation_) cancellation_->RequestCancel();
        // Published /plan can finish the local request before the native result.
        // Release only planning registration; late acceptance still honors the
        // transport-owned cancellation state configured for this exact UUID.
        if (ticket_) ticket_->RetirePlanning();
        active_ = false;
        accept_ = {};
        result_ = {};
        validation_ = {};
        goal_handle_.reset();
        candidate_.Clear();
        checking_fallback_ = false;
        cancellation_.reset();
        ticket_.reset();
    }
    BT::NodeStatus Fail(const std::string& reason) {
        CancelRequest();
        // Keep the last good path so FollowPath is not canceled when a
        // RateController replan collides or times out.
        try {
            if (!last_good_.poses().empty() && last_good_epoch_ == Epoch()) {
                OutputPath(last_good_, false);
            } else {
                OutputPath(Path{}, false);
            }
        } catch (const std::exception&) {}
        SetErrorPorts(*this, 1, reason);
        AWARN_EVERY(20) << "nav " << name() << " fail " << reason;
        return BT::NodeStatus::FAILURE;
    }
    BT::NodeStatus AcceptPath(NavClient& nav, bool computed) {
        if (ticket_ && ticket_->Revoked()) return Fail("planning dispatch revision revoked");
        if (Epoch() != epoch_ || nav.planning_generation() != generation_)
            return Fail("path completed for a superseded goal");
        OutputPath(candidate_, computed);
        if (computed) {
            last_good_ = candidate_;
            last_good_epoch_ = epoch_;
            last_good_identity_ = identity_;
            last_good_time_ = std::chrono::steady_clock::now();
        }
        CancelRequest();
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
    bool BeginFallback(NavClient& client, const Goal& goal) {
        if (checking_fallback_) return false;
        Path fallback;
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_good_time_).count();
        double max_age = 2.0;
        getInput("max_path_reuse_age", max_age);
        if (last_good_epoch_ == epoch_ && last_good_identity_ == identity_ &&
            std::isfinite(max_age) && max_age >= 0.0 && age <= max_age)
            fallback = last_good_;
        if (fallback.poses().empty()) fallback = InitialFallback(goal);
        if (fallback.poses().empty() ||
            !::autonomy::task::navigation::SameGoalPose(
                fallback.poses(fallback.poses_size()-1), endpoint_, 0.10, 0.10)) return false;
        // Cancel the failed operation before validating a retained path.
        if (cancellation_) cancellation_->RequestCancel();
        candidate_ = std::move(fallback);
        validation_ = client.ValidatePathAsync(candidate_);
        checking_fallback_ = true;
        validation_started_ = std::chrono::steady_clock::now();
        return validation_.valid();
    }
    BT::NodeStatus Step() {
        try {
            auto nav = ResolveClient(*this);
            auto& client = GetClient(*nav);
            Goal goal;
            Pose endpoint;
            if (!BuildGoal(goal, endpoint)) {
                CancelRequest();
                if (EmptyGoalIsComplete()) {
                    ClearErrorPorts(*this);
                    return BT::NodeStatus::SUCCESS;
                }
                return Fail("missing/empty planning input or final pose not reached");
            }
            if (!::autonomy::task::navigation::SameGoalPose(endpoint, endpoint, 0.0, 0.0))
                return Fail("invalid planning endpoint");
            const auto identity = goal.SerializeAsString();
            if (active_ && (identity != identity_ || Epoch() != epoch_ ||
                            nav->planning_generation() != generation_)) {
                CancelRequest();
                OutputPath(Path{}, false);
            }
            if (!active_ && !last_good_.poses().empty() &&
                last_good_epoch_ == Epoch() && last_good_identity_ == identity) {
                double max_age = 2.0;
                getInput("max_path_reuse_age", max_age);
                const double age = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - last_good_time_).count();
                if (std::isfinite(max_age) && max_age >= 0.0 && age <= max_age &&
                    ::autonomy::task::navigation::SameGoalPose(
                        last_good_.poses(last_good_.poses_size() - 1), endpoint,
                        0.10, 0.10)) {
                    OutputPath(last_good_, false);
                    ClearErrorPorts(*this);
                    return BT::NodeStatus::SUCCESS;
                }
            }
            if (!active_) {
                if (!client.ActionServerIsReady()) return BT::NodeStatus::RUNNING;
                endpoint_ = endpoint;
                identity_ = identity;
                epoch_ = Epoch();
                generation_ = nav->planning_generation();
                request_id_ = nav->NewPlanRequestId();
                goal.set_request_id(request_id_);
                cancellation_ = client.MakeGoalCancellation();
                typename Client::SendGoalOptions options;
                const auto cancellation = cancellation_;
                cancellation->Configure(options);
                if (auto gate = ResolveMotionGate(*this)) {
                    ticket_ = gate->Reserve([cancellation] { cancellation->RequestCancel(); },
                        [cancellation] { return cancellation->HasFailure(); },
                        common::MotionDispatchGate::Kind::kPlanning);
                    if (!ticket_) return Fail("planning dispatch admission revoked");
                    const auto ticket = ticket_;
                    cancellation->SetTerminalCallback([ticket] { ticket->Finish(); });
                    if (!ticket->Start()) return Fail("planning dispatch revision revoked");
                }
                started_ = std::chrono::steady_clock::now();
                ResetResultMetadata();
                active_ = true;
                accept_ = client.AsyncSendGoal(goal, options);
                return BT::NodeStatus::RUNNING;
            }
            const auto now = std::chrono::steady_clock::now();
            if (ticket_ && ticket_->Revoked()) return Fail("planning dispatch revision revoked");
            if (validation_.valid()) {
                if (now - validation_started_ > std::chrono::seconds(1)) return Fail("path validation timeout");
                if (validation_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
                    return BT::NodeStatus::RUNNING;
                const auto valid = validation_.get();
                if (!valid || !valid->is_valid()) return Fail("returned/retained path is not safe in current map");
                return AcceptPath(*nav, !checking_fallback_);
            }
            if (now - started_ > std::chrono::seconds(8)) {
                if (BeginFallback(*nav, goal)) return BT::NodeStatus::RUNNING;
                return Fail("planning request timeout");
            }
            if (UsePublishedPlan() &&
                nav->TryGetCorrelatedPlan(endpoint_, started_, request_id_, candidate_)) {
                // Planner already lethal-validated. Re-checking with 253 /
                // unknown-as-obstacle rejected every Habitat corridor path.
                return AcceptPath(*nav, true);
            }
            if (!goal_handle_) {
                if (!accept_.valid()) return Fail("invalid planning acceptance future");
                if (accept_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
                    return BT::NodeStatus::RUNNING;
                goal_handle_ = accept_.get();
                if (!goal_handle_) return Fail("planning goal rejected");
                result_ = goal_handle_->AsyncGetResult();
            }
            if (result_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
                return BT::NodeStatus::RUNNING;
            const auto result = result_.get();
            if (result.code != autolink::action::ResultCode::SUCCEEDED || !result.result ||
                result.result->error_code() != 0 ||
                result.result->request_id() != request_id_ || result.result->path().poses().empty()) {
                if (BeginFallback(*nav, goal)) return BT::NodeStatus::RUNNING;
                return Fail("planning action failed or returned an uncorrelated path");
            }
            candidate_ = result.result->path();
            Pose result_end = candidate_.poses(candidate_.poses_size() - 1);
            if (result_end.header().frame_id().empty()) {
                result_end.mutable_header()->set_frame_id(
                    candidate_.header().frame_id());
            }
            Pose expected = endpoint_;
            if (expected.header().frame_id().empty()) {
                expected.mutable_header()->set_frame_id(
                    candidate_.header().frame_id());
            }
            // SmoothPath / NavFn often leave pose frames empty while the path
            // header is "map". Do not fail the pipeline: keep the unsmoothed
            // input so FollowPath can run.
            if (!::autonomy::task::navigation::CompatibleFrames(
                    candidate_.header().frame_id(), expected.header().frame_id()) ||
                !::autonomy::task::navigation::SameGoalPose(
                    result_end, expected, 0.50, 1.0)) {
                Path fallback = InitialFallback(goal);
                if (fallback.poses().empty()) {
                    return Fail("planning result endpoint/frame mismatch");
                }
                AWARN << "nav " << name()
                      << " keep_input poses=" << fallback.poses_size()
                      << " reason=endpoint_or_frame";
                candidate_ = std::move(fallback);
            }
            RecordResult(*result.result);
            return AcceptPath(*nav, true);
        } catch (const std::exception& ex) {
            return Fail(ex.what());
        }
    }

    bool active_{false}, checking_fallback_{false};
    int epoch_{0}, last_good_epoch_{-1};
    uint64_t generation_{0};
    std::string request_id_, identity_, last_good_identity_;
    Pose endpoint_;
    Path candidate_, last_good_;
    std::chrono::steady_clock::time_point started_{}, validation_started_{}, last_good_time_{};
    std::shared_ptr<typename Client::GoalCancellation> cancellation_;
    std::shared_ptr<common::MotionDispatchGate::Ticket> ticket_; ///< Planning-only send ownership.
    std::shared_future<std::shared_ptr<GoalHandle>> accept_;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_future<WrappedResult> result_;
    NavClient::PathValidationFuture validation_;
};
}  // namespace autonomy::task::plugins::navigation

#endif  // AUTONOMY_TASK_NAVIGATION_PLUGINS_ASYNC_PATH_ACTION_NODE_HPP_
