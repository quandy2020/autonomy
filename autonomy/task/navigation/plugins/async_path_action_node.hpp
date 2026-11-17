/* Copyright 2026 The Openbot Authors */
#pragma once

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

// Shared nonblocking planning/smoothing lifecycle. Every stage is polled by the
// BT owner: acceptance, correlated result, then current-map path validation.
// No private thread, sleep or synchronous service wait is introduced.
template <typename ActionT>
class AsyncPathActionNode : public BtStatefulAction {
public:
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

    virtual Client& GetClient(NavClient& client) = 0;
    virtual bool BuildGoal(Goal& goal, Pose& endpoint) = 0;
    virtual void OutputPath(const Path& path, bool computed) = 0;
    virtual bool EmptyGoalIsComplete() const { return false; }
    virtual Path InitialFallback(const Goal&) const { return {}; }
    virtual bool UsePublishedPlan() const { return true; }
    virtual void RecordResult(const Result&) {}
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
        active_ = false;
        accept_ = {};
        result_ = {};
        validation_ = {};
        goal_handle_.reset();
        candidate_.Clear();
        checking_fallback_ = false;
        cancellation_.reset();
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
                options.goal_response_callback = [cancellation](std::shared_ptr<GoalHandle> handle) {
                    cancellation->OnAccepted(std::move(handle));
                };
                started_ = std::chrono::steady_clock::now();
                ResetResultMetadata();
                active_ = true;
                accept_ = client.AsyncSendGoal(goal, options);
                return BT::NodeStatus::RUNNING;
            }
            const auto now = std::chrono::steady_clock::now();
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
    std::shared_future<std::shared_ptr<GoalHandle>> accept_;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_future<WrappedResult> result_;
    NavClient::PathValidationFuture validation_;
};
}  // namespace autonomy::task::plugins::navigation
