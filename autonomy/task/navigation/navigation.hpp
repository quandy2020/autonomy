/*
 * Copyright 2026 The Openbot Authors
 *
 * NavigationTask — behavior-tree navigator for single-pose and through-poses
 * goals. NavigationClient delegates RPCs; TaskServer owns exclusive activation.
 */
/** @file navigation.hpp @brief Versioned native navigation admission and cancellation. */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "autonomy/task/behavior_tree/bt_task_app.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include <automsgs/task/navigation.pb.h>
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace task {

/** @brief Admit validated goals, preserve accepted identity and revoke stale ticks. */
class NavigationTask : public BtTaskApp<
                           ::autonomy::task::proto::NavigationGoal,
                           ::autonomy::task::proto::NavigationFeedback,
                           ::autonomy::task::proto::NavigationResult>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(NavigationTask);

    /// Close navigation before releasing the runner or client dependencies.
    ~NavigationTask() override;
    /// Revoke pending admission before waiting for worker cleanup.
    void Shutdown() override;
    /// Interrupt readiness and stop the current invocation without its wait lock.
    bool Cancel() override;
    /// Pause the tree and revoke its current motion eligibility.
    bool Pause() override;
    /// Resume a paused invocation after restoring its current tick eligibility.
    bool Resume() override;
    /** @brief Cancel only the matching pending or accepted navigation identity. */
    bool CancelForTask(const std::string& expected_task_id);

    /// Native task classification used by TaskServer and typed snapshots.
    ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const override;

protected:

    /// Enable a fresh domain generation only after its dependency is available.
    bool OnTreeInitialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override;

    /// Populate the joined invocation's geometry, identity, plugins and epoch.
    void PopulateBlackboard(const BT::Blackboard::Ptr& blackboard) override;
    /// Refuse ticks from canceled/shutdown invocations; dispatch has a separate gate.
    bool BeforeTreeTick() override;
    /// Publish only the current invocation's progress and terminal cleanup.
    void OnTreeTick() override;

    /// START/REPLAN replace identity, including anonymous requests; controls do not.
    bool ShouldUpdateGoalIdentity(
        const ::autonomy::task::proto::NavigationGoal& goal) const override;
    /**
     * @brief Validate/admit START/REPLAN or execute an identity-matched control.
     * @return Domain acceptance, not successful arrival or physical-stop ACK.
     * Readiness failure preserves accepted work; superseded requests refuse.
     */
    bool OnGoal(const ::autonomy::task::proto::NavigationGoal& goal) override;
    /// Fill feedback while the typed host holds its lifecycle mutex.
    void FillFeedback(
        ::autonomy::task::proto::NavigationFeedback* feedback) const override;
    /// Fill a terminal result after the typed host verifies accepted identity.
    void FillResult(
        ::autonomy::task::proto::NavigationResult* result) const override;

    /// RPC dependency seam; production requires a configured NavigationClient.
    virtual bool EnsureNavigationClient();
    /// Bounded readiness probe; no navigation lifecycle/goal mutex is held.
    virtual bool NavigationServersReady() const;
    /// Default admission deadline; overridable by an embedding application's policy.
    virtual std::chrono::milliseconds NavigationReadyTimeout() const;
    /// Cancel remote work after revoking ticks; production client remains asynchronous.
    virtual void CancelRemoteNavigation();

private:
    ::autonomy::task::proto::NavigationStatus MapStatus() const;
    std::string ResolveTreeForGoal(
        const ::autonomy::task::proto::NavigationGoal& goal) const;
    /// Release only this request's pending slot; newer requests retain their slot.
    void FinishPendingGoal(uint64_t epoch);
    /// Cancel matching work; an empty identity requests unconditional cancellation.
    bool CancelMatchingGoal(const std::string& expected_task_id);
    /// Pause only matching work; empty identity applies to all current work.
    bool PauseMatchingGoal(const std::string& expected_task_id);
    /// Resume only the matching retained invocation, after a fresh eligibility check.
    bool ResumeMatchingGoal(const std::string& expected_task_id);
    void ApplyGoalToBlackboard(
        const BT::Blackboard::Ptr& blackboard,
        const ::autonomy::task::proto::NavigationGoal& goal);

    /// Active tree's goal, protected by the inherited lifecycle mutex.
    std::optional<::autonomy::task::proto::NavigationGoal> active_goal_;
    /// Most recent admission still waiting for readiness; never published as accepted.
    std::optional<::autonomy::task::proto::NavigationGoal> pending_goal_;
    /// Epoch of pending_goal_, protected by mutex_.
    uint64_t pending_epoch_{0};
    /// Epoch of active_goal_, cleared after terminal processing.
    uint64_t active_epoch_{0};
    /// Epoch already being canceled, preventing recursive remote cancellation.
    uint64_t canceling_epoch_{0};
    /// Blackboard planning revision, protected by mutex_.
    uint64_t replan_epoch_{0};
    /// Distance baseline for the current accepted invocation, protected by mutex_.
    float initial_distance_{-1.f};
    /// Serialize tree replacement/control; readiness and revocation never hold it.
    mutable std::mutex goal_mutex_;
    /// Latest request generation; pending starts abandon stale generations.
    std::atomic<uint64_t> goal_epoch_{0};
    /// Invocation token installed only after the preceding worker has joined.
    std::atomic<uint64_t> tree_epoch_{0};
    /// Current invocation eligibility; zero after pause/cancel/terminal cleanup.
    std::atomic<uint64_t> motion_epoch_{0};
    /// Protect condition-variable waiting, independently of task state and tree joins.
    std::mutex ready_mutex_;
    /// Wake pending readiness probes on cancellation or replacement.
    std::condition_variable ready_wake_;
    /// Refuse queued requests after shutdown until a fresh initialization.
    std::atomic<bool> shutting_down_{false};
    /// A cancellation exception prohibits subsequent motion until reinitialization.
    std::atomic<bool> motion_fault_{false};
};

}  // namespace task
}  // namespace autonomy
