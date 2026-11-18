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

/** @file typed_task.hpp @brief Typed task lifecycle and accepted-goal identity. */

#pragma once

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

#include "autonomy/task/common/task_interface.hpp"
#include <automsgs/task/task_common.pb.h>

namespace autonomy {
namespace task {

/**
 * Shared lifecycle state machine for typed task applications.
 *
 * Derived classes implement OnGoal / FillFeedback / FillResult.
 */
template <typename GoalType, typename FeedbackType, typename ResultType>
class TypedTaskAppBase
    : public TypedTaskInterface<GoalType, FeedbackType, ResultType>
{
public:
    ~TypedTaskAppBase() override = default;

    TaskLifecycle GetLifecycle() const override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return lifecycle_;
    }

    bool Initialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override {
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            server_options_ = options;
            accepting_goals_ = true;
        }
        return OnInitialize(options);
    }

    void Shutdown() override {
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            accepting_goals_ = false;
            invalidated_admission_ = next_admission_;
        }
        Cancel();
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        lifecycle_ = TaskLifecycle::kIdle;
        progress_.Clear();
        task_id_.clear();
        client_id_.clear();
    }

    bool Cancel() override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (lifecycle_ == TaskLifecycle::kIdle ||
            IsTerminalLifecycle(lifecycle_)) {
            return false;
        }
        lifecycle_ = TaskLifecycle::kCanceled;
        return true;
    }

    bool Pause() override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (lifecycle_ != TaskLifecycle::kRunning) {
            return false;
        }
        lifecycle_ = TaskLifecycle::kPaused;
        return true;
    }

    bool Resume() override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (lifecycle_ != TaskLifecycle::kPaused) {
            return false;
        }
        lifecycle_ = TaskLifecycle::kRunning;
        return true;
    }

    ::autonomy::task::proto::ActiveTaskSnapshot GetSnapshot() const override {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        ::autonomy::task::proto::ActiveTaskSnapshot snapshot;
        snapshot.set_task_type(this->GetTaskType());
        snapshot.set_task_status(ToRobotTaskStatus(lifecycle_));
        snapshot.set_task_id(task_id_);
        snapshot.set_client_id(client_id_);
        snapshot.set_progress(progress_.progress());
        return snapshot;
    }

    /**
     * @brief Publish goal identity only after the domain accepts that goal.
     * @return False for rejection, closed admission, or an obsolete acceptance.
     *
     * OnGoal executes without mutex_ so cancellation/worker joins remain legal.
     * Domain handlers still serialize or version their own execution changes.
     * While any handler is pending, terminal reads cannot associate its changing
     * state with a previous goal. Exceptions release that read barrier and
     * propagate to the caller. A later rejected goal does not erase an earlier
     * valid acceptance; an earlier late acceptance cannot replace a newer one.
     */
    bool SubmitGoal(const GoalType& goal) override {
        return SubmitGoalWithHandler(goal, [this, &goal] { return OnGoal(goal); });
    }

    bool GetFeedback(FeedbackType* feedback) const override {
        if (feedback == nullptr) {
            return false;
        }
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        FillFeedback(feedback);
        return true;
    }

    bool GetResult(ResultType* result) const override {
        if (result == nullptr) {
            return false;
        }
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (pending_admissions_ != 0 || !IsTerminalLifecycle(lifecycle_)) {
            return false;
        }
        FillResult(result);
        return true;
    }

    /**
     * @brief Read a terminal result only for the expected goal identity.
     * @param expected_task_id Nonempty identity carried by TaskHeader.
     * @param result Destination; untouched on mismatch or nonterminal state.
     * @return True when identity and result were checked under one lock.
     *
     * A separate GetSnapshot/GetResult pair can observe different goals during
     * preemption. Consumers use this operation to reject stale completions.
     */
    bool GetResultForTask(const std::string& expected_task_id,
                         ResultType* result) const {
        if (expected_task_id.empty() || result == nullptr) return false;
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (pending_admissions_ != 0 || task_id_ != expected_task_id ||
            !IsTerminalLifecycle(lifecycle_))
            return false;
        FillResult(result);
        return true;
    }

protected:
    /**
     * @brief Retain typed admission/identity ordering for a domain-specific entry.
     * @param goal Goal whose identity is published only after handler acceptance.
     * @param handler Synchronous domain admission, invoked without mutex_.
     * @return Domain acceptance after stale-admission and shutdown checks.
     * Exceptions release the pending-result barrier before propagating. This
     * helper does not bypass the domain's lifecycle or execution serialization.
     */
    template <class Handler>
    bool SubmitGoalWithHandler(const GoalType& goal, Handler&& handler) {
        uint64_t admission;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (!accepting_goals_ ||
                next_admission_ == std::numeric_limits<uint64_t>::max() ||
                pending_admissions_ == std::numeric_limits<uint64_t>::max())
                return false;
            admission = ++next_admission_;
            ++pending_admissions_;
        }
        bool accepted;
        try {
            accepted = handler();
        } catch (...) {
            FinishAdmission(admission, goal, false);
            throw;
        }
        return FinishAdmission(admission, goal, accepted);
    }

    /**
     * @brief Decide whether acceptance replaces the active task identity.
     *
     * The default preserves headerless control requests. Domains distinguish
     * START/replacement from PAUSE/STOP and may return true for an anonymous
     * new task so its missing header clears the preceding task identity.
     * Called with mutex_ held; implementations must be bounded and nonblocking.
     */
    virtual bool ShouldUpdateGoalIdentity(const GoalType& goal) const {
        return goal.has_header();
    }

    virtual bool OnInitialize(
        const ::autonomy::task::proto::TaskServerOptions& /*options*/) {
        return true;
    }

    virtual bool OnGoal(const GoalType& goal) = 0;
    /** Called with mutex_ held; do not call Lifecycle() (re-locks). */
    virtual void FillFeedback(FeedbackType* feedback) const = 0;
    /** Called with mutex_ held; do not call Lifecycle() (re-locks). */
    virtual void FillResult(ResultType* result) const = 0;

    void SetLifecycle(TaskLifecycle lifecycle) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        lifecycle_ = lifecycle;
    }

    void SetProgress(float progress, const std::string& detail = {}) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        progress_.set_progress(progress);
        if (!detail.empty()) {
            progress_.set_detail(detail);
        }
    }

    [[nodiscard]] TaskLifecycle Lifecycle() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return lifecycle_;
    }

    [[nodiscard]] ::autonomy::task::proto::TaskResult MakeTaskResult() const {
        ::autonomy::task::proto::TaskResult envelope;
        envelope.set_status(ToRobotTaskStatus(lifecycle_));
        envelope.set_message(progress_.detail());
        return envelope;
    }

    mutable std::recursive_mutex mutex_;
    TaskLifecycle lifecycle_{TaskLifecycle::kIdle};
    ::autonomy::task::proto::TaskServerOptions server_options_;
    ::autonomy::task::proto::TaskProgress progress_;
    std::string task_id_;
    std::string client_id_;

private:
    /**
     * @brief Close one handler and commit only a current accepted identity.
     * @param admission Monotonic ticket assigned before entering OnGoal.
     * @param goal Header to publish for identity-changing domain requests.
     * @param accepted Domain acceptance, independent of terminal completion.
     */
    bool FinishAdmission(uint64_t admission, const GoalType& goal, bool accepted) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        --pending_admissions_;
        if (!accepted || !accepting_goals_ || admission <= invalidated_admission_ ||
            admission < accepted_admission_) return false;
        if (!ShouldUpdateGoalIdentity(goal)) return true;
        task_id_ = goal.has_header() ? goal.header().task_id() : std::string{};
        client_id_ = goal.has_header() ? goal.header().client_id() : std::string{};
        accepted_admission_ = admission;
        return true;
    }

    /// Monotonic admission tickets; exhaustion refuses further goals.
    uint64_t next_admission_{0};
    /// Newest accepted identity, protected by mutex_.
    uint64_t accepted_admission_{0};
    /// All tickets at or below this cutoff were invalidated by shutdown.
    uint64_t invalidated_admission_{0};
    /// Pending handlers suppress terminal reads without holding their lock.
    uint64_t pending_admissions_{0};
    /// Shutdown closes acceptance before clearing public state.
    bool accepting_goals_{true};
};

}  // namespace task
}  // namespace autonomy
