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

#pragma once

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
        }
        return OnInitialize(options);
    }

    void Shutdown() override {
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

    bool SubmitGoal(const GoalType& goal) override {
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (goal.has_header()) {
                task_id_ = goal.header().task_id();
                client_id_ = goal.header().client_id();
            }
        }
        // Run OnGoal outside the lifecycle mutex so preemption can StopTree
        // (join BT worker) without deadlocking feedback / tick callbacks.
        return OnGoal(goal);
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
        if (!IsTerminalLifecycle(lifecycle_)) {
            return false;
        }
        FillResult(result);
        return true;
    }

protected:
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
};

}  // namespace task
}  // namespace autonomy
