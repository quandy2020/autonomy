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
#include "autonomy/task/proto/task_common.pb.h"

namespace autonomy {
namespace task {

/**
 * @brief TypedTaskInterface 公共状态机骨架，各 apps/* 继承并实现 OnGoal。
 */
template <typename GoalT, typename FeedbackT, typename ResultT>
class TypedTaskAppBase : public TypedTaskInterface<GoalT, FeedbackT, ResultT>
{
public:
    ~TypedTaskAppBase() override = default;

    TaskLifecycle GetLifecycle() const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lifecycle_;
    }

    bool Initialize(
        const ::autonomy::task::proto::TaskServerOptions& options) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        server_options_ = options;
        return OnInitialize(options);
    }

    void Shutdown() override
    {
        Cancel();
        std::lock_guard<std::mutex> lock(mutex_);
        lifecycle_ = TaskLifecycle::kIdle;
        progress_.Clear();
        task_id_.clear();
        client_id_.clear();
    }

    bool Cancel() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (lifecycle_ == TaskLifecycle::kIdle ||
            IsTerminalLifecycle(lifecycle_)) {
            return false;
        }
        lifecycle_ = TaskLifecycle::kCanceled;
        return true;
    }

    bool Pause() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (lifecycle_ != TaskLifecycle::kRunning) {
            return false;
        }
        lifecycle_ = TaskLifecycle::kPaused;
        return true;
    }

    bool Resume() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (lifecycle_ != TaskLifecycle::kPaused) {
            return false;
        }
        lifecycle_ = TaskLifecycle::kRunning;
        return true;
    }

    ::autonomy::task::proto::ActiveTaskSnapshot GetSnapshot() const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ::autonomy::task::proto::ActiveTaskSnapshot snapshot;
        snapshot.set_task_type(this->GetTaskType());
        snapshot.set_task_status(ToRobotTaskStatus(lifecycle_));
        snapshot.set_task_id(task_id_);
        snapshot.set_client_id(client_id_);
        snapshot.set_progress(progress_.progress());
        return snapshot;
    }

    bool SubmitGoal(const GoalT& goal) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal.has_header()) {
            task_id_ = goal.header().task_id();
            client_id_ = goal.header().client_id();
        }
        return OnGoal(goal);
    }

    bool GetFeedback(FeedbackT* feedback) const override
    {
        if (feedback == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        FillFeedback(feedback);
        return true;
    }

    bool GetResult(ResultT* result) const override
    {
        if (result == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!IsTerminalLifecycle(lifecycle_)) {
            return false;
        }
        FillResult(result);
        return true;
    }

protected:
    virtual bool OnInitialize(
        const ::autonomy::task::proto::TaskServerOptions& /*options*/)
    {
        return true;
    }

    virtual bool OnGoal(const GoalT& goal) = 0;
    virtual void FillFeedback(FeedbackT* feedback) const = 0;
    virtual void FillResult(ResultT* result) const = 0;

    void SetLifecycle(TaskLifecycle lifecycle) { lifecycle_ = lifecycle; }

    void SetProgress(float progress, const std::string& detail = {})
    {
        progress_.set_progress(progress);
        if (!detail.empty()) {
            progress_.set_detail(detail);
        }
    }

    TaskLifecycle Lifecycle() const { return lifecycle_; }

    ::autonomy::task::proto::TaskResult MakeTaskResult() const
    {
        ::autonomy::task::proto::TaskResult envelope;
        envelope.set_status(ToRobotTaskStatus(lifecycle_));
        envelope.set_message(progress_.detail());
        return envelope;
    }

    mutable std::mutex mutex_;
    TaskLifecycle lifecycle_{TaskLifecycle::kIdle};
    ::autonomy::task::proto::TaskServerOptions server_options_;
    ::autonomy::task::proto::TaskProgress progress_;
    std::string task_id_;
    std::string client_id_;
};

}  // namespace task
}  // namespace autonomy
