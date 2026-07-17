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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/proto/vehicle_msgs.pb.h"
#include "autonomy/task/common/task_interface.hpp"
#include "autonomy/task/proto/task_options.pb.h"
#include "autonomy/task/scheduler/task_mux.hpp"

namespace autonomy {
namespace task {

/** 流水线单步回调，按顺序执行。 */
using TaskPipelineStep = std::function<void()>;

/**
 * @brief 任务调度器
 *
 * 职责：
 * - 注册 / 查找 TaskInterface 插件
 * - TaskMux 互斥导航类任务（navigation / tracking / teleop / exploration / dock）
 * - 后台线程驱动反馈轮询与流水线编排
 * - 回收进入终态的活跃会话
 */
class TaskScheduler
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskScheduler);

    explicit TaskScheduler(
        ::autonomy::task::proto::SchedulerOptions options = {});

    ~TaskScheduler();

    TaskScheduler(const TaskScheduler&) = delete;
    TaskScheduler& operator=(const TaskScheduler&) = delete;

    bool Initialize(const ::autonomy::task::proto::TaskServerOptions& options);
    void Shutdown();

    void RegisterTask(TaskInterface::SharedPtr task);
    TaskInterface::SharedPtr GetTask(
        ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType type) const;

    /**
     * @brief 申请激活任务（通过互斥检查后加入活跃列表）
     * @return false 插件未注册、互斥冲突或未初始化
     */
    bool RequestActivation(TaskInterface::SharedPtr task);
    void ReleaseActivation(TaskInterface::SharedPtr task);

    bool CancelActive(
        ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType type);
    bool PauseActive(
        ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType type);
    bool ResumeActive(
        ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType type);

    /** 启动反馈轮询循环（幂等）。 */
    bool Start();
    void Stop();

    bool IsRunning() const { return running_.load(); }

    std::vector<::autonomy::task::proto::ActiveTaskSnapshot> GetActiveSnapshots()
        const;

    const TaskMux& GetMux() const { return mux_; }

    /**
     * @brief 顺序执行多步流水线（阻塞至完成）
     *
     * 典型用法：localization → mapping.load → navigation.start
     */
    void RunPipeline(const std::vector<TaskPipelineStep>& steps);
    void RunPipelineAsync(const std::vector<TaskPipelineStep>& steps);

    std::size_t WorkerCount() const;

private:
    void PollActiveTasks();
    void PruneInactiveSessions();

    ::autonomy::task::proto::SchedulerOptions options_;
    ::autonomy::task::proto::TaskServerOptions server_options_;
    TaskMux mux_;

    mutable std::mutex mutex_;
    std::unordered_map<int, TaskInterface::SharedPtr> registry_;
    std::vector<TaskInterface::SharedPtr> active_sessions_;

    struct Runtime;
    std::unique_ptr<Runtime> runtime_;
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};
};

}  // namespace task
}  // namespace autonomy
