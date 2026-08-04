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

#include <functional>
#include <string>

#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_task_status.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>
#include "autonomy/task/proto/charging.pb.h"
#include "autonomy/task/proto/exploration.pb.h"
#include "autonomy/task/proto/localization.pb.h"
#include "autonomy/task/proto/mapping.pb.h"
#include "autonomy/task/proto/navigation.pb.h"
#include "autonomy/task/proto/task_common.pb.h"
#include "autonomy/task/proto/task_options.pb.h"
#include "autonomy/task/proto/teleop.pb.h"
#include "autonomy/task/proto/tracker.pb.h"

namespace autonomy {
namespace task {

/**
 * @brief 任务生命周期（与 RobotTaskStatus 及各类 *Status 终态对齐）
 */
enum class TaskLifecycle {
    kIdle,
    kRunning,
    kPaused,
    kSucceeded,
    kFailed,
    kCanceled,
};

inline ::automsgs::msgs::vehicle_msgs::RobotTaskStatus
ToRobotTaskStatus(TaskLifecycle lifecycle)
{
    using ::automsgs::msgs::vehicle_msgs::RobotTaskStatus;
    switch (lifecycle) {
    case TaskLifecycle::kIdle:
        return RobotTaskStatus::ROBOT_TASK_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        return RobotTaskStatus::ROBOT_TASK_STATUS_RUNNING;
    case TaskLifecycle::kPaused:
        return RobotTaskStatus::ROBOT_TASK_STATUS_PAUSED;
    case TaskLifecycle::kSucceeded:
        return RobotTaskStatus::ROBOT_TASK_STATUS_SUCCEEDED;
    case TaskLifecycle::kFailed:
        return RobotTaskStatus::ROBOT_TASK_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return RobotTaskStatus::ROBOT_TASK_STATUS_CANCELED;
    }
    return RobotTaskStatus::ROBOT_TASK_STATUS_UNKNOWN;
}

inline bool IsTerminalLifecycle(TaskLifecycle lifecycle)
{
    return lifecycle == TaskLifecycle::kSucceeded ||
           lifecycle == TaskLifecycle::kFailed ||
           lifecycle == TaskLifecycle::kCanceled;
}

/**
 * @brief 任务插件统一生命周期接口
 *
 * TaskServer / Scheduler 通过 TaskInterface::SharedPtr 管理各 apps/* 实现。
 * 具体 Goal / Feedback / Result 由 TypedTaskInterface 子类承载，与
 * autonomy/task/proto 中各 *Goal / *Feedback / *Result 一一对应。
 */
class TaskInterface
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskInterface);

    virtual ~TaskInterface() = default;

    /** @return 本插件对应的 RobotTaskType（localization 等内部任务可返回 NONE） */
    virtual ::automsgs::msgs::vehicle_msgs::RobotTaskType GetTaskType()
        const = 0;

    /** @return 当前生命周期状态 */
    virtual TaskLifecycle GetLifecycle() const = 0;

    /** @return 是否持有活跃会话（Running 或 Paused） */
    virtual bool IsActive() const
    {
        const auto lifecycle = GetLifecycle();
        return lifecycle == TaskLifecycle::kRunning ||
               lifecycle == TaskLifecycle::kPaused;
    }

    /**
     * @brief 使用 TaskServer 配置初始化插件
     * @param options TaskServerOptions（含 apps 开关与 config_directory）
     * @return true 初始化成功
     */
    virtual bool Initialize(
        const ::autonomy::task::proto::TaskServerOptions& options) = 0;

    /** @brief 停止插件并释放活跃任务 */
    virtual void Shutdown() = 0;

    /**
     * @brief 取消当前任务（等价于提交各 proto 中的 *CMD_CANCEL）
     * @return true 已对活跃会话应用取消
     */
    virtual bool Cancel() = 0;

    /** @brief 暂停当前任务 */
    virtual bool Pause() = 0;

    /** @brief 从暂停恢复 */
    virtual bool Resume() = 0;

    /** @brief 供 Scheduler 与 Bridge 适配器查询的轻量快照 */
    virtual ::autonomy::task::proto::ActiveTaskSnapshot GetSnapshot() const = 0;
};

/**
 * @brief 强类型任务接口：Goal / Feedback / Result 与 task proto 对齐
 *
 * 各 apps/* 实现本模板，通过 SubmitGoal 接收带 TaskHeader 的目标或子命令，
 * 周期性调用 GetFeedback，终态时通过 GetResult 取回 TaskResult 信封。
 */
template <typename GoalT, typename FeedbackT, typename ResultT>
class TypedTaskInterface : public TaskInterface
{
public:
    using Goal = GoalT;
    using Feedback = FeedbackT;
    using Result = ResultT;
    using CancelChecker = std::function<bool()>;

    ~TypedTaskInterface() override = default;

    /**
     * @brief 提交新目标或子命令（START / STOP / UPDATE_TARGET 等）
     * @param goal 含 TaskHeader 与各 *Command 枚举的 Goal 消息
     * @return true 已接受；false 表示拒绝（互斥、未初始化等）
     */
    virtual bool SubmitGoal(const Goal& goal) = 0;

    /**
     * @brief 读取当前反馈（由 Scheduler 按 feedback_period_ms 轮询）
     * @param feedback 输出参数
     * @return true 反馈有效
     */
    virtual bool GetFeedback(Feedback* feedback) const = 0;

    /**
     * @brief 任务进入终态后读取结果
     * @param result 输出参数
     * @return true 结果可用（通常 IsTerminalLifecycle(GetLifecycle()) 为真）
     */
    virtual bool GetResult(Result* result) const = 0;
};

// apps/navigation
using NavigationTaskInterface = TypedTaskInterface<
    ::autonomy::task::proto::NavigationGoal,
    ::autonomy::task::proto::NavigationFeedback,
    ::autonomy::task::proto::NavigationResult>;

// apps/tracking
using TrackerTaskInterface = TypedTaskInterface<
    ::autonomy::task::proto::TrackerGoal,
    ::autonomy::task::proto::TrackerFeedback,
    ::autonomy::task::proto::TrackerResult>;

// apps/teleop
using TeleopTaskInterface = TypedTaskInterface<
    ::autonomy::task::proto::TeleopGoal,
    ::autonomy::task::proto::TeleopFeedback,
    ::autonomy::task::proto::TeleopResult>;

// apps/exploration
using ExplorationTaskInterface = TypedTaskInterface<
    ::autonomy::task::proto::ExplorationGoal,
    ::autonomy::task::proto::ExplorationFeedback,
    ::autonomy::task::proto::ExplorationResult>;

// apps/charging
using ChargingTaskInterface = TypedTaskInterface<
    ::autonomy::task::proto::ChargingGoal,
    ::autonomy::task::proto::ChargingFeedback,
    ::autonomy::task::proto::ChargingResult>;

// apps/mapping
using MappingTaskInterface = TypedTaskInterface<
    ::autonomy::task::proto::MappingGoal,
    ::autonomy::task::proto::MappingFeedback,
    ::autonomy::task::proto::MappingResult>;

// apps/localization（内部 SLAM 后端控制，无对应 RobotTaskType）
using LocalizationTaskInterface = TypedTaskInterface<
    ::autonomy::task::proto::LocalizationGoal,
    ::autonomy::task::proto::LocalizationFeedback,
    ::autonomy::task::proto::LocalizationResult>;

}  // namespace task
}  // namespace autonomy
