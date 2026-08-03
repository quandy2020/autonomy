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

#include <automsgs/msgs/vehicle_msgs/vehicle_msgs.pb.h>

namespace autonomy {
namespace task {

/** 导航类任务互斥（对标 navigator::NavigatorMuxer / Bridge CommandFsm）。 */
class TaskMux
{
public:
    explicit TaskMux(bool exclusive_navigation_tasks = true)
        : exclusive_navigation_tasks_(exclusive_navigation_tasks) {}

    bool IsExclusive() const { return exclusive_navigation_tasks_; }

    bool IsNavigating() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_navigation_task_ !=
               ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NONE;
    }

    ::automsgs::msgs::vehicle_msgs::RobotTaskType ActiveNavigationTask()
        const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_navigation_task_;
    }

    /**
     * @brief 尝试占用导航类任务槽位
     * @return false 表示已有互斥任务在运行
     */
    bool TryAcquire(
        ::automsgs::msgs::vehicle_msgs::RobotTaskType task_type)
    {
        if (!exclusive_navigation_tasks_) {
            return true;
        }
        if (!IsNavigationClass(task_type)) {
            return true;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_navigation_task_ !=
            ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NONE) {
            return false;
        }
        active_navigation_task_ = task_type;
        return true;
    }

    void Release(
        ::automsgs::msgs::vehicle_msgs::RobotTaskType task_type)
    {
        if (!exclusive_navigation_tasks_) {
            return;
        }
        if (!IsNavigationClass(task_type)) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_navigation_task_ == task_type) {
            active_navigation_task_ =
                ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NONE;
        }
    }

    void Reset(bool exclusive_navigation_tasks)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        exclusive_navigation_tasks_ = exclusive_navigation_tasks;
        active_navigation_task_ =
            ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NONE;
    }

    static bool IsNavigationClass(
        ::automsgs::msgs::vehicle_msgs::RobotTaskType task_type)
    {
        using ::automsgs::msgs::vehicle_msgs::RobotTaskType;
        switch (task_type) {
        case RobotTaskType::ROBOT_TASK_NAVIGATION:
        case RobotTaskType::ROBOT_TASK_FOLLOW:
        case RobotTaskType::ROBOT_TASK_TELEOP:
        case RobotTaskType::ROBOT_TASK_EXPLORATION:
        case RobotTaskType::ROBOT_TASK_DOCK:
            return true;
        default:
            return false;
        }
    }

private:
    bool exclusive_navigation_tasks_;
    mutable std::mutex mutex_;
    ::automsgs::msgs::vehicle_msgs::RobotTaskType
        active_navigation_task_{
            ::automsgs::msgs::vehicle_msgs::ROBOT_TASK_NONE};
};

}  // namespace task
}  // namespace autonomy
