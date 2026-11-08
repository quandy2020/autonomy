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
 * @file following_task.hpp
 * @brief Keeps a stand-off distance behind a moving target.
 */

#pragma once

#include "navigation/behavior/tasks/task_primitive.hpp"

namespace navigation
{
namespace behavior
{

class FollowingTask final : public TaskPrimitive
{
public:
    /** @brief Construct with the desired follow distance. */
    explicit FollowingTask(double desired_distance = 1.0);

    /** @brief Set follow target. */
    void SetTarget(const common::Pose2D& target);

    /** @brief Mark target validity. */
    void SetTargetValid(bool valid);

    /** @brief Begin the task. */
    void Start(TaskContext& context) override;

    /** @brief Advance one tick. */
    TaskStatus Tick(TaskContext& context) override;

    /** @brief Reset local state. */
    void Reset() override;

    /** @brief Stable name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Stand-off distance in metres. */
    double desired_distance_{1.0};
    /** @brief Latest follow target in the map frame. */
    common::Pose2D target_;
    /** @brief True when target_ is fresh. */
    bool target_valid_{false};
    /** @brief True after the follow goal was submitted. */
    bool goal_issued_{false};
    /** @brief Primitive lifecycle state. */
    TaskStatus status_{TaskStatus::kIdle};
};

}  // namespace behavior
}  // namespace navigation
