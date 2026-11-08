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

#include "navigation/behavior/tasks/semantic_search_task.hpp"

#include <utility>

namespace navigation
{
namespace behavior
{

void SemanticSearchTask::SetGraph(memory::SemanticSceneGraph* graph)
{
    graph_ = graph;
}

void SemanticSearchTask::SetLabel(std::string label)
{
    label_ = std::move(label);
}

void SemanticSearchTask::Start(TaskContext& context)
{
    if (graph_ == nullptr || context.session == nullptr || label_.empty())
    {
        status_ = TaskStatus::kFailed;
        return;
    }
    const auto node = graph_->Nearest(label_, context.pose);
    if (!node.has_value())
    {
        status_ = TaskStatus::kFailed;
        return;
    }
    planning::Waypoint waypoint;
    waypoint.pose = node->pose;
    if (!context.session->SetGoal(MakePoseGoal(waypoint, "semantic:" + label_)))
    {
        status_ = TaskStatus::kFailed;
        return;
    }
    status_ = TaskStatus::kRunning;
}

TaskStatus SemanticSearchTask::Tick(TaskContext& context)
{
    if (status_ != TaskStatus::kRunning)
    {
        return status_;
    }
    if (context.goal_reached)
    {
        status_ = TaskStatus::kSucceeded;
    }
    return status_;
}

void SemanticSearchTask::Reset()
{
    status_ = TaskStatus::kIdle;
    label_.clear();
}

std::string_view SemanticSearchTask::Name() const
{
    return "semantic_search";
}

}  // namespace behavior
}  // namespace navigation
