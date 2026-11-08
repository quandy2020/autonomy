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

#include "navigation/planning/tasks/semantic_search_task.hpp"

#include <utility>

namespace navigation
{
namespace planning
{

void SemanticSearchTask::SetGraph(memory::SemanticSceneGraph* graph)
{
    graph_ = graph;
}

void SemanticSearchTask::SetLabel(std::string label)
{
    label_ = std::move(label);
}

common::Status SemanticSearchTask::Execute(NavigationFrame& frame)
{
    if (frame.goal.has_pose())
    {
        return common::Status{};
    }
    if (graph_ == nullptr || label_.empty())
    {
        return common::Status::InvalidArgument("semantic search is not configured");
    }
    const auto node = graph_->Nearest(label_, frame.pose);
    if (!node.has_value())
    {
        return common::Status::Unavailable("label not found");
    }
    frame.goal.Clear();
    frame.goal.set_type(GoalSpec::TYPE_POSE);
    frame.goal.set_goal_id("semantic:" + label_);
    auto* pose = frame.goal.mutable_pose();
    pose->mutable_header()->set_frame_id("map");
    pose->mutable_pose()->mutable_position()->set_x(node->pose.x());
    pose->mutable_pose()->mutable_position()->set_y(node->pose.y());
    pose->mutable_pose()->mutable_orientation()->set_w(1.0);
    return common::Status{};
}

void SemanticSearchTask::Reset()
{
    label_.clear();
}

std::string_view SemanticSearchTask::Name() const
{
    return "semantic_search";
}

}  // namespace planning
}  // namespace navigation
