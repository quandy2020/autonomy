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

#include "navigation/behavior/intent/language_to_goal.hpp"

namespace navigation
{
namespace behavior
{

bool LanguageToGoal::Convert(const std::string& text,
                             const memory::SemanticSceneGraph& graph,
                             const common::Pose2D& pose, GoalSpec* goal) const
{
    if (goal == nullptr || text.empty())
    {
        return false;
    }
    const auto node = graph.Nearest(text, pose);
    if (!node.has_value())
    {
        return false;
    }
    goal->Clear();
    goal->set_type(GoalSpec::TYPE_POSE);
    auto* pose_proto = goal->mutable_pose();
    pose_proto->mutable_header()->set_frame_id("map");
    pose_proto->mutable_pose()->mutable_position()->set_x(node->pose.x());
    pose_proto->mutable_pose()->mutable_position()->set_y(node->pose.y());
    pose_proto->mutable_pose()->mutable_orientation()->set_w(1.0);
    return true;
}

}  // namespace behavior
}  // namespace navigation
