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

#include "navigation/memory/semantic_scene_graph.hpp"

#include <algorithm>
#include <utility>

namespace navigation
{
namespace memory
{

std::uint64_t SemanticSceneGraph::AddNode(std::string label, const common::Pose2D& pose)
{
    SceneNode node;
    node.id = next_id_++;
    node.label = std::move(label);
    node.pose = pose;
    nodes_.push_back(std::move(node));
    return nodes_.back().id;
}

bool SemanticSceneGraph::AddEdge(std::uint64_t from, std::uint64_t to)
{
    const auto exists = [this](std::uint64_t id) {
        return std::any_of(nodes_.begin(), nodes_.end(),
                           [id](const SceneNode& node) { return node.id == id; });
    };
    if (from == to || !exists(from) || !exists(to))
    {
        return false;
    }
    edges_.emplace_back(from, to);
    return true;
}

std::vector<SceneNode> SemanticSceneGraph::Find(const std::string& label) const
{
    std::vector<SceneNode> matches;
    for (const auto& node : nodes_)
    {
        if (node.label == label)
        {
            matches.push_back(node);
        }
    }
    return matches;
}

std::optional<SceneNode> SemanticSceneGraph::Nearest(const std::string& label,
                                                     const common::Pose2D& pose) const
{
    std::optional<SceneNode> best;
    double best_distance = 1e18;
    for (const auto& node : nodes_)
    {
        if (node.label != label)
        {
            continue;
        }
        const double distance = common::Distance(node.pose, pose);
        if (distance < best_distance)
        {
            best_distance = distance;
            best = node;
        }
    }
    return best;
}

std::size_t SemanticSceneGraph::Size() const
{
    return nodes_.size();
}

void SemanticSceneGraph::Clear()
{
    nodes_.clear();
    edges_.clear();
    next_id_ = 1;
}

}  // namespace memory
}  // namespace navigation
