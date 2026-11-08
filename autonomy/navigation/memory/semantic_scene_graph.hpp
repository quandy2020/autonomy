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
 * @file semantic_scene_graph.hpp
 * @brief Labeled places and connectivity used for language goals.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace memory
{

/** @brief Labeled place in the semantic graph. */
struct SceneNode
{
    /** @brief Stable node id. */
    std::uint64_t id{0};
    /** @brief Language label, e.g. "kitchen". */
    std::string label;
    /** @brief Map-frame place pose. */
    common::Pose2D pose;
};

class SemanticSceneGraph
{
public:
    /**
     * @brief Add a labeled place.
     *
     * @param[in] label Language label.
     * @param[in] pose Place pose.
     * @return New node id.
     */
    std::uint64_t AddNode(std::string label, const common::Pose2D& pose);

    /**
     * @brief Add a directed edge.
     *
     * @param[in] from Source node id.
     * @param[in] to Destination node id.
     * @return false when either endpoint is missing or from==to.
     */
    bool AddEdge(std::uint64_t from, std::uint64_t to);

    /** @brief Return all nodes with an exact label. */
    [[nodiscard]] std::vector<SceneNode> Find(const std::string& label) const;

    /**
     * @brief Return the nearest node with a label.
     *
     * @param[in] label Exact label.
     * @param[in] pose Query pose.
     */
    [[nodiscard]] std::optional<SceneNode> Nearest(const std::string& label,
                                                   const common::Pose2D& pose) const;

    /** @brief Number of nodes. */
    [[nodiscard]] std::size_t Size() const;

    /** @brief Remove all nodes and edges. */
    void Clear();

private:
    /** @brief Next node id. */
    std::uint64_t next_id_{1};
    /** @brief Node storage. */
    std::vector<SceneNode> nodes_;
    /** @brief Directed edge storage. */
    std::vector<std::pair<std::uint64_t, std::uint64_t>> edges_;
};

}  // namespace memory
}  // namespace navigation
