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
 * @file semantic_search_task.hpp
 * @brief Resolves a language label into frame.goal.
 */

#pragma once

#include <string>

#include "navigation/memory/semantic_scene_graph.hpp"
#include "navigation/planning/interface/task_base.hpp"

namespace navigation
{
namespace planning
{

class SemanticSearchTask final : public TaskBase
{
public:
    /** @brief Set the graph to query; non-owning. */
    void SetGraph(memory::SemanticSceneGraph* graph);

    /** @brief Set the target language label. */
    void SetLabel(std::string label);

    /** @brief Resolve the label into frame.goal when no pose goal exists. */
    common::Status Execute(NavigationFrame& frame) override;

    /** @brief Clear the label. */
    void Reset() override;

    /** @brief Task name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    memory::SemanticSceneGraph* graph_{nullptr};
    std::string label_;
};

}  // namespace planning
}  // namespace navigation
