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
 * @brief Resolves a language label to the nearest place and navigates there.
 */

#pragma once

#include <string>

#include "navigation/behavior/tasks/task_primitive.hpp"
#include "navigation/memory/semantic_scene_graph.hpp"

namespace navigation
{
namespace behavior
{

class SemanticSearchTask final : public TaskPrimitive
{
public:
    /** @brief Set semantic graph. */
    void SetGraph(memory::SemanticSceneGraph* graph);

    /** @brief Set language label. */
    void SetLabel(std::string label);

    /** @brief Begin the task. */
    void Start(TaskContext& context) override;

    /** @brief Advance one tick. */
    TaskStatus Tick(TaskContext& context) override;

    /** @brief Reset local state. */
    void Reset() override;

    /** @brief Stable name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Graph to query; non-owning. */
    memory::SemanticSceneGraph* graph_{nullptr};
    /** @brief Target label. */
    std::string label_;
    /** @brief Primitive lifecycle state. */
    TaskStatus status_{TaskStatus::kIdle};
};

}  // namespace behavior
}  // namespace navigation
