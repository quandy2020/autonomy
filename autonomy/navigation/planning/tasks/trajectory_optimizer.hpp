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
 * @file trajectory_optimizer.hpp
 * @brief Command smoothing to keep chunk boundaries continuous.
 */

#pragma once

#include "navigation/planning/interface/task_base.hpp"

namespace navigation
{
namespace planning
{

class TrajectoryOptimizer final : public TaskBase
{
public:
    /** @brief Construct with a curvature blend factor in [0,1]. */
    explicit TrajectoryOptimizer(double blend = 0.3);

    /** @brief Execute one optimization step. */
    common::Status Execute(NavigationFrame& frame) override;

    /** @brief Reset internal history. */
    void Reset() override;

    /** @brief Task name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Curvature blend factor in [0,1]. */
    double blend_{0.3};
    /** @brief Last emitted command. */
    common::Twist2D previous_{};
    /** @brief True once previous_ is valid. */
    bool has_previous_{false};
};

}  // namespace planning
}  // namespace navigation
