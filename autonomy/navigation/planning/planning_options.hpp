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
 * @file planning_options.hpp
 * @brief Validated planning-side view of NavigationConfig.
 */

#pragma once

#include <string>

#include "navigation/common/status.hpp"
#include "navigation/common/status_or.hpp"
#include "navigation/proto/config.pb.h"

namespace navigation
{
namespace planning
{

class PlanningOptions
{
public:
    /** @brief Construct from a parsed configuration. */
    explicit PlanningOptions(config::NavigationConfig configuration);

    /** @brief Read-only access to the wrapped configuration. */
    [[nodiscard]] const config::NavigationConfig& Configuration() const;

    /** @brief Cross-field validation; on failure @p error is set. */
    [[nodiscard]] bool Validate(std::string* error) const;

    /**
     * @brief Load and validate a configuration file.
     *
     * @param[in] path Absolute or NAVIGATION_CONF_DIR-relative path.
     */
    static common::StatusOr<PlanningOptions> Load(const std::string& path);

private:
    /** @brief Wrapped configuration snapshot. */
    config::NavigationConfig configuration_;
};

}  // namespace planning
}  // namespace navigation
