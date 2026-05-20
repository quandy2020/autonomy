/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/common/smoother_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy {
namespace planning {

struct PlannerPluginSpec {
    std::string id;
    std::string type;
};

struct SmootherPluginSpec {
    std::string id;
    std::string type;
};

/** Planner/smoother plugin loading via autolink::plugin_manager::PluginManager. */
class PlanningPluginManager
{
public:
    static PlanningPluginManager& Instance();

    /** Register in-process classes and load external plugin descriptions. */
    void Initialize(const proto::PlannerOptions& options);

    void RegisterBuiltinPlugins();

    /** Load plugin description XML files (autolink plugin_manager format). */
    bool LoadPluginDescription(const std::string& description_path);

    /** Discover plugins from AUTOLINK_PLUGIN_INDEX_PATH / install tree. */
    void LoadInstalledPlugins();

    common::GlobalPlanner::SharedPtr CreatePlanner(const std::string& type);

    common::Smoother::SharedPtr CreateSmoother(const std::string& type);

    bool IsPlannerRegistered(const std::string& type) const;

    bool IsSmootherRegistered(const std::string& type) const;

    static std::vector<PlannerPluginSpec> ParsePlannerPluginSpecs(
        const std::vector<std::string>& plugin_entries);

    static std::vector<SmootherPluginSpec> ParseSmootherPluginSpecs(
        const std::vector<std::string>& plugin_entries);

    static std::string ResolvePlannerType(const std::string& plugin_id);

    static std::string ResolveSmootherType(const std::string& smoother_id);

private:
    PlanningPluginManager();

    bool initialized_{false};
};

void ApplySmootherOptions(common::Smoother& smoother,
                          const proto::PlannerOptions& options,
                          const std::string& smoother_id);

}  // namespace planning
}  // namespace autonomy
