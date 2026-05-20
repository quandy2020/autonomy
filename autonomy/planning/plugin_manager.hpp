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

/**
 * @brief Parsed planner plugin entry from configuration.
 *
 * Each entry in planner_plugins is either "plugin_id" or "plugin_id:TypeName".
 * id is the key used by PlannerServer; type is the class_loader / plugin_manager
 * derived class name passed to CreatePlanner.
 */
struct PlannerPluginEntry {
    /** Instance id (e.g. navfn_planner) used in BT and PlannerServer maps. */
    std::string id;
    /** Implementation type (e.g. NavfnPlanner) registered with autolink. */
    std::string type;
};

/**
 * @brief Parsed smoother plugin entry from configuration.
 *
 * Same format as planner plugins: "smoother_id" or "smoother_id:TypeName".
 */
struct SmootherPluginEntry {
    /** Instance id (e.g. simple_smoother) used by SmootherServer. */
    std::string id;
    /** Implementation type (e.g. SimpleSmoother) registered with autolink. */
    std::string type;
};

/**
 * Plugin loading contract for planning:
 * - Built-in planners/smoothers are registered in-process via CLASS_LOADER and
 *   RegisterInProcessClass (see plugin_manager.cpp and per-plugin .cpp files).
 * - External plugins use autolink plugin description XML; paths are listed in
 *   PlannerOptions.planner_plugin_libraries and loaded by Initialize().
 * - Plugin id aliases (e.g. navfn_planner -> NavfnPlanner) are resolved by
 *   ResolvePlannerType / ResolveSmootherType before Create*.
 * - Create* returns nullptr if the type is unknown or library load fails.
 */
class PluginManager
{
public:
    /**
     * @brief Returns the process-wide plugin manager singleton.
     * @return Reference to the singleton PluginManager instance.
     */
    static PluginManager& Instance();

    /**
     * @brief One-time setup: load external plugin descriptions and index paths.
     *
     * Safe to call multiple times; subsequent calls are no-ops after the first
     * successful initialization. PlannerServer and SmootherServer call this
     * before creating plugin instances.
     *
     * @param options PlannerOptions containing planner_plugin_libraries paths.
     */
    void Initialize(const proto::PlannerOptions& options);

    /**
     * @brief Register built-in planner and smoother types with autolink.
     *
     * Called from the constructor; maps class names to
     * autonomy::planning::common::GlobalPlanner and Smoother base types.
     */
    void RegisterBuiltinPlugins();

    /**
     * @brief Load a single plugin from an autolink plugin description XML file.
     * @param description_path Path to the plugin description file.
     * @return true if the description was parsed and registered.
     */
    bool LoadPluginDescription(const std::string& description_path);

    /**
     * @brief Discover and register plugins from AUTOLINK_PLUGIN_INDEX_PATH.
     *
     * Uses lazy loading: libraries are loaded when CreatePlanner/CreateSmoother
     * is first invoked for a class in that plugin.
     */
    void LoadPlugins();

    /**
     * @brief Create a planner plugin instance by type or alias.
     * @param type Plugin type or config id (e.g. NavfnPlanner, navfn_planner).
     * @return Shared pointer to the planner, or nullptr on failure.
     */
    common::GlobalPlanner::SharedPtr CreatePlanner(const std::string& type);

    /**
     * @brief Create a smoother plugin instance by type or alias.
     * @param type Plugin type or config id (e.g. SimpleSmoother,
     * simple_smoother).
     * @return Shared pointer to the smoother, or nullptr on failure.
     */
    common::Smoother::SharedPtr CreateSmoother(const std::string& type);

    /**
     * @brief Check whether a planner type or id is registered.
     * @param type Plugin type or alias id.
     * @return true if CreatePlanner would resolve this type.
     */
    bool IsPlannerRegistered(const std::string& type) const;

    /**
     * @brief Check whether a smoother type or id is registered.
     * @param type Plugin type or alias id.
     * @return true if CreateSmoother would resolve this type.
     */
    bool IsSmootherRegistered(const std::string& type) const;

    /**
     * @brief Parse planner_plugins configuration strings.
     * @param plugin_entries Entries from PlannerOptions or lua config.
     * @return Vector of id/type pairs for PlannerServer::LoadPlugins.
     */
    static std::vector<PlannerPluginEntry> ParsePlannerPluginEntries(
        const std::vector<std::string>& plugin_entries);

    /**
     * @brief Parse smoother_plugins configuration strings.
     * @param plugin_entries Entries from PlannerOptions or lua config.
     * @return Vector of id/type pairs for SmootherServer::LoadPlugins.
     */
    static std::vector<SmootherPluginEntry> ParseSmootherPluginEntries(
        const std::vector<std::string>& plugin_entries);

    /**
     * @brief Map configuration plugin id to class_loader type name.
     * @param plugin_id Short id from config (e.g. navfn_planner).
     * @return Resolved type (e.g. NavfnPlanner), or plugin_id if no alias.
     */
    static std::string ResolvePlannerType(const std::string& plugin_id);

    /**
     * @brief Map configuration smoother id to class_loader type name.
     * @param smoother_id Short id from config (e.g. simple_smoother).
     * @return Resolved type (e.g. SimpleSmoother), or smoother_id if no alias.
     */
    static std::string ResolveSmootherType(const std::string& smoother_id);

private:
    /**
     * @brief Private constructor; use Instance() for access.
     */
    PluginManager();

    /** Whether Initialize() has completed successfully. */
    bool initialized_{false};
};

/**
 * @brief Apply per-smoother options from PlannerOptions after plugin creation.
 *
 * Dispatches to SimpleSmoother, SavitzkyGolaySmoother, FemPosSmoother, or
 * CosThetaPathSmoother based on dynamic_cast. No-op for unknown smoother types.
 *
 * @param smoother Smoother instance to configure.
 * @param options Full planner options proto (contains smoother sub-messages).
 * @param smoother_id Plugin instance id (reserved for future per-id overrides).
 */
void ApplySmootherOptions(common::Smoother& smoother,
                          const proto::PlannerOptions& options,
                          const std::string& smoother_id);

}  // namespace planning
}  // namespace autonomy
