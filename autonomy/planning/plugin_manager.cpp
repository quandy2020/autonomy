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

#include "autonomy/planning/plugin_manager.hpp"

#include <algorithm>
#include <functional>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/planning/smoother/savitzky_golay_smoother.hpp"
#include "autonomy/planning/smoother/simple_smoother.hpp"
#include "autonomy/planning/smoother/cos_theta_path_smoother.hpp"
#include "autonomy/planning/smoother/fem_pos_smoother.hpp"

namespace autonomy {
namespace planning {

namespace {

constexpr char kNavfnType[] = "NavfnPlanner";
constexpr char kDijkstraType[] = "DijkstraPlanner";
constexpr char kThetaStarType[] = "ThetaStarPlanner";

autolink::plugin_manager::PluginManager* AutolinkPluginManager() {
    return autolink::plugin_manager::PluginManager::Instance();
}

template <typename Base>
bool IsClassRegistered(const std::string& type) {
    const auto names =
        AutolinkPluginManager()->GetDerivedClassNameByBaseClass<Base>();
    return std::find(names.begin(), names.end(), type) != names.end();
}

template <typename Base>
std::shared_ptr<Base> CreatePluginInstance(const std::string& type) {
    return AutolinkPluginManager()->CreateInstance<Base>(type);
}

std::vector<PlannerPluginEntry> ParsePluginEntries(
    const std::vector<std::string>& plugin_entries,
    const std::function<std::string(const std::string&)>& resolve_type) {
    std::vector<PlannerPluginEntry> entries;
    entries.reserve(plugin_entries.size());
    for (const auto& entry_str : plugin_entries) {
        if (entry_str.empty()) {
            continue;
        }
        PlannerPluginEntry entry;
        const size_t colon_pos = entry_str.find(':');
        if (colon_pos != std::string::npos) {
            entry.id = entry_str.substr(0, colon_pos);
            entry.type = entry_str.substr(colon_pos + 1);
        } else {
            entry.id = entry_str;
            entry.type = resolve_type(entry_str);
        }
        if (!entry.id.empty() && !entry.type.empty()) {
            entries.push_back(std::move(entry));
        }
    }
    return entries;
}

}  // namespace

PluginManager::PluginManager() {
    RegisterBuiltinPlugins();
}

PluginManager& PluginManager::Instance() {
    static PluginManager instance;
    return instance;
}

void PluginManager::RegisterBuiltinPlugins() {
    auto* pm = AutolinkPluginManager();
    pm->RegisterInProcessClass<common::GlobalPlanner>(kNavfnType);
    pm->RegisterInProcessClass<common::GlobalPlanner>(kDijkstraType);
    pm->RegisterInProcessClass<common::GlobalPlanner>(kThetaStarType);

    pm->RegisterInProcessClass<common::Smoother>("SimpleSmoother");
    pm->RegisterInProcessClass<common::Smoother>("SavitzkyGolaySmoother");
    pm->RegisterInProcessClass<common::Smoother>("FemPosSmoother");
    pm->RegisterInProcessClass<common::Smoother>("CosThetaPathSmoother");
}

void PluginManager::Initialize(const proto::PlannerOptions& options) {
    if (initialized_) {
        return;
    }

    for (int i = 0; i < options.planner_plugin_libraries_size(); ++i) {
        const std::string& entry = options.planner_plugin_libraries(i);
        if (entry.empty()) {
            continue;
        }
        if (!LoadPluginDescription(entry)) {
            AWARN << "Failed to load planner plugin description: " << entry;
        }
    }

    LoadPlugins();
    initialized_ = true;
}

bool PluginManager::LoadPluginDescription(const std::string& description_path) {
    return AutolinkPluginManager()->LoadPlugin(description_path);
}

void PluginManager::LoadPlugins() {
    AutolinkPluginManager()->LoadInstalledPlugins();
}

common::GlobalPlanner::SharedPtr PluginManager::CreatePlanner(
    const std::string& type) {
    const std::string resolved = ResolvePlannerType(type);
    auto instance = CreatePluginInstance<common::GlobalPlanner>(resolved);
    return instance ? common::GlobalPlanner::SharedPtr(std::move(instance))
                    : nullptr;
}

common::Smoother::SharedPtr PluginManager::CreateSmoother(
    const std::string& type) {
    const std::string resolved = ResolveSmootherType(type);
    auto instance = CreatePluginInstance<common::Smoother>(resolved);
    return instance ? common::Smoother::SharedPtr(std::move(instance)) : nullptr;
}

bool PluginManager::IsPlannerRegistered(const std::string& type) const {
    return IsClassRegistered<common::GlobalPlanner>(ResolvePlannerType(type));
}

bool PluginManager::IsSmootherRegistered(const std::string& type) const {
    return IsClassRegistered<common::Smoother>(ResolveSmootherType(type));
}

std::vector<PlannerPluginEntry> PluginManager::ParsePlannerPluginEntries(
    const std::vector<std::string>& plugin_entries) {
    return ParsePluginEntries(plugin_entries, ResolvePlannerType);
}

std::vector<SmootherPluginEntry> PluginManager::ParseSmootherPluginEntries(
    const std::vector<std::string>& plugin_entries) {
    std::vector<SmootherPluginEntry> entries;
    entries.reserve(plugin_entries.size());
    for (const auto& entry_str : plugin_entries) {
        if (entry_str.empty()) {
            continue;
        }
        SmootherPluginEntry entry;
        const size_t colon_pos = entry_str.find(':');
        if (colon_pos != std::string::npos) {
            entry.id = entry_str.substr(0, colon_pos);
            entry.type = entry_str.substr(colon_pos + 1);
        } else {
            entry.id = entry_str;
            entry.type = ResolveSmootherType(entry_str);
        }
        if (!entry.id.empty() && !entry.type.empty()) {
            entries.push_back(std::move(entry));
        }
    }
    return entries;
}

std::string PluginManager::ResolvePlannerType(const std::string& plugin_id) {
    if (plugin_id == "navfn_planner") {
        return kNavfnType;
    }
    if (plugin_id == "dijkstra_planner") {
        return kDijkstraType;
    }
    if (plugin_id == "theta_star_planner") {
        return kThetaStarType;
    }
    return plugin_id;
}

std::string PluginManager::ResolveSmootherType(const std::string& smoother_id) {
    if (smoother_id == "simple_smoother") {
        return "SimpleSmoother";
    }
    if (smoother_id == "savitzky_golay_smoother") {
        return "SavitzkyGolaySmoother";
    }
    if (smoother_id == "fem_pos_smoother") {
        return "FemPosSmoother";
    }
    if (smoother_id == "cos_theta_smoother") {
        return "CosThetaPathSmoother";
    }
    return smoother_id;
}

void ApplySmootherOptions(common::Smoother& smoother,
                          const proto::PlannerOptions& options,
                          const std::string& smoother_id) {
    if (auto* simple = dynamic_cast<smoother::SimpleSmoother*>(&smoother)) {
        simple->ApplyOptions(options.simple_smoother());
        return;
    }
    if (auto* sg = dynamic_cast<smoother::SavitzkyGolaySmoother*>(&smoother)) {
        sg->ApplyOptions(options.savitzky_golay_smoother());
        return;
    }
    if (auto* fem = dynamic_cast<smoother::FemPosSmoother*>(&smoother)) {
        fem->ApplyOptions(options.fem_pos_smoother(),
                          options.fem_pos_smoother_path_bound());
        return;
    }
    if (auto* cos_theta =
            dynamic_cast<smoother::CosThetaPathSmoother*>(&smoother)) {
        cos_theta->ApplyOptions(options.cos_theta_smoother(),
                                options.cos_theta_smoother_path_bound());
        return;
    }
    (void)smoother_id;
}

}  // namespace planning
}  // namespace autonomy
