/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/planning/simulation_plugins.hpp"

#include "navigation/perception/depth_statistics_encoder.hpp"
#include "navigation/planning/plugin_manager.hpp"
#include "navigation/planning/plugins/policy/simulation/policy.hpp"

namespace navigation
{
namespace planning
{

void RegisterSimulationPlugins()
{
    PluginManager& plugins = PluginManager::Instance();
    plugins.RegisterEncoder("defm", [](const config::NavigationConfig&) {
        return std::make_unique<perception::DepthStatisticsEncoder>();
    });
    plugins.RegisterPolicy("simulation", [](const config::NavigationConfig& configuration) {
        return std::make_unique<simulation::Policy>(configuration.policy().vint());
    });
}

}  // namespace planning
}  // namespace navigation
