/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file simulation_plugins.hpp
 * @brief Registers model-free plugins used by autosim.
 */

#pragma once

namespace navigation
{
namespace planning
{

/** @brief Register the statistics encoder and simulation policy. */
void RegisterSimulationPlugins();

}  // namespace planning
}  // namespace navigation
