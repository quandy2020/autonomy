/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight planning-scene geometry file I/O (MoveIt .scene analogue subset).
 */

#pragma once

#include <string>

#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

/**
 * @brief Save world objects + occupancy to a simple text geometry file.
 * @param[in] scene Source planning scene.
 * @param[in] path Output file path.
 * @return true on successful write.
 */
bool SaveGeometryToFile(const PlanningScene& scene, const std::string& path);

/**
 * @brief Load world objects + occupancy from a geometry file into @p scene.
 * @param[in,out] scene Destination scene (world cleared then replaced).
 * @param[in] path Input file path.
 * @return true on successful read/apply.
 */
bool LoadGeometryFromFile(PlanningScene* scene, const std::string& path);

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
