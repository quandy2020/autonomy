/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Amanatides–Woo voxel traversal from FIESTA, without ROS.
 */

#pragma once

#include <vector>

#include <Eigen/Core>

namespace autonomy {
namespace map {
namespace esdf {

/**
 * @brief Walk voxels from start to end in grid coordinates.
 *
 * Coordinates are typically world / resolution. Only voxels inside
 * [min, max) are appended. Throws if the walk exceeds 1500 cells.
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min, const Eigen::Vector3d& max,
             std::vector<Eigen::Vector3d>* output);

}  // namespace esdf
}  // namespace map
}  // namespace autonomy
