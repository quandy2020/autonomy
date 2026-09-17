/*
 * Copyright 2026 The Openbot Authors
 *
 * True VHACD convex decomposition (FEATURE vhacd).
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief Run VHACD when AUTONOMY_HAS_VHACD; otherwise return 0.
 * @return Number of convex parts appended to @p out.
 */
int VhacdConvexDecomposition(const std::vector<MeshVertex>& verts,
                             const std::vector<int>& triangles,
                             const ConvexDecomposeOptions& options,
                             std::vector<LinkCollisionShape>* out,
                             const std::string& link_name,
                             const automsgs::msgs::geometry_msgs::Pose& origin);

/** @brief true if compiled with AUTONOMY_HAS_VHACD. */
bool HasVhacdBackend();

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
