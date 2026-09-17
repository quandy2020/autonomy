/*
 * Copyright 2026 The Openbot Authors
 *
 * Multi-convex mesh policy: offline *.convexparts (production default) +
 * optional online approximate decomposition (VHACD-free spatial split).
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief Online multi-convex approx without V-HACD dependency.
 *
 * Recursively splits the triangle cloud on the longest AABB axis and builds a
 * convex hull per leaf. Suitable when no `*.convexparts` sidecar exists.
 *
 * @return Number of convex parts written to @p out (≥1 on success).
 */
int ApproximateConvexDecomposition(
    const std::vector<MeshVertex>& verts,
    const std::vector<int>& triangles, const ConvexDecomposeOptions& options,
    std::vector<LinkCollisionShape>* out, const std::string& link_name,
    const core::Transform& origin);

/**
 * @brief Write a `*.convexparts` sidecar listing part STL paths (one per line).
 */
bool WriteConvexPartsSidecar(const std::string& sidecar_path,
                             const std::vector<std::string>& part_stl_paths);

/** @brief Export one convex hull as binary STL (triangle soup). */
bool WriteConvexStl(const std::string& path,
                    const std::vector<MeshVertex>& hull_verts,
                    const std::vector<int>& hull_faces);

/**
 * @brief Resolve multi-convex for a mesh: sidecar first, else optional online.
 * @return Number of shapes appended to @p out.
 */
int ResolveMultiConvexForMesh(const std::string& mesh_path,
                              const std::string& link_name,
                              const core::Transform& origin, bool enable_online,
                              const ConvexDecomposeOptions& options,
                              std::vector<LinkCollisionShape>* out);

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
