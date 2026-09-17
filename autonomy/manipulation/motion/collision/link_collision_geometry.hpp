/*
 * Copyright 2026 The Openbot Authors
 *
 * URDF collision geometry: primitives, triangle mesh (BVH), multi-convex parts.
 *
 * Multi-convex without V-HACD: multiple URDF <collision> tags and/or sidecar
 * `*.convexparts` (one STL path per line) next to the mesh file.
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/model/link_forward_kinematics.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/** @brief Single mesh vertex in object-local coordinates. */
struct MeshVertex {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/** @brief Options for online approximate convex decomposition. */
struct ConvexDecomposeOptions {
  int max_parts = 8;
  double min_part_size = 0.02;
  int max_depth = 6;
};

/** @brief Per-link collision primitive, convex hull, or triangle mesh. */
enum class LinkShapeKind { kSphere, kBox, kCylinder, kConvex, kMesh };

/**
 * @brief One collision shape attached to a link (URDF `<collision>` analogue).
 *
 * Origin is relative to the link frame.
 * - @c kConvex: @p convex_vertices + @p convex_faces (triangle triplets)
 * - @c kMesh: @p mesh_vertices + @p mesh_triangles (triangle triplets) for FCL BVH
 */
struct LinkCollisionShape {
  std::string link_name;
  LinkShapeKind kind = LinkShapeKind::kSphere;
  automsgs::msgs::geometry_msgs::Pose origin;
  double size_x = 0.04;  // sphere r | box lx | cylinder r
  double size_y = 0.04;  // box ly
  double size_z = 0.04;  // box lz | cylinder height
  std::vector<MeshVertex> convex_vertices;
  std::vector<int> convex_faces;  // triplets
  std::vector<MeshVertex> mesh_vertices;
  std::vector<int> mesh_triangles;  // triplets
};

/**
 * @brief Robot collision model parsed from URDF collision tags.
 *
 * Mesh policy:
 * 1. If `mesh.convexparts` lists part STLs → one @c kConvex per part (multi-convex).
 * 2. Else if online decompose enabled → VHACD (FEATURE) or AABB approx.
 * 3. Else keep triangle mesh as @c kMesh for FCL BVH.
 * 4. Multiple `<collision>` tags on one link → multiple shapes (multi-body).
 */
class LinkCollisionModel {
 public:
  bool LoadFromUrdf(const std::string& urdf_path, std::string* error = nullptr);

  /**
   * @brief When true and no `*.convexparts`, run online multi-convex (VHACD/AABB).
   * Production default remains offline sidecar / BVH mesh.
   */
  void SetEnableOnlineDecompose(bool enable) { enable_online_decompose_ = enable; }

  void SetDecomposeOptions(ConvexDecomposeOptions options) {
    decompose_options_ = std::move(options);
  }

  const std::vector<LinkCollisionShape>& Shapes() const { return shapes_; }

  bool empty() const { return shapes_.empty(); }

  /** @brief Append shapes (e.g. offline multi-convex parts). */
  void AppendShape(LinkCollisionShape shape) {
    shapes_.push_back(std::move(shape));
  }

 private:
  std::vector<LinkCollisionShape> shapes_;
  bool enable_online_decompose_ = false;
  ConvexDecomposeOptions decompose_options_;
};

bool BuildConvexHull(const std::vector<MeshVertex>& verts,
                     std::vector<MeshVertex>* hull_verts,
                     std::vector<int>* hull_faces);

/** @brief Load STL vertices only (dedup not applied). */
bool LoadStlVertices(const std::string& path,
                     std::vector<MeshVertex>* verts);

/**
 * @brief Load STL as indexed triangle mesh (binary keeps topology; ASCII fans).
 * @param[out] verts Vertex list.
 * @param[out] triangles Index triplets into @p verts.
 */
bool LoadStlMesh(const std::string& path, std::vector<MeshVertex>* verts,
                 std::vector<int>* triangles);

/**
 * @brief Load multi-convex sidecar: text file, one STL path per non-empty line.
 * Paths are relative to the sidecar directory.
 * @return Number of convex parts appended into @p out.
 */
int LoadConvexPartsSidecar(const std::string& sidecar_path,
                           const std::string& link_name,
                           const automsgs::msgs::geometry_msgs::Pose& origin,
                           std::vector<LinkCollisionShape>* out);

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
