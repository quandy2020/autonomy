/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/collision/vhacd_decomposition.hpp"

#include <algorithm>
#include <cstdint>

#include "autonomy/common/logging.hpp"

#if defined(AUTONOMY_HAS_VHACD)
#if __has_include(<VHACD.h>)
#include <VHACD.h>
#elif __has_include(<vhacd/VHACD.h>)
#include <vhacd/VHACD.h>
#elif __has_include(<VHACD/VHACD.h>)
#include <VHACD/VHACD.h>
#else
#error "AUTONOMY_HAS_VHACD set but VHACD.h not found"
#endif
#endif

namespace autonomy {
namespace manipulation {
namespace collision {

bool HasVhacdBackend() {
#if defined(AUTONOMY_HAS_VHACD)
  return true;
#else
  return false;
#endif
}

int VhacdConvexDecomposition(const std::vector<MeshVertex>& verts,
                             const std::vector<int>& triangles,
                             const ConvexDecomposeOptions& options,
                             std::vector<LinkCollisionShape>* out,
                             const std::string& link_name,
                             const core::Transform& origin) {
#if !defined(AUTONOMY_HAS_VHACD)
  (void)verts;
  (void)triangles;
  (void)options;
  (void)out;
  (void)link_name;
  (void)origin;
  return 0;
#else
  if (!out || verts.size() < 4) {
    return 0;
  }
  std::vector<double> points;
  points.reserve(verts.size() * 3);
  for (const auto& v : verts) {
    points.push_back(v.x);
    points.push_back(v.y);
    points.push_back(v.z);
  }
  std::vector<uint32_t> tris;
  if (!triangles.empty()) {
    tris.reserve(triangles.size());
    for (int t : triangles) {
      if (t >= 0) {
        tris.push_back(static_cast<uint32_t>(t));
      }
    }
  } else {
    // Fan if no topology (degenerate); prefer real triangles.
    return 0;
  }
  if (tris.size() < 9 || tris.size() % 3 != 0) {
    return 0;
  }

  VHACD::IVHACD* iface = VHACD::CreateVHACD();
  if (!iface) {
    AWARN << "VHACD::CreateVHACD failed";
    return 0;
  }
  VHACD::IVHACD::Parameters params;
  params.m_maxConvexHulls = static_cast<uint32_t>(std::max(1, options.max_parts));
  const bool ok = iface->Compute(points.data(),
                                 static_cast<uint32_t>(verts.size()),
                                 tris.data(),
                                 static_cast<uint32_t>(tris.size() / 3), params);
  if (!ok) {
    iface->Clean();
    iface->Release();
    AWARN << "VHACD Compute failed for link " << link_name;
    return 0;
  }
  const std::size_t before = out->size();
  const uint32_t n = iface->GetNConvexHulls();
  for (uint32_t i = 0; i < n; ++i) {
    VHACD::IVHACD::ConvexHull hull;
    iface->GetConvexHull(i, hull);
    LinkCollisionShape shape;
    shape.link_name = link_name;
    shape.origin = origin;
    shape.kind = LinkShapeKind::kConvex;
    shape.convex_vertices.reserve(hull.m_nPoints);
    for (uint32_t p = 0; p < hull.m_nPoints; ++p) {
      MeshVertex mv;
      mv.x = hull.m_points[p * 3 + 0];
      mv.y = hull.m_points[p * 3 + 1];
      mv.z = hull.m_points[p * 3 + 2];
      shape.convex_vertices.push_back(mv);
    }
    shape.convex_faces.reserve(hull.m_nTriangles * 3);
    for (uint32_t t = 0; t < hull.m_nTriangles; ++t) {
      shape.convex_faces.push_back(static_cast<int>(hull.m_triangles[t * 3 + 0]));
      shape.convex_faces.push_back(static_cast<int>(hull.m_triangles[t * 3 + 1]));
      shape.convex_faces.push_back(static_cast<int>(hull.m_triangles[t * 3 + 2]));
    }
    if (shape.convex_vertices.size() >= 4 && shape.convex_faces.size() >= 9) {
      out->push_back(std::move(shape));
    }
  }
  iface->Clean();
  iface->Release();
  const int added = static_cast<int>(out->size() - before);
  AINFO << "VHACD parts=" << added << " for link " << link_name;
  return added;
#endif
}

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
