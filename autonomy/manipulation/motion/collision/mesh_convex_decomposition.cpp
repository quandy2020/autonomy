/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/collision/mesh_convex_decomposition.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/motion/collision/vhacd_convex_decomposition.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

std::string Dirname(const std::string& path) {
  const auto pos = path.find_last_of("/\\");
  return pos == std::string::npos ? std::string(".") : path.substr(0, pos);
}

std::string JoinPath(const std::string& dir, const std::string& rel) {
  if (rel.empty()) {
    return dir;
  }
  if (rel[0] == '/') {
    return rel;
  }
  if (dir.empty() || dir == ".") {
    return rel;
  }
  if (dir.back() == '/' || dir.back() == '\\') {
    return dir + rel;
  }
  return dir + "/" + rel;
}

struct Aabb {
  double minx = 1e9, miny = 1e9, minz = 1e9;
  double maxx = -1e9, maxy = -1e9, maxz = -1e9;
  void Expand(const MeshVertex& v) {
    minx = std::min(minx, v.x);
    miny = std::min(miny, v.y);
    minz = std::min(minz, v.z);
    maxx = std::max(maxx, v.x);
    maxy = std::max(maxy, v.y);
    maxz = std::max(maxz, v.z);
  }
  double Diag() const {
    const double dx = maxx - minx;
    const double dy = maxy - miny;
    const double dz = maxz - minz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  int LongestAxis() const {
    const double dx = maxx - minx;
    const double dy = maxy - miny;
    const double dz = maxz - minz;
    if (dx >= dy && dx >= dz) {
      return 0;
    }
    if (dy >= dz) {
      return 1;
    }
    return 2;
  }
  double Mid(int axis) const {
    if (axis == 0) {
      return 0.5 * (minx + maxx);
    }
    if (axis == 1) {
      return 0.5 * (miny + maxy);
    }
    return 0.5 * (minz + maxz);
  }
};

double Coord(const MeshVertex& v, int axis) {
  return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

void SplitIndices(const std::vector<MeshVertex>& verts,
                  const std::vector<int>& indices, int axis, double mid,
                  std::vector<int>* left, std::vector<int>* right) {
  left->clear();
  right->clear();
  for (int idx : indices) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= verts.size()) {
      continue;
    }
    if (Coord(verts[static_cast<std::size_t>(idx)], axis) <= mid) {
      left->push_back(idx);
    } else {
      right->push_back(idx);
    }
  }
  // Avoid empty child: put all on one side.
  if (left->empty() || right->empty()) {
    left->clear();
    right->clear();
    const std::size_t half = indices.size() / 2;
    for (std::size_t i = 0; i < indices.size(); ++i) {
      (i < half ? left : right)->push_back(indices[i]);
    }
  }
}

void CollectUniqueVerts(const std::vector<MeshVertex>& verts,
                        const std::vector<int>& indices,
                        std::vector<MeshVertex>* out) {
  out->clear();
  std::vector<char> seen(verts.size(), 0);
  for (int idx : indices) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= verts.size()) {
      continue;
    }
    if (seen[static_cast<std::size_t>(idx)]) {
      continue;
    }
    seen[static_cast<std::size_t>(idx)] = 1;
    out->push_back(verts[static_cast<std::size_t>(idx)]);
  }
}

void DecomposeRecursive(const std::vector<MeshVertex>& verts,
                        const std::vector<int>& indices,
                        const ConvexDecomposeOptions& options, int depth,
                        std::vector<LinkCollisionShape>* out,
                        const std::string& link_name,
                        const automsgs::msgs::geometry_msgs::Pose& origin) {
  if (indices.empty() ||
      static_cast<int>(out->size()) >= options.max_parts) {
    return;
  }
  std::vector<MeshVertex> subset;
  CollectUniqueVerts(verts, indices, &subset);
  if (subset.size() < 4) {
    return;
  }
  Aabb box;
  for (const auto& v : subset) {
    box.Expand(v);
  }
  const bool stop = depth >= options.max_depth ||
                    box.Diag() <= options.min_part_size ||
                    static_cast<int>(out->size()) + 1 >= options.max_parts;
  if (stop) {
    LinkCollisionShape shape;
    shape.link_name = link_name;
    shape.origin = origin;
    shape.kind = LinkShapeKind::kConvex;
    if (BuildConvexHull(subset, &shape.convex_vertices, &shape.convex_faces) &&
        shape.convex_vertices.size() >= 4) {
      out->push_back(std::move(shape));
    }
    return;
  }
  const int axis = box.LongestAxis();
  const double mid = box.Mid(axis);
  std::vector<int> left;
  std::vector<int> right;
  SplitIndices(verts, indices, axis, mid, &left, &right);
  DecomposeRecursive(verts, left, options, depth + 1, out, link_name, origin);
  DecomposeRecursive(verts, right, options, depth + 1, out, link_name, origin);
}

}  // namespace

int ApproximateConvexDecomposition(
    const std::vector<MeshVertex>& verts,
    const std::vector<int>& triangles, const ConvexDecomposeOptions& options,
    std::vector<LinkCollisionShape>* out, const std::string& link_name,
    const automsgs::msgs::geometry_msgs::Pose& origin) {
  if (!out || verts.size() < 4) {
    return 0;
  }
  const std::size_t before = out->size();
  std::vector<int> all_idx;
  if (!triangles.empty()) {
    all_idx = triangles;
  } else {
    all_idx.resize(verts.size());
    for (std::size_t i = 0; i < verts.size(); ++i) {
      all_idx[i] = static_cast<int>(i);
    }
  }
  DecomposeRecursive(verts, all_idx, options, 0, out, link_name, origin);
  if (out->size() == before) {
    // Fallback: single hull of all verts.
    LinkCollisionShape shape;
    shape.link_name = link_name;
    shape.origin = origin;
    shape.kind = LinkShapeKind::kConvex;
    if (BuildConvexHull(verts, &shape.convex_vertices, &shape.convex_faces)) {
      out->push_back(std::move(shape));
    }
  }
  return static_cast<int>(out->size() - before);
}

bool WriteConvexStl(const std::string& path,
                    const std::vector<MeshVertex>& hull_verts,
                    const std::vector<int>& hull_faces) {
  if (hull_verts.size() < 3 || hull_faces.size() < 9) {
    return false;
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }
  char header[80] = {};
  std::snprintf(header, sizeof(header), "autonomy convex part");
  out.write(header, 80);
  const uint32_t ntri = static_cast<uint32_t>(hull_faces.size() / 3);
  out.write(reinterpret_cast<const char*>(&ntri), 4);
  for (std::size_t i = 0; i + 2 < hull_faces.size(); i += 3) {
    const auto& a = hull_verts[static_cast<std::size_t>(hull_faces[i])];
    const auto& b = hull_verts[static_cast<std::size_t>(hull_faces[i + 1])];
    const auto& c = hull_verts[static_cast<std::size_t>(hull_faces[i + 2])];
    float n[3] = {0, 0, 0};
    float v[9] = {static_cast<float>(a.x), static_cast<float>(a.y),
                  static_cast<float>(a.z), static_cast<float>(b.x),
                  static_cast<float>(b.y), static_cast<float>(b.z),
                  static_cast<float>(c.x), static_cast<float>(c.y),
                  static_cast<float>(c.z)};
    out.write(reinterpret_cast<const char*>(n), 12);
    out.write(reinterpret_cast<const char*>(v), 36);
    uint16_t attr = 0;
    out.write(reinterpret_cast<const char*>(&attr), 2);
  }
  return static_cast<bool>(out);
}

bool WriteConvexPartsSidecar(const std::string& sidecar_path,
                             const std::vector<std::string>& part_stl_paths) {
  std::ofstream out(sidecar_path);
  if (!out) {
    return false;
  }
  out << "# autonomy multi-convex parts (one STL per line)\n";
  for (const auto& p : part_stl_paths) {
    out << p << "\n";
  }
  return true;
}

int ResolveMultiConvexForMesh(const std::string& mesh_path,
                              const std::string& link_name,
                              const automsgs::msgs::geometry_msgs::Pose& origin, bool enable_online,
                              const ConvexDecomposeOptions& options,
                              std::vector<LinkCollisionShape>* out) {
  if (!out) {
    return 0;
  }
  const std::string sidecar = mesh_path + ".convexparts";
  const int nparts =
      LoadConvexPartsSidecar(sidecar, link_name, origin, out);
  if (nparts > 0) {
    return nparts;
  }
  if (!enable_online) {
    return 0;
  }
  std::vector<MeshVertex> verts;
  std::vector<int> tris;
  if (!LoadStlMesh(mesh_path, &verts, &tris) || verts.size() < 4) {
    AWARN << "online convex: mesh load failed " << mesh_path;
    return 0;
  }
  // Prefer true VHACD when FEATURE linked.
  int n = VhacdConvexDecomposition(verts, tris, options, out, link_name,
                                   origin);
  if (n > 0) {
    return n;
  }
  n = ApproximateConvexDecomposition(verts, tris, options, out, link_name,
                                     origin);
  if (n > 0) {
    AINFO << "online multi-convex parts=" << n << " for " << mesh_path
          << " (approx AABB split)";
  }
  return n;
}

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
