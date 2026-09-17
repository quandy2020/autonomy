/*
 * Copyright 2026 The Openbot Authors
 *
 * URDF collision parse + STL load + single-hull convex decomposition.
 */

#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"
#include "autonomy/manipulation/motion/collision/mesh_convex_decomposition.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <regex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "autonomy/common/logging.hpp"
#include "autolink/common/file.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {
namespace {

std::string NormalizePath(const std::string& url) {
  if (url.rfind("file://", 0) == 0) {
    return url.substr(7);
  }
  return url;
}

std::string JoinPath(const std::string& dir, const std::string& rel) {
  if (rel.empty()) {
    return dir;
  }
  if (rel[0] == '/' || (rel.size() > 1 && rel[1] == ':')) {
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

std::string ExtractAttr(const std::string& tag, const std::string& attr) {
  const std::regex pattern(attr + R"regex(="([^"]*)")regex");
  std::smatch match;
  if (std::regex_search(tag, match, pattern) && match.size() > 1) {
    return match[1].str();
  }
  return {};
}

bool ParseVec3(const std::string& text, double* x, double* y, double* z) {
  if (!x || !y || !z) {
    return false;
  }
  std::istringstream iss(text);
  if (!(iss >> *x >> *y >> *z)) {
    *x = *y = *z = 0.0;
    return false;
  }
  return true;
}

core::Transform OriginFromTag(const std::string& tag) {
  double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
  ParseVec3(ExtractAttr(tag, "xyz"), &x, &y, &z);
  ParseVec3(ExtractAttr(tag, "rpy"), &roll, &pitch, &yaw);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  core::Transform t;
  t.qw = cr * cp * cy + sr * sp * sy;
  t.qx = sr * cp * cy - cr * sp * sy;
  t.qy = cr * sp * cy + sr * cp * sy;
  t.qz = cr * cp * sy - sr * sp * cy;
  t.x = x;
  t.y = y;
  t.z = z;
  return t;
}

struct Vec3 {
  double x = 0, y = 0, z = 0;
};

Vec3 Sub(const Vec3& a, const Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
double Dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
double Norm(const Vec3& a) { return std::sqrt(Dot(a, a)); }
Vec3 Normalize(const Vec3& a) {
  const double n = Norm(a);
  return n > 1e-12 ? Vec3{a.x / n, a.y / n, a.z / n} : Vec3{0, 0, 1};
}

void AabbFaces(const std::vector<MeshVertex>& verts,
               std::vector<MeshVertex>* out_v,
               std::vector<int>* out_f) {
  double xmin = 1e9, ymin = 1e9, zmin = 1e9;
  double xmax = -1e9, ymax = -1e9, zmax = -1e9;
  for (const auto& v : verts) {
    xmin = std::min(xmin, v.x);
    ymin = std::min(ymin, v.y);
    zmin = std::min(zmin, v.z);
    xmax = std::max(xmax, v.x);
    ymax = std::max(ymax, v.y);
    zmax = std::max(zmax, v.z);
  }
  if (xmax < xmin) {
    xmin = ymin = zmin = -0.01;
    xmax = ymax = zmax = 0.01;
  }
  out_v->clear();
  out_v->push_back({xmin, ymin, zmin});
  out_v->push_back({xmax, ymin, zmin});
  out_v->push_back({xmax, ymax, zmin});
  out_v->push_back({xmin, ymax, zmin});
  out_v->push_back({xmin, ymin, zmax});
  out_v->push_back({xmax, ymin, zmax});
  out_v->push_back({xmax, ymax, zmax});
  out_v->push_back({xmin, ymax, zmax});
  out_f->clear();
  const int faces[12][3] = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6},
                            {0, 4, 5}, {0, 5, 1}, {1, 5, 6}, {1, 6, 2},
                            {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0}};
  for (const auto& f : faces) {
    out_f->push_back(f[0]);
    out_f->push_back(f[1]);
    out_f->push_back(f[2]);
  }
}

}  // namespace

bool BuildConvexHull(const std::vector<MeshVertex>& verts,
                     std::vector<MeshVertex>* hull_verts,
                     std::vector<int>* hull_faces) {
  if (!hull_verts || !hull_faces) {
    return false;
  }
  if (verts.size() < 4) {
    AabbFaces(verts.empty() ? std::vector<MeshVertex>{{0, 0, 0}} : verts,
              hull_verts, hull_faces);
    return true;
  }

  // Deduplicate.
  std::vector<Vec3> pts;
  pts.reserve(verts.size());
  for (const auto& v : verts) {
    bool dup = false;
    for (const auto& p : pts) {
      if (std::abs(p.x - v.x) < 1e-9 && std::abs(p.y - v.y) < 1e-9 &&
          std::abs(p.z - v.z) < 1e-9) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      pts.push_back({v.x, v.y, v.z});
    }
  }
  if (pts.size() < 4) {
    std::vector<MeshVertex> tmp;
    for (const auto& p : pts) {
      tmp.push_back({p.x, p.y, p.z});
    }
    AabbFaces(tmp, hull_verts, hull_faces);
    return true;
  }

  // Seed tetrahedron from extreme points.
  int i0 = 0;
  for (int i = 1; i < static_cast<int>(pts.size()); ++i) {
    if (pts[static_cast<std::size_t>(i)].x < pts[static_cast<std::size_t>(i0)].x) {
      i0 = i;
    }
  }
  int i1 = (i0 == 0) ? 1 : 0;
  double best = -1.0;
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    if (i == i0) {
      continue;
    }
    const double d = Norm(Sub(pts[static_cast<std::size_t>(i)],
                              pts[static_cast<std::size_t>(i0)]));
    if (d > best) {
      best = d;
      i1 = i;
    }
  }
  int i2 = 0;
  best = -1.0;
  const Vec3 e01 = Sub(pts[static_cast<std::size_t>(i1)],
                       pts[static_cast<std::size_t>(i0)]);
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    if (i == i0 || i == i1) {
      continue;
    }
    const double a =
        Norm(Cross(e01, Sub(pts[static_cast<std::size_t>(i)],
                            pts[static_cast<std::size_t>(i0)])));
    if (a > best) {
      best = a;
      i2 = i;
    }
  }
  if (best < 1e-12) {
    std::vector<MeshVertex> tmp;
    for (const auto& p : pts) {
      tmp.push_back({p.x, p.y, p.z});
    }
    AabbFaces(tmp, hull_verts, hull_faces);
    return true;
  }
  int i3 = 0;
  best = -1.0;
  const Vec3 n012 =
      Normalize(Cross(e01, Sub(pts[static_cast<std::size_t>(i2)],
                               pts[static_cast<std::size_t>(i0)])));
  for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
    if (i == i0 || i == i1 || i == i2) {
      continue;
    }
    const double h = std::abs(
        Dot(n012, Sub(pts[static_cast<std::size_t>(i)],
                      pts[static_cast<std::size_t>(i0)])));
    if (h > best) {
      best = h;
      i3 = i;
    }
  }
  if (best < 1e-12) {
    std::vector<MeshVertex> tmp;
    for (const auto& p : pts) {
      tmp.push_back({p.x, p.y, p.z});
    }
    AabbFaces(tmp, hull_verts, hull_faces);
    return true;
  }

  // Orient tetra so faces point outward.
  const Vec3 c = {
      0.25 * (pts[static_cast<std::size_t>(i0)].x +
              pts[static_cast<std::size_t>(i1)].x +
              pts[static_cast<std::size_t>(i2)].x +
              pts[static_cast<std::size_t>(i3)].x),
      0.25 * (pts[static_cast<std::size_t>(i0)].y +
              pts[static_cast<std::size_t>(i1)].y +
              pts[static_cast<std::size_t>(i2)].y +
              pts[static_cast<std::size_t>(i3)].y),
      0.25 * (pts[static_cast<std::size_t>(i0)].z +
              pts[static_cast<std::size_t>(i1)].z +
              pts[static_cast<std::size_t>(i2)].z +
              pts[static_cast<std::size_t>(i3)].z)};

  struct Face {
    int a, b, c;
  };
  auto make_face = [&](int a, int b, int cc) {
    Vec3 n = Cross(Sub(pts[static_cast<std::size_t>(b)],
                       pts[static_cast<std::size_t>(a)]),
                   Sub(pts[static_cast<std::size_t>(cc)],
                       pts[static_cast<std::size_t>(a)]));
    if (Dot(n, Sub(c, pts[static_cast<std::size_t>(a)])) > 0) {
      std::swap(b, cc);
    }
    return Face{a, b, cc};
  };

  std::vector<Face> faces;
  faces.push_back(make_face(i0, i1, i2));
  faces.push_back(make_face(i0, i2, i3));
  faces.push_back(make_face(i0, i3, i1));
  faces.push_back(make_face(i1, i3, i2));

  auto face_outward = [&](const Face& f) {
    return Normalize(Cross(
        Sub(pts[static_cast<std::size_t>(f.b)], pts[static_cast<std::size_t>(f.a)]),
        Sub(pts[static_cast<std::size_t>(f.c)], pts[static_cast<std::size_t>(f.a)])));
  };

  std::unordered_set<int> used = {i0, i1, i2, i3};
  for (int pi = 0; pi < static_cast<int>(pts.size()); ++pi) {
    if (used.count(pi)) {
      continue;
    }
    const Vec3& p = pts[static_cast<std::size_t>(pi)];
    std::vector<int> visible;
    for (int fi = 0; fi < static_cast<int>(faces.size()); ++fi) {
      const Face& f = faces[static_cast<std::size_t>(fi)];
      const Vec3 n = face_outward(f);
      const double d =
          Dot(n, Sub(p, pts[static_cast<std::size_t>(f.a)]));
      if (d > 1e-9) {
        visible.push_back(fi);
      }
    }
    if (visible.empty()) {
      continue;  // inside
    }
    // Horizon edges: edges of visible faces appearing once.
    std::unordered_map<std::uint64_t, int> edge_count;
    auto edge_key = [](int a, int b) {
      if (a > b) {
        std::swap(a, b);
      }
      return (static_cast<std::uint64_t>(a) << 32) |
             static_cast<std::uint64_t>(b);
    };
    for (int fi : visible) {
      const Face& f = faces[static_cast<std::size_t>(fi)];
      const int e[3][2] = {{f.a, f.b}, {f.b, f.c}, {f.c, f.a}};
      for (const auto& ed : e) {
        ++edge_count[edge_key(ed[0], ed[1])];
      }
    }
    std::vector<std::pair<int, int>> horizon;
    for (int fi : visible) {
      const Face& f = faces[static_cast<std::size_t>(fi)];
      const int e[3][2] = {{f.a, f.b}, {f.b, f.c}, {f.c, f.a}};
      for (const auto& ed : e) {
        if (edge_count[edge_key(ed[0], ed[1])] == 1) {
          horizon.emplace_back(ed[0], ed[1]);
        }
      }
    }
    // Remove visible (descending index).
    std::sort(visible.begin(), visible.end());
    visible.erase(std::unique(visible.begin(), visible.end()), visible.end());
    for (int k = static_cast<int>(visible.size()) - 1; k >= 0; --k) {
      faces.erase(faces.begin() + visible[static_cast<std::size_t>(k)]);
    }
    for (const auto& e : horizon) {
      faces.push_back(make_face(e.first, e.second, pi));
    }
    used.insert(pi);
  }

  // Compact vertex index map.
  std::unordered_map<int, int> remap;
  hull_verts->clear();
  for (const Face& f : faces) {
    for (int idx : {f.a, f.b, f.c}) {
      if (!remap.count(idx)) {
        const int ni = static_cast<int>(hull_verts->size());
        remap[idx] = ni;
        hull_verts->push_back({pts[static_cast<std::size_t>(idx)].x,
                               pts[static_cast<std::size_t>(idx)].y,
                               pts[static_cast<std::size_t>(idx)].z});
      }
    }
  }
  hull_faces->clear();
  for (const Face& f : faces) {
    hull_faces->push_back(remap[f.a]);
    hull_faces->push_back(remap[f.b]);
    hull_faces->push_back(remap[f.c]);
  }
  if (hull_verts->size() < 4 || hull_faces->empty()) {
    AabbFaces(verts, hull_verts, hull_faces);
  }
  return true;
}

bool LoadStlVertices(const std::string& path,
                     std::vector<MeshVertex>* verts) {
  std::vector<int> tris;
  return LoadStlMesh(path, verts, &tris);
}

bool LoadStlMesh(const std::string& path, std::vector<MeshVertex>* verts,
                 std::vector<int>* triangles) {
  if (!verts || !triangles) {
    return false;
  }
  verts->clear();
  triangles->clear();
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return false;
  }
  char header[6] = {};
  in.read(header, 5);
  in.seekg(0);
  const bool maybe_ascii =
      std::string(header, 5) == "solid" || std::string(header, 5) == "Solid";

  if (maybe_ascii) {
    std::string line;
    std::vector<MeshVertex> face_verts;
    while (std::getline(in, line)) {
      std::istringstream iss(line);
      std::string tok;
      iss >> tok;
      if (tok == "vertex") {
        MeshVertex v;
        if (iss >> v.x >> v.y >> v.z) {
          face_verts.push_back(v);
        }
      } else if (tok == "endfacet" && face_verts.size() >= 3) {
        const int base = static_cast<int>(verts->size());
        for (const auto& v : face_verts) {
          verts->push_back(v);
        }
        // Fan triangulation.
        for (std::size_t i = 1; i + 1 < face_verts.size(); ++i) {
          triangles->push_back(base);
          triangles->push_back(base + static_cast<int>(i));
          triangles->push_back(base + static_cast<int>(i + 1));
        }
        face_verts.clear();
      } else if (tok == "facet") {
        face_verts.clear();
      }
    }
    if (!verts->empty()) {
      return true;
    }
    in.clear();
    in.seekg(0);
  }

  char skip[80];
  in.read(skip, 80);
  std::uint32_t ntri = 0;
  in.read(reinterpret_cast<char*>(&ntri), 4);
  if (!in || ntri > 5000000u) {
    return !verts->empty();
  }
  verts->reserve(static_cast<std::size_t>(ntri) * 3u);
  triangles->reserve(static_cast<std::size_t>(ntri) * 3u);
  for (std::uint32_t i = 0; i < ntri; ++i) {
    float buf[12];
    in.read(reinterpret_cast<char*>(buf), 12 * 4);
    std::uint16_t attr = 0;
    in.read(reinterpret_cast<char*>(&attr), 2);
    if (!in) {
      break;
    }
    const int base = static_cast<int>(verts->size());
    for (int k = 0; k < 3; ++k) {
      verts->push_back({buf[3 + 3 * k], buf[4 + 3 * k], buf[5 + 3 * k]});
    }
    triangles->push_back(base);
    triangles->push_back(base + 1);
    triangles->push_back(base + 2);
  }
  return !verts->empty() && !triangles->empty();
}

int LoadConvexPartsSidecar(const std::string& sidecar_path,
                           const std::string& link_name,
                           const core::Transform& origin,
                           std::vector<LinkCollisionShape>* out) {
  if (!out) {
    return 0;
  }
  std::ifstream in(sidecar_path);
  if (!in.is_open()) {
    return 0;
  }
  const std::string dir = ::autolink::common::GetDirName(sidecar_path);
  int count = 0;
  std::string line;
  while (std::getline(in, line)) {
    // trim
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
      line.pop_back();
    }
    const auto start = line.find_first_not_of(" \t");
    if (start == std::string::npos || line[start] == '#') {
      continue;
    }
    const std::string rel = line.substr(start);
    const std::string part_path = JoinPath(dir, rel);
    std::vector<MeshVertex> verts;
    if (!LoadStlVertices(part_path, &verts) || verts.empty()) {
      AWARN << "convexparts: skip " << part_path;
      continue;
    }
    LinkCollisionShape shape;
    shape.link_name = link_name;
    shape.kind = LinkShapeKind::kConvex;
    shape.origin = origin;
    BuildConvexHull(verts, &shape.convex_vertices, &shape.convex_faces);
    out->push_back(std::move(shape));
    ++count;
  }
  return count;
}

bool LinkCollisionModel::LoadFromUrdf(const std::string& urdf_path,
                                      std::string* error) {
  shapes_.clear();
  if (urdf_path.empty()) {
    return true;
  }
  const std::string path = NormalizePath(urdf_path);
  std::ifstream input(path);
  if (!input.is_open()) {
    if (error) {
      *error = "cannot open URDF: " + path;
    }
    return false;
  }
  std::ostringstream oss;
  oss << input.rdbuf();
  const std::string xml = oss.str();
  const std::string base_dir = ::autolink::common::GetDirName(path);

  const std::regex link_re(R"regex(<link\b([^>]*)>([\s\S]*?)</link>)regex");
  for (std::sregex_iterator it(xml.begin(), xml.end(), link_re), end; it != end;
       ++it) {
    const std::string link_attrs = (*it)[1].str();
    const std::string body = (*it)[2].str();
    const std::string link_name = ExtractAttr(link_attrs, "name");
    if (link_name.empty()) {
      continue;
    }
    const std::regex coll_re(
        R"regex(<collision\b([^>]*)>([\s\S]*?)</collision>)regex");
    for (std::sregex_iterator cit(body.begin(), body.end(), coll_re), cend;
         cit != cend; ++cit) {
      const std::string coll_body = (*cit)[2].str();
      LinkCollisionShape shape;
      shape.link_name = link_name;
      shape.origin = core::IdentityTransform();

      const std::regex origin_re(R"regex(<origin\b([^>]*)/?>)regex");
      std::smatch om;
      if (std::regex_search(coll_body, om, origin_re)) {
        shape.origin = OriginFromTag(om[1].str());
      }

      const std::regex sphere_re(R"regex(<sphere\b([^>]*)/?>)regex");
      const std::regex box_re(R"regex(<box\b([^>]*)/?>)regex");
      const std::regex cyl_re(R"regex(<cylinder\b([^>]*)/?>)regex");
      const std::regex mesh_re(R"regex(<mesh\b([^>]*)/?>)regex");
      std::smatch sm;
      if (std::regex_search(coll_body, sm, sphere_re)) {
        shape.kind = LinkShapeKind::kSphere;
        const double r = std::stod(ExtractAttr(sm[1].str(), "radius").empty()
                                       ? "0.04"
                                       : ExtractAttr(sm[1].str(), "radius"));
        shape.size_x = std::max(1e-4, r);
        shapes_.push_back(std::move(shape));
      } else if (std::regex_search(coll_body, sm, box_re)) {
        shape.kind = LinkShapeKind::kBox;
        double sx = 0.08, sy = 0.08, sz = 0.08;
        ParseVec3(ExtractAttr(sm[1].str(), "size"), &sx, &sy, &sz);
        shape.size_x = std::max(1e-4, sx);
        shape.size_y = std::max(1e-4, sy);
        shape.size_z = std::max(1e-4, sz);
        shapes_.push_back(std::move(shape));
      } else if (std::regex_search(coll_body, sm, cyl_re)) {
        shape.kind = LinkShapeKind::kCylinder;
        const std::string ra = ExtractAttr(sm[1].str(), "radius");
        const std::string le = ExtractAttr(sm[1].str(), "length");
        shape.size_x = std::max(1e-4, ra.empty() ? 0.04 : std::stod(ra));
        shape.size_z = std::max(1e-4, le.empty() ? 0.08 : std::stod(le));
        shapes_.push_back(std::move(shape));
      } else if (std::regex_search(coll_body, sm, mesh_re)) {
        std::string filename = ExtractAttr(sm[1].str(), "filename");
        if (filename.rfind("package://", 0) == 0) {
          const auto slash = filename.find('/', 10);
          if (slash != std::string::npos) {
            filename = filename.substr(slash + 1);
          }
        }
        const std::string mesh_path = JoinPath(base_dir, filename);
        const int nparts = ResolveMultiConvexForMesh(
            mesh_path, link_name, shape.origin, enable_online_decompose_,
            decompose_options_, &shapes_);
        if (nparts > 0) {
          AINFO << "LinkCollisionModel multi-convex parts=" << nparts
                << " for " << mesh_path;
          continue;
        }
        std::vector<MeshVertex> verts;
        std::vector<int> tris;
        if (!LoadStlMesh(mesh_path, &verts, &tris) || verts.empty()) {
          AWARN << "LinkCollisionModel: mesh load failed " << mesh_path
                << "; using default sphere";
          shape.kind = LinkShapeKind::kSphere;
          shape.size_x = 0.04;
          shapes_.push_back(std::move(shape));
        } else {
          // True mesh → FCL BVH (when no sidecar / online parts).
          shape.kind = LinkShapeKind::kMesh;
          shape.mesh_vertices = std::move(verts);
          shape.mesh_triangles = std::move(tris);
          shapes_.push_back(std::move(shape));
        }
      }
    }
  }

  AINFO << "LinkCollisionModel loaded shapes=" << shapes_.size()
        << " from " << path;
  return true;
}

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
