/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_geometry.hpp"

#include "autoviz/rendering/scene_overlay.hpp"

#include <cmath>
#include <unordered_map>

namespace autoviz {
namespace rendering {
namespace {

struct QuantizedKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const QuantizedKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct QuantizedKeyHash {
  std::size_t operator()(const QuantizedKey& key) const {
    return static_cast<std::size_t>(key.x) ^
           (static_cast<std::size_t>(key.y) << 10) ^
           (static_cast<std::size_t>(key.z) << 20);
  }
};

QuantizedKey QuantizePosition(const QVector3D& position) {
  constexpr float kScale = 10000.f;
  return {static_cast<int>(std::lround(position.x() * kScale)),
          static_cast<int>(std::lround(position.y() * kScale)),
          static_cast<int>(std::lround(position.z() * kScale))};
}

}  // namespace

void BuildGroundGridLines(std::vector<OgreLineVertex>* out) {
  if (out == nullptr) {
    return;
  }
  constexpr int kHalfLines = 20;
  constexpr float kStep = 1.f;
  const QVector4D color(0.35f, 0.35f, 0.35f, 1.f);
  for (int i = -kHalfLines; i <= kHalfLines; ++i) {
    const float p = static_cast<float>(i) * kStep;
    out->push_back({QVector3D(-kHalfLines * kStep, p, 0.f),
                    color});
    out->push_back({QVector3D(kHalfLines * kStep, p, 0.f), color});
    out->push_back({QVector3D(p, -kHalfLines * kStep, 0.f), color});
    out->push_back({QVector3D(p, kHalfLines * kStep, 0.f), color});
  }
}

void BuildOriginAxisLines(std::vector<OgreLineVertex>* out) {
  if (out == nullptr) {
    return;
  }
  *out = {
      {QVector3D(0.f, 0.f, 0.f), QVector4D(1.f, 0.f, 0.f, 1.f)},
      {QVector3D(2.f, 0.f, 0.f), QVector4D(1.f, 0.f, 0.f, 1.f)},
      {QVector3D(0.f, 0.f, 0.f), QVector4D(0.f, 1.f, 0.f, 1.f)},
      {QVector3D(0.f, 2.f, 0.f), QVector4D(0.f, 1.f, 0.f, 1.f)},
      {QVector3D(0.f, 0.f, 0.f), QVector4D(0.f, 0.f, 1.f, 1.f)},
      {QVector3D(0.f, 0.f, 2.f), QVector4D(0.f, 0.f, 1.f, 1.f)},
  };
}

void AppendSceneOverlayLines(const SceneOverlay& overlay,
                             std::vector<OgreLineVertex>* out) {
  if (out == nullptr) {
    return;
  }
  for (const auto& vertex : overlay.lineVertices()) {
    out->push_back({vertex.position, vertex.color});
  }
}

void AppendSceneOverlayPoints(const SceneOverlay& overlay,
                              std::vector<OgreLineVertex>* out) {
  if (out == nullptr) {
    return;
  }
  for (const auto& vertex : overlay.pointVertices()) {
    out->push_back({vertex.position, vertex.color});
  }
}

void AppendSceneOverlayTriangles(const SceneOverlay& overlay,
                                 std::vector<OgreLineVertex>* out) {
  AppendSceneOverlayTriangles(overlay, QMatrix4x4(), out);
}

void AppendSceneOverlayTriangles(const SceneOverlay& overlay,
                                 const QMatrix4x4& view,
                                 std::vector<OgreLineVertex>* out) {
  if (out == nullptr) {
    return;
  }
  const auto triangles =
      view.isIdentity() ? overlay.triangleVertices()
                        : overlay.expandedTriangles(view);
  for (const auto& vertex : triangles) {
    out->push_back({vertex.position, vertex.color});
  }
}

void AppendSceneOverlayFlatTriangles(const SceneOverlay& overlay,
                                     const QMatrix4x4& view,
                                     std::vector<OgreLineVertex>* out) {
  if (out == nullptr) {
    return;
  }
  const auto triangles = overlay.expandedFlatTriangles(view);
  for (const auto& vertex : triangles) {
    out->push_back({vertex.position, vertex.color});
  }
}

void BuildPbrMeshFromPbrVertices(
    const std::vector<SceneOverlay::PbrVertex>& vertices,
    std::vector<OgrePbrVertex>* out) {
  if (out == nullptr) {
    return;
  }
  out->clear();
  out->reserve(vertices.size());
  for (const SceneOverlay::PbrVertex& vertex : vertices) {
    out->push_back({vertex.position, vertex.normal, vertex.albedo,
                    vertex.metallic, vertex.roughness});
  }
}

void BuildPbrMeshFromTriangles(const std::vector<OgreLineVertex>& triangles,
                               std::vector<OgrePbrVertex>* out) {
  if (out == nullptr || triangles.size() < 3) {
    return;
  }
  out->clear();
  out->reserve(triangles.size());

  std::unordered_map<QuantizedKey, QVector3D, QuantizedKeyHash> accumulated;
  std::unordered_map<QuantizedKey, int, QuantizedKeyHash> counts;

  for (std::size_t i = 0; i + 2 < triangles.size(); i += 3) {
    const QVector3D& a = triangles[i].position;
    const QVector3D& b = triangles[i + 1].position;
    const QVector3D& c = triangles[i + 2].position;
    QVector3D face_normal = QVector3D::crossProduct(b - a, c - a);
    if (face_normal.lengthSquared() < 1e-10f) {
      face_normal = QVector3D(0.f, 0.f, 1.f);
    } else {
      face_normal.normalize();
    }
    const QuantizedKey keys[3] = {QuantizePosition(a), QuantizePosition(b),
                                  QuantizePosition(c)};
    for (int j = 0; j < 3; ++j) {
      accumulated[keys[j]] += face_normal;
      counts[keys[j]] += 1;
    }
  }

  for (std::size_t i = 0; i + 2 < triangles.size(); i += 3) {
    const OgreLineVertex* tri[3] = {&triangles[i], &triangles[i + 1],
                                    &triangles[i + 2]};
    for (int j = 0; j < 3; ++j) {
      const QuantizedKey key = QuantizePosition(tri[j]->position);
      QVector3D normal = accumulated[key];
      if (normal.lengthSquared() < 1e-10f) {
        normal = QVector3D(0.f, 0.f, 1.f);
      } else {
        normal.normalize();
      }
      out->push_back({tri[j]->position, normal, tri[j]->color, 0.08f, 0.52f});
    }
  }
}

}  // namespace rendering
}  // namespace autoviz
