/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>
#include <vector>

#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace rendering {

struct OgreLineVertex {
  QVector3D position;
  QVector4D color;
};

struct OgrePbrVertex {
  QVector3D position;
  QVector3D normal;
  QVector4D albedo;
  float metallic = 0.08f;
  float roughness = 0.52f;
};

void BuildGroundGridLines(std::vector<OgreLineVertex>* out);
void BuildOriginAxisLines(std::vector<OgreLineVertex>* out);
void AppendSceneOverlayLines(const SceneOverlay& overlay,
                             std::vector<OgreLineVertex>* out);
void AppendSceneOverlayPoints(const SceneOverlay& overlay,
                              std::vector<OgreLineVertex>* out);
void AppendSceneOverlayTriangles(const SceneOverlay& overlay,
                                 std::vector<OgreLineVertex>* out);
void AppendSceneOverlayTriangles(const SceneOverlay& overlay,
                                 const QMatrix4x4& view,
                                 std::vector<OgreLineVertex>* out);
void AppendSceneOverlayFlatTriangles(const SceneOverlay& overlay,
                                     const QMatrix4x4& view,
                                     std::vector<OgreLineVertex>* out);
void BuildPbrMeshFromPbrVertices(
    const std::vector<SceneOverlay::PbrVertex>& vertices,
    std::vector<OgrePbrVertex>* out);
/** @deprecated Flat-shaded overlay triangles only; prefer flat + PBR split. */
void BuildPbrMeshFromTriangles(const std::vector<OgreLineVertex>& triangles,
                               std::vector<OgrePbrVertex>* out);

}  // namespace rendering
}  // namespace autoviz
