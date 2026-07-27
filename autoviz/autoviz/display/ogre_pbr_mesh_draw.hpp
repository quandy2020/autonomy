/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QImage>
#include <QMatrix4x4>

#include "autoviz/display/obj_mesh.hpp"

namespace autoviz {
namespace common {
class DisplayContext;
}  // namespace common
namespace rendering {
class SceneOverlay;
}  // namespace rendering

namespace display {

struct PbrMeshInstance {
  ObjMesh mesh;
  QMatrix4x4 transform;
  QColor color;
  float metallic = 0.08f;
  float roughness = 0.52f;
};

struct PbrTexturedMeshInstance {
  ObjMesh mesh;
  QMatrix4x4 transform;
  QImage texture;
  QColor tint;
  float metallic = 0.08f;
  float roughness = 0.52f;
};

bool drawPbrMeshesOgreOrGl(common::DisplayContext* context,
                           rendering::SceneOverlay& scene,
                           const std::string& display_name,
                           const std::vector<PbrMeshInstance>& meshes);

bool drawPbrTexturedMeshesOgreOrGl(
    common::DisplayContext* context, rendering::SceneOverlay& scene,
    const std::string& display_name,
    const std::vector<PbrTexturedMeshInstance>& meshes);

}  // namespace display
}  // namespace autoviz
