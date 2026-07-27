/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QMatrix4x4>

#include "autoviz/display/obj_mesh.hpp"

#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {
class DisplayContext;
}  // namespace common
namespace rendering {
class SceneOverlay;
}  // namespace rendering

namespace display {

struct ColoredMeshInstance {
  ObjMesh mesh;
  QMatrix4x4 transform;
  QColor color;
  bool wireframe = false;
  common::PickHandle pick_handle = common::kInvalidPickHandle;
};

/** Ogre persistent mesh ManualObject when ogre_scene_host is set. */
bool drawMeshesOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name,
                        const std::vector<ColoredMeshInstance>& meshes);

}  // namespace display
}  // namespace autoviz
