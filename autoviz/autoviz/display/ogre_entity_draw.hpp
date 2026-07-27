/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "autoviz/display/ogre_mesh_draw.hpp"

namespace autoviz {
namespace common {
class DisplayContext;
}  // namespace common
namespace rendering {
class SceneOverlay;
}  // namespace rendering

namespace display {

/** Ogre Entity path (MeshManager + SceneNode) when ogre_scene_host is set. */
bool drawEntityMeshesOgreOrGl(common::DisplayContext* context,
                              rendering::SceneOverlay& scene,
                              const std::string& display_name,
                              const std::vector<ColoredMeshInstance>& meshes);

}  // namespace display
}  // namespace autoviz
