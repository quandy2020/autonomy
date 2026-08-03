/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/ogre_mesh_draw.hpp"

#include "autoviz/common/display_context.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_scene_host.hpp"
#endif

namespace autoviz {
namespace display {
namespace {

void syncOgreDisplayVisibility(common::DisplayContext* context,
                               const std::string& display_name) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr &&
      context->active_display_visibility_bits != nullptr) {
    context->ogre_scene_host->setDisplayVisibilityBits(
        display_name, *context->active_display_visibility_bits);
  }
#endif
}

}  // namespace

bool drawMeshesOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name,
                        const std::vector<ColoredMeshInstance>& meshes) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    std::vector<rendering::OgreColoredMeshInstance> ogre_meshes;
    ogre_meshes.reserve(meshes.size());
    for (const auto& mesh : meshes) {
      ogre_meshes.push_back({mesh.mesh, mesh.transform(), mesh.color,
                             mesh.wireframe, mesh.pick_handle});
    }
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayMeshes(display_name, ogre_meshes);
    return true;
  }
#endif

  for (const auto& instance : meshes) {
    if (instance.wireframe) {
      scene.addTriangleMeshWireframe(instance.mesh, instance.transform(),
                                     instance.color);
    } else {
      scene.addTriangleMeshSolid(instance.mesh, instance.transform(),
                                 instance.color);
    }
  }
  return false;
}

}  // namespace display
}  // namespace autoviz
