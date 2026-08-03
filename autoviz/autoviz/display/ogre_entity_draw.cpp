/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/ogre_entity_draw.hpp"

#include "autoviz/common/display_context.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_mesh_loader.hpp"
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

bool drawEntityMeshesOgreOrGl(common::DisplayContext* context,
                              rendering::SceneOverlay& scene,
                              const std::string& display_name,
                              const std::vector<ColoredMeshInstance>& meshes) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    std::vector<rendering::OgreEntityInstance> entities;
    std::vector<ColoredMeshInstance> wireframe_fallback;
    entities.reserve(meshes.size());
    for (const auto& mesh : meshes) {
      if (mesh.wireframe) {
        wireframe_fallback.push_back(mesh);
        continue;
      }
      const std::string mesh_name =
          rendering::OgreMeshLoader::registerCachedObjMesh(mesh.mesh);
      if (mesh_name.empty()) {
        wireframe_fallback.push_back(mesh);
        continue;
      }
      entities.push_back(
          {mesh_name, mesh.transform(), mesh.color, mesh.pick_handle});
    }
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayEntities(display_name, entities);
    if (!wireframe_fallback.empty()) {
      drawMeshesOgreOrGl(context, scene, display_name + "/wire", wireframe_fallback);
    }
    return true;
  }
#endif
  return drawMeshesOgreOrGl(context, scene, display_name, meshes);
}

}  // namespace display
}  // namespace autoviz
