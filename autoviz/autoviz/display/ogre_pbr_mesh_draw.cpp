/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/ogre_pbr_mesh_draw.hpp"

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

bool drawPbrMeshesOgreOrGl(common::DisplayContext* context,
                           rendering::SceneOverlay& scene,
                           const std::string& display_name,
                           const std::vector<PbrMeshInstance>& meshes) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    std::vector<rendering::OgrePbrMeshInstance> ogre_meshes;
    ogre_meshes.reserve(meshes.size());
    for (const PbrMeshInstance& mesh : meshes) {
      ogre_meshes.push_back({mesh.mesh, mesh.transform(), mesh.color, mesh.metallic,
                             mesh.roughness});
    }
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayPbrMeshes(display_name, ogre_meshes);
    return true;
  }
#endif

  for (const PbrMeshInstance& instance : meshes) {
    scene.addTriangleMeshSolidPbr(instance.mesh, instance.transform(), instance.color,
                                  instance.metallic, instance.roughness);
  }
  return false;
}

bool drawPbrTexturedMeshesOgreOrGl(
    common::DisplayContext* context, rendering::SceneOverlay& scene,
    const std::string& display_name,
    const std::vector<PbrTexturedMeshInstance>& meshes) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    std::vector<rendering::OgrePbrTexturedMeshInstance> ogre_meshes;
    ogre_meshes.reserve(meshes.size());
    for (const PbrTexturedMeshInstance& mesh : meshes) {
      ogre_meshes.push_back({mesh.mesh, mesh.transform(), mesh.texture, mesh.tint,
                             mesh.metallic, mesh.roughness});
    }
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayPbrTexturedMeshes(display_name,
                                                          ogre_meshes);
    return true;
  }
#endif

  for (const PbrTexturedMeshInstance& instance : meshes) {
    scene.addTriangleMeshTexturedPbr(instance.mesh, instance.transform(),
                                     instance.texture, instance.tint,
                                     instance.metallic, instance.roughness);
  }
  return false;
}

}  // namespace display
}  // namespace autoviz
