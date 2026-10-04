/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_entity_draw.hpp
 * @brief Draw helper that uploads meshes as Ogre Entities (MeshManager path).
 *
 * Prefer this over @ref drawMeshesOgreOrGl when geometry should participate in
 * the scene graph as reusable @c Ogre::Entity instances (e.g. robot links,
 * repeated marker meshes) rather than one-shot ManualObjects.
 *
 * Falls back to SceneOverlay GL when @c ogre_scene_host is unset.
 *
 * @see ColoredMeshInstance
 * @see ogre_mesh_draw.hpp
 * @see RobotModelDisplay
 */

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

/**
 * @brief Draws colored meshes via Ogre Entity + SceneNode, or GL fallback.
 *
 * @param context Display context providing the Ogre scene host.
 * @param scene GL overlay used when Ogre is unavailable.
 * @param display_name Stable prefix for entity / node names.
 * @param meshes Mesh instances with transform, color, and optional pick handle.
 * @return @c true if the draw was accepted by a backend.
 *
 * @see ColoredMeshInstance
 * @see drawMeshesOgreOrGl()
 */
bool drawEntityMeshesOgreOrGl(common::DisplayContext* context,
                              rendering::SceneOverlay& scene,
                              const std::string& display_name,
                              const std::vector<ColoredMeshInstance>& meshes);

}  // namespace display
}  // namespace autoviz
