/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_mesh_draw.hpp
 * @brief Draw helper for colored triangle meshes (Ogre ManualObject or GL).
 *
 * Uploads @ref ColoredMeshInstance batches as persistent Ogre ManualObjects
 * when @c ogre_scene_host is set; otherwise triangulates into
 * @ref rendering::SceneOverlay.
 *
 * For Entity-based reuse prefer @ref drawEntityMeshesOgreOrGl; for PBR /
 * textured shading prefer @ref ogre_pbr_mesh_draw.hpp.
 *
 * @see ColoredMeshInstance
 * @see ObjMesh
 * @see ogre_entity_draw.hpp
 * @see ogre_pbr_mesh_draw.hpp
 */

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

/**
 * @struct ColoredMeshInstance
 * @brief One mesh draw call: geometry, world transform, color, and pick id.
 */
struct ColoredMeshInstance {
  ObjMesh mesh;              /**< CPU triangle mesh in local space. */
  QMatrix4x4 transform;      /**< Local → fixed/world transform. */
  QColor color;              /**< Flat / tint color (alpha included). */
  bool wireframe = false;    /**< When @c true, edges instead of filled faces. */
  common::PickHandle pick_handle = common::kInvalidPickHandle; /**< Selection id. */
};

/**
 * @brief Draws colored meshes via Ogre ManualObject when available, else GL.
 *
 * @param context Display context providing the Ogre scene host.
 * @param scene GL overlay fallback.
 * @param display_name Stable prefix for ManualObject names.
 * @param meshes Instances to draw this frame.
 * @return @c true if a backend accepted the draw.
 *
 * @see ColoredMeshInstance
 * @see drawEntityMeshesOgreOrGl()
 * @see drawPbrMeshesOgreOrGl()
 */
bool drawMeshesOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name,
                        const std::vector<ColoredMeshInstance>& meshes);

}  // namespace display
}  // namespace autoviz
