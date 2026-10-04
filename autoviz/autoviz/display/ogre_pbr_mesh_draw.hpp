/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_pbr_mesh_draw.hpp
 * @brief Draw helpers for PBR-shaded (and textured) triangle meshes.
 *
 * Uploads @ref PbrMeshInstance / @ref PbrTexturedMeshInstance batches through
 * the Ogre PBR material path when @c ogre_scene_host is set; otherwise falls
 * back to SceneOverlay GL with approximate shading.
 *
 * Primary consumer: @ref RobotModelDisplay when URDF materials / textures are
 * enabled.
 *
 * @see PbrMeshInstance
 * @see PbrTexturedMeshInstance
 * @see ObjMesh
 * @see RobotModelDisplay
 */

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

/**
 * @struct PbrMeshInstance
 * @brief Untextured mesh with metallic-roughness PBR parameters.
 */
struct PbrMeshInstance {
  ObjMesh mesh;              /**< Local-space geometry. */
  QMatrix4x4 transform;      /**< Local → world transform. */
  QColor color;              /**< Base color / albedo tint. */
  float metallic = 0.08f;    /**< Metallic factor in [0, 1]. */
  float roughness = 0.52f;   /**< Roughness factor in [0, 1]. */
};

/**
 * @struct PbrTexturedMeshInstance
 * @brief Textured mesh with metallic-roughness PBR parameters.
 */
struct PbrTexturedMeshInstance {
  ObjMesh mesh;              /**< Local-space geometry (UVs required). */
  QMatrix4x4 transform;      /**< Local → world transform. */
  QImage texture;            /**< Diffuse / albedo image. */
  QColor tint;               /**< Multiplicative tint over @c texture. */
  float metallic = 0.08f;    /**< Metallic factor in [0, 1]. */
  float roughness = 0.52f;   /**< Roughness factor in [0, 1]. */
};

/**
 * @brief Draws untextured PBR meshes via Ogre when available, else GL.
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param meshes PBR instances to draw.
 * @return @c true if a backend accepted the draw.
 *
 * @see PbrMeshInstance
 */
bool drawPbrMeshesOgreOrGl(common::DisplayContext* context,
                           rendering::SceneOverlay& scene,
                           const std::string& display_name,
                           const std::vector<PbrMeshInstance>& meshes);

/**
 * @brief Draws textured PBR meshes via Ogre when available, else GL.
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param meshes Textured PBR instances to draw.
 * @return @c true if a backend accepted the draw.
 *
 * @note Callers should ensure UVs exist (@ref ensureMeshTexcoords) before
 *       drawing textured instances.
 * @see PbrTexturedMeshInstance
 */
bool drawPbrTexturedMeshesOgreOrGl(
    common::DisplayContext* context, rendering::SceneOverlay& scene,
    const std::string& display_name,
    const std::vector<PbrTexturedMeshInstance>& meshes);

}  // namespace display
}  // namespace autoviz
