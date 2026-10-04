/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_materials.hpp
 * @brief Register Autoviz-specific Ogre materials (point sprites and PBR).
 *
 * Call once after the Ogre @c SceneManager exists so point-disc sprites and
 * PBR materials (@c AvizPBR / @c AvizPBRTextured from ogre_media scripts, or
 * fixed-function fallbacks) are available to @ref OgreSceneHost uploads.
 *
 * @see OgreMaterialManager
 * @see EnsureOgreMaterials()
 */

#pragma once

namespace Ogre {
class SceneManager;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @brief Ensures Autoviz Ogre materials exist (point sprite disc + PBR).
 *
 * Idempotent: safe to call every frame or on backend init. Point sprites are
 * built procedurally; PBR materials come from @c ogre_media GLSL120 scripts
 * when available, otherwise fixed-function stand-ins.
 *
 * @param scene Non-null Ogre scene manager whose resource group is active.
 *
 * @see OgreMaterialManager::ensureDefaultMaterials()
 * @see OgreMaterialManager::ensureAvizMediaMaterials()
 */
void EnsureOgreMaterials(Ogre::SceneManager* scene);

}  // namespace rendering
}  // namespace autoviz

