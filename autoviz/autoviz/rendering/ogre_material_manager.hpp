/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_material_manager.hpp
 * @brief Helpers to create and configure Autoviz / rviz Ogre materials.
 *
 * Subset of @c rviz_rendering::MaterialManager: unlit/lit materials, alpha
 * blending, and loading of @c ogre_media script materials (with stub fallbacks).
 *
 * @see EnsureOgreMaterials()
 * @see OgreShape
 * @see OgrePointCloud
 */

#pragma once

#include <string>

#include <OgreColourValue.h>
#include <OgreMaterial.h>

namespace autoviz {
namespace rendering {

/**
 * @class OgreMaterialManager
 * @brief Static factory for Autoviz Ogre materials.
 */
class OgreMaterialManager {
 public:
  /**
   * @brief Alpha above which blending may be disabled (treat as opaque).
   */
  static constexpr float kUnitAlphaThreshold = 0.9998f;

  /**
   * @brief Resource group for Autoviz-created materials (ManualObject default).
   */
  static const Ogre::String& resourceGroup();

  /**
   * @brief Creates (or retrieves) an unlit material by name.
   *
   * @param name Unique material resource name.
   * @return Material pointer suitable for ManualObject / Entity coloring.
   */
  static Ogre::MaterialPtr createMaterialWithNoLighting(const std::string& name);

  /**
   * @brief Creates (or retrieves) a lit material by name.
   *
   * @param name Unique material resource name.
   * @return Material pointer with lighting enabled.
   */
  static Ogre::MaterialPtr createMaterialWithLighting(const std::string& name);

  /**
   * @brief Configures scene blending based on @p alpha.
   *
   * Below @ref kUnitAlphaThreshold enables alpha blending; above treats as opaque.
   *
   * @param material Target material (must be non-null).
   * @param alpha Opacity in \[0, 1\].
   */
  static void enableAlphaBlending(Ogre::MaterialPtr material, float alpha);

  /**
   * @brief Ensures built-in Autoviz default materials exist.
   *
   * Idempotent; called during backend initialization.
   */
  static void ensureDefaultMaterials();

  /**
   * @brief Loads rviz @c ogre_media script materials when available.
   *
   * Includes BaseWhiteNoLighting, point-cloud, and pick schemes.
   */
  static void ensureAvizMediaMaterials();

  /**
   * @brief Installs programmatic stand-ins when GLSL scripts are unavailable.
   *
   * Used when Ogre version or media layout does not match rviz 1.12 scripts.
   */
  static void ensureStubAvizMaterials();
};

}  // namespace rendering
}  // namespace autoviz

