/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <string>

#include <OgreColourValue.h>
#include <OgreMaterial.h>

namespace autoviz {
namespace rendering {

/** rviz_rendering::MaterialManager subset for Autoviz Ogre backend. */
class OgreMaterialManager {
 public:
  static constexpr float kUnitAlphaThreshold = 0.9998f;

  static Ogre::MaterialPtr createMaterialWithNoLighting(const std::string& name);
  static Ogre::MaterialPtr createMaterialWithLighting(const std::string& name);
  static void enableAlphaBlending(Ogre::MaterialPtr material, float alpha);
  static void ensureDefaultMaterials();
  /** Load rviz ogre_media script materials (BaseWhiteNoLighting, point cloud, pick). */
  static void ensureRvizMediaMaterials();
  /** Programmatic stand-ins when ogre_media GLSL scripts are unavailable (Ogre != 1.12). */
  static void ensureStubRvizMaterials();
};

}  // namespace rendering
}  // namespace autoviz

#endif
