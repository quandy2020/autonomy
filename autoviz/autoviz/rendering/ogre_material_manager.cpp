/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_material_manager.hpp"

#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreResourceGroupManager.h>
#include <OgreTechnique.h>

#include <memory>

namespace autoviz {
namespace rendering {
namespace {

constexpr char kAvizResourceGroup[] = "aviz_rendering";

Ogre::MaterialPtr CreateBaseMaterial(const std::string& name, bool lighting,
                                    const Ogre::String& resource_group) {
  if (Ogre::MaterialManager::getSingleton().resourceExists(name, resource_group)) {
    return Ogre::MaterialManager::getSingleton().getByName(name, resource_group);
  }
  Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(name, resource_group);
  Ogre::Technique* technique = material->getNumTechniques() > 0
                                   ? material->getTechnique(0)
                                   : material->createTechnique();
  Ogre::Pass* pass = technique->getNumPasses() > 0 ? technique->getPass(0)
                                                   : technique->createPass();
  pass->setLightingEnabled(lighting);
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE | Ogre::TVC_AMBIENT);
  pass->setCullingMode(Ogre::CULL_NONE);
  return material;
}

Ogre::MaterialPtr CreateBaseMaterial(const std::string& name, bool lighting) {
  return CreateBaseMaterial(name, lighting, OgreMaterialManager::resourceGroup());
}

}  // namespace

const Ogre::String& OgreMaterialManager::resourceGroup() {
  return Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
}

Ogre::MaterialPtr OgreMaterialManager::createMaterialWithNoLighting(
    const std::string& name) {
  return CreateBaseMaterial(name, false);
}

Ogre::MaterialPtr OgreMaterialManager::createMaterialWithLighting(
    const std::string& name) {
  return CreateBaseMaterial(name, true);
}

void OgreMaterialManager::enableAlphaBlending(Ogre::MaterialPtr material,
                                            float alpha) {
  if (!material) {
    return;
  }
  Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
  if (alpha >= kUnitAlphaThreshold) {
    pass->setSceneBlending(Ogre::SBT_REPLACE);
    pass->setDepthWriteEnabled(true);
    return;
  }
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthWriteEnabled(false);
}

void OgreMaterialManager::ensureDefaultMaterials() {
  createMaterialWithNoLighting("Autoviz/FlatNoLighting");
  createMaterialWithNoLighting("Autoviz/LineNoLighting");
  createMaterialWithLighting("Autoviz/FlatLit");
  Ogre::MaterialPtr overlay_line = createMaterialWithNoLighting("Autoviz/OverlayLine");
  if (overlay_line) {
    Ogre::Pass* pass = overlay_line->getTechnique(0)->getPass(0);
    pass->setDepthCheckEnabled(false);
    pass->setDepthWriteEnabled(false);
  }
}

void OgreMaterialManager::ensureAvizMediaMaterials() {
  auto retrieve = Ogre::MaterialManager::getSingleton().createOrRetrieve(
      "BaseWhiteNoLighting", kAvizResourceGroup);
  if (auto material = std::dynamic_pointer_cast<Ogre::Material>(retrieve.first)) {
    material->setLightingEnabled(false);
  }
}

void OgreMaterialManager::ensureStubAvizMaterials() {
  CreateBaseMaterial("BaseWhiteNoLighting", false, kAvizResourceGroup);
  const char* point_cloud_materials[] = {
      "aviz/PointCloudPoint",     "aviz/PointCloudSquare",
      "aviz/PointCloudFlatSquare", "aviz/PointCloudSphere",
      "aviz/PointCloudTile",      "aviz/PointCloudBox",
  };
  for (const char* name : point_cloud_materials) {
    CreateBaseMaterial(name, false, Ogre::String(kAvizResourceGroup));
  }
}

}  // namespace rendering
}  // namespace autoviz

