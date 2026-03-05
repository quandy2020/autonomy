/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/rendering/material_manager.hpp"

#include <OgreMaterial.h>
#include <OgreTechnique.h>

#include <string>

namespace aviz {
namespace rendering {

void MaterialManager::createColorMaterial(const std::string& name, const Ogre::ColourValue& color,
                                          bool use_self_illumination) {
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(name, "aviz_rendering");
  mat->setAmbient(color * 0.5f);
  mat->setDiffuse(color);
  if (use_self_illumination) {
    mat->setSelfIllumination(color);
  }
  mat->setLightingEnabled(true);
  mat->setReceiveShadows(false);
}

void MaterialManager::createDefaultColorMaterials() {
  createColorMaterial("AVIZ/Red", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), true);
  createColorMaterial("AVIZ/Green", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), true);
  createColorMaterial("AVIZ/Blue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), true);
  createColorMaterial("AVIZ/Cyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), true);
  createColorMaterial("AVIZ/ShadedRed", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), false);
  createColorMaterial("AVIZ/ShadedGreen", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), false);
  createColorMaterial("AVIZ/ShadedBlue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), false);
  createColorMaterial("AVIZ/ShadedCyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), false);
}

Ogre::MaterialPtr MaterialManager::createMaterialWithNoLighting(std::string name) {
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "aviz_rendering");
  material->setReceiveShadows(false);
  material->getTechnique(0)->setLightingEnabled(false);

  return material;
}

Ogre::MaterialPtr MaterialManager::createMaterialWithLighting(std::string name) {
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "aviz_rendering");
  material->setReceiveShadows(false);
  material->getTechnique(0)->setLightingEnabled(true);

  return material;
}

Ogre::MaterialPtr MaterialManager::createMaterialWithShadowsAndLighting(std::string name) {
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "aviz_rendering");
  material->getTechnique(0)->setLightingEnabled(true);

  return material;
}

Ogre::MaterialPtr MaterialManager::createMaterialWithShadowsAndNoLighting(std::string name) {
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "aviz_rendering");
  material->getTechnique(0)->setLightingEnabled(false);

  return material;
}

void MaterialManager::enableAlphaBlending(Ogre::MaterialPtr material, float alpha) {
  if (alpha < unit_alpha_threshold) {
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setDepthWriteEnabled(false);
  } else {
    material->setSceneBlending(Ogre::SBT_REPLACE);
    material->setDepthWriteEnabled(true);
  }
}

void MaterialManager::enableAlphaBlending(Ogre::SceneBlendType& blending, bool& depth_write, float alpha) {
  if (alpha < unit_alpha_threshold) {
    blending = Ogre::SBT_TRANSPARENT_ALPHA;
    depth_write = false;
  } else {
    blending = Ogre::SBT_REPLACE;
    depth_write = true;
  }
}

void MaterialManager::createDefaultMaterials() {
  auto retrieve_result =
      Ogre::MaterialManager::getSingleton().createOrRetrieve("BaseWhiteNoLighting", "aviz_rendering");
  Ogre::MaterialPtr material = retrieve_result.first.staticCast<Ogre::Material>();
  if (material.get()) {
    material->setLightingEnabled(false);
  }
}

}  // namespace rendering
}  // namespace aviz
