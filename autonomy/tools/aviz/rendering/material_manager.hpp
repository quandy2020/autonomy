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

#ifndef AVIZ_RENDERING__MATERIAL_MANAGER_HPP_
#define AVIZ_RENDERING__MATERIAL_MANAGER_HPP_

#include <OgreColourValue.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>

#include <string>

// Local visibility macro (previously from visibility_control.hpp)
#ifndef AVIZ_RENDERING_PUBLIC
#define AVIZ_RENDERING_PUBLIC
#endif

namespace aviz {
namespace rendering {

const float unit_alpha_threshold = 0.9998f;

class AVIZ_RENDERING_PUBLIC MaterialManager {
 public:
  static void createColorMaterial(const std::string& name, const Ogre::ColourValue& color, bool use_self_illumination);

  static void createDefaultColorMaterials();

  static Ogre::MaterialPtr createMaterialWithNoLighting(std::string name);

  static Ogre::MaterialPtr createMaterialWithLighting(std::string name);

  static Ogre::MaterialPtr createMaterialWithShadowsAndLighting(std::string name);

  static Ogre::MaterialPtr createMaterialWithShadowsAndNoLighting(std::string name);

  static void createDefaultMaterials();

  static void enableAlphaBlending(Ogre::MaterialPtr material, float alpha);

  static void enableAlphaBlending(Ogre::SceneBlendType& blending, bool& depth_write, float alpha);
};

}  // namespace rendering
}  // namespace aviz

#endif  // AVIZ_RENDERING__MATERIAL_MANAGER_HPP_
