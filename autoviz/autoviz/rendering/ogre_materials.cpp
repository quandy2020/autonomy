/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_materials.hpp"

#include <Ogre.h>

#include <cmath>
#include <string>
#include <vector>

namespace autoviz {
namespace rendering {
namespace {

constexpr char kPointSpriteTex[] = "AvizPointDiscTex";
constexpr char kPointSpriteMat[] = "AvizPointSprite";
constexpr char kPbrMat[] = "AvizPBR";
constexpr char kPbrTexturedMat[] = "AvizPBRTextured";

void CreatePointDiscTexture() {
  if (Ogre::TextureManager::getSingleton().resourceExists(kPointSpriteTex)) {
    return;
  }
  constexpr int kSize = 64;
  std::vector<Ogre::uint8> pixels(static_cast<std::size_t>(kSize * kSize * 4));
  const float center = (kSize - 1) * 0.5f;
  const float radius = center - 1.f;
  for (int y = 0; y < kSize; ++y) {
    for (int x = 0; x < kSize; ++x) {
      const float dx = static_cast<float>(x) - center;
      const float dy = static_cast<float>(y) - center;
      const float dist = std::sqrt(dx * dx + dy * dy);
      float alpha = 1.f;
      if (dist > radius) {
        alpha = 0.f;
      } else if (dist > radius - 1.5f) {
        alpha = (radius - dist) / 1.5f;
      }
      const std::size_t idx =
          static_cast<std::size_t>((y * kSize + x) * 4);
      pixels[idx + 0] = 255;
      pixels[idx + 1] = 255;
      pixels[idx + 2] = 255;
      pixels[idx + 3] = static_cast<Ogre::uint8>(alpha * 255.f);
    }
  }
  Ogre::TexturePtr texture =
      Ogre::TextureManager::getSingleton().createManual(
          kPointSpriteTex,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          Ogre::TEX_TYPE_2D, kSize, kSize, 0, Ogre::PF_R8G8B8A8,
          Ogre::TU_STATIC);
  Ogre::PixelBox box(static_cast<Ogre::uint32>(kSize),
                     static_cast<Ogre::uint32>(kSize), 1,
                     Ogre::PF_R8G8B8A8, pixels.data());
  texture->getBuffer()->blitFromMemory(box);
}

void CreatePointSpriteMaterial() {
  if (Ogre::MaterialManager::getSingleton().resourceExists(kPointSpriteMat)) {
    return;
  }
  CreatePointDiscTexture();
  Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(
          kPointSpriteMat,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
  pass->createTextureUnitState(kPointSpriteTex);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthWriteEnabled(false);
  pass->setLightingEnabled(false);
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
}

void CreateFixedFunctionFallback(const char* name, bool alpha_blend) {
  if (Ogre::MaterialManager::getSingleton().resourceExists(name)) {
    return;
  }
  Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(
          name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
  pass->setCullingMode(Ogre::CULL_NONE);
  if (alpha_blend) {
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }
}

void EnsurePbrMaterials() {
  // Preferred path: scripts in ogre_media (materials/scripts/aviz_pbr.material +
  // materials/glsl120/aviz_pbr.program) loaded by RenderSystem.
  if (!Ogre::MaterialManager::getSingleton().resourceExists(kPbrMat)) {
    CreateFixedFunctionFallback(kPbrMat, false);
  }
  if (!Ogre::MaterialManager::getSingleton().resourceExists(kPbrTexturedMat)) {
    CreateFixedFunctionFallback(kPbrTexturedMat, true);
  }
}

}  // namespace

void EnsureOgreMaterials(Ogre::SceneManager* /*scene*/) {
  CreatePointSpriteMaterial();
  EnsurePbrMaterials();
}

}  // namespace rendering
}  // namespace autoviz
